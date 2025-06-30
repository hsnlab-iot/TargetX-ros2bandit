#include <dlfcn.h>
#include <iostream>
#include "rmw/rmw.h"
#include "rmw/init.h"
#include "rmw/init_options.h"
#include "rmw/qos_profiles.h"
#include "rmw/allocators.h"
#include "rmw/event.h"
#include "rmw/serialized_message.h"
#include "rmw/subscription_content_filter_options.h"
#include "rmw/time.h"
#include "rmw/names_and_types.h"
#include "rmw/names_and_types.h"
#include "rmw/network_flow_endpoint.h"
#include "rmw/serialized_message.h"
#include "rmw/qos_profiles.h"
#include "rmw/network_flow_endpoint_array.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rmw/get_topic_names_and_types.h"
#include "rcutils/types/string_array.h"
#include "rclcpp/rclcpp.hpp"

#include <unordered_map>
#include <string>
#include <mutex>
#include <queue>

//#include <std_msgs/msg/string.hpp>

#include <zmq.hpp>
#include <thread>
#include <map>
#include <cstdlib>  // rand()
#include <nlohmann/json.hpp>  // For JSON parsing

#include "rmw_bandit_typedefs.h"

#define VERSION "0.3.0"

#define LOAD_REAL_SYMBOL(sym) \
    real_##sym = (sym##_t)dlsym(rmw_lib_handle, #sym)

using json = nlohmann::json;

// Definitions (allocated here)
zmq::context_t global_zmq_ctx;  // Define global context
zmq::socket_t zmq_dealer_socket(global_zmq_ctx, zmq::socket_type::dealer); // Define global socket
bool broker_connected = false;
std::string broker_addr;

std::string bandit_domain;
std::string bandit_token;

std::vector<std::pair<std::string, bool>> published_topics; // Topic name and sent status
std::vector<std::pair<std::string, bool>> action_servers; // Action name and sent status
//std::vector<std::pair<std::string, bool>> action_clients; // Action name and sent status
std::unordered_map<std::string, const rosidl_message_type_support_t *> topic_type_support_map;
std::vector<std::pair<std::string, bool>> nodenames;
std::mutex queue_mutex;

std::atomic<bool> stop_threads(false);
std::atomic<bool> thread_zmqcb_running(false);
std::atomic<bool> thread_zmqcm_running(false);
std::atomic<bool> thread_stat_running(false);

std::mutex cv_mtx;
std::condition_variable cv;

const char *action_start_suffix = "/_action/send_goal";
const char *action_end_suffix = "/_action/get_result";

class TopicTrafficTracker {
private:
    std::queue<size_t> byte_history;  // Queue to store byte history
    std::queue<size_t> update_history;  // Queue to store update counts
    size_t current_bytes;  // Current bytes in the slot
    size_t current_updates;  // Current updates in the slot
    size_t byte_total;  // Total bytes in the last 3 slots
    size_t update_total;  // Total updates in the last 3 slots
    std::chrono::system_clock::time_point last_update_time;  // Last update time

    static constexpr long int SLOT_DURATION_MS = 1000;  // Slot duration in milliseconds
    static constexpr size_t MAX_SLOTS = 3;  // Maximum number of slots

    void advance_slot_if_needed() {
        auto now = std::chrono::system_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time).count();

        while (elapsed_time >= SLOT_DURATION_MS) {
            // Move to the next slot
            byte_history.push(current_bytes);
            update_history.push(current_updates);
            byte_total += current_bytes;
            update_total += current_updates;

            current_bytes = 0;
            current_updates = 0;
            last_update_time = last_update_time + std::chrono::milliseconds(SLOT_DURATION_MS);
            elapsed_time -= SLOT_DURATION_MS;
        }

        // Remove oldest slot if exceeding max slots
        if (byte_history.size() > MAX_SLOTS) {
            byte_total -= byte_history.front();
            update_total -= update_history.front();
            byte_history.pop();
            update_history.pop();
        }
    }

public:
    TopicTrafficTracker()
        : byte_history(), update_history(),
          current_bytes(0), current_updates(0), byte_total(0), update_total(0),
          last_update_time(std::chrono::system_clock::now()) {}

    void update(size_t bytes) {
        advance_slot_if_needed();
        current_bytes += bytes;
        current_updates++;
    }

    size_t get_bw() {
        advance_slot_if_needed();
        return byte_total / MAX_SLOTS;  // Average bandwidth over the last slots
    }

    size_t get_hz() {
        advance_slot_if_needed();
        return update_total / MAX_SLOTS;  // Average frequency over the last slots
    }
};

// Initialize a static blocked topics list
static std::unordered_set<std::string> blocked_topics;
static std::unordered_set<std::string> logged_topics;
static std::unordered_map<std::string, std::chrono::system_clock::time_point> trigger_topics;
static std::unordered_map<std::string, std::chrono::system_clock::time_point> actions;
static std::unordered_set<std::string> trigger_actions;

static std::unordered_map<std::string, TopicTrafficTracker> topic_traffic_map;


rclcpp::Logger logger = rclcpp::get_logger("rmw_bandit");

static void *rmw_lib_handle = nullptr;


std::string remove_suffix_if_exists(const char *input, const char *suffix) {
    if (!input || !suffix) {
        return std::string(); // Return empty string if input or suffix is null
    }

    size_t input_len = std::strlen(input);
    size_t suffix_len = std::strlen(suffix);

    if (suffix_len < input_len) {
        if (std::strcmp(input + input_len - suffix_len, suffix) == 0) {
            return std::string(input, input_len - suffix_len); // Remove suffix
        }
    }

    return std::string(); // Suffix not found
}

void zmq_send_message(nlohmann::json json,  zmq::send_flags flags = zmq::send_flags::none) {
    json["token"] = bandit_token;
    std::string message = json.dump();
    try {
        zmq_dealer_socket.send(zmq::buffer(message), flags);
        RCLCPP_DEBUG(logger, "Sent message: %s", message.c_str());
    } catch (const zmq::error_t &e) {
        RCLCPP_WARN(logger, "Failed to send message: %s", e.what());
    }
}

void zmq_send_publishers() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        for (auto& [topic, sent] : published_topics) {
            if (sent) continue;  // Skip if already sent

            RCLCPP_DEBUG(logger, "Sending publisher info for topic: %s", topic.c_str());
            nlohmann::json message = {{"type", "publisher"}, {"topic", topic}, {"domain", bandit_domain}};
            zmq_send_message(message);
            sent = true;  // Mark as sent
        }
    }
}

void zmq_send_actions() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        for (auto& [action, sent] : action_servers) {
            if (sent) continue;  // Skip if already sent
            RCLCPP_DEBUG(logger, "Sending action_servers info: %s", action.c_str());
            nlohmann::json message = {{"type", "action_server"}, {"action", action}, {"domain", bandit_domain}};
            zmq_send_message(message);
            sent = true;  // Mark as sent
        }
        /*
        for (auto& [action, sent] : action_clients) {
            if (sent) continue;  // Skip if already sent
            RCLCPP_DEBUG(logger, "Sending action_clients info: %s", action.c_str());
            nlohmann::json message = {{"type", "action_client"}, {"action", action}, {"domain", bandit_domain}};
            zmq_send_message(message);
            sent = true;  // Mark as sent
        }
        */
    }
}

void zmq_send_nodename() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        for (auto &[nodename, sent] : nodenames) {
            if (sent) continue;  // Skip if already sent

            RCLCPP_DEBUG(logger, "Sending nodename: %s", nodename.c_str());
            nlohmann::json message = {{"type", "nodename"}, {"nodename", nodename}, {"domain", bandit_domain}};
            zmq_send_message(message);
            sent = true;  // Mark as sent
        }
    }
}

void zmq_send_trigger(const std::string& topic) {
    if (!broker_connected) {
        // Do not send if it is not connected
        return;
    }
    RCLCPP_DEBUG(logger, "Sending trigger for topic: %s", topic.c_str());
    nlohmann::json message = {{"type", "trigger"}, {"topic", topic}};
    zmq_send_message(message);
}

void zmq_send_action_trigger(const std::string& action) {
    if (!broker_connected) {
        // Do not send if it is not connected
        return;
    }
    RCLCPP_DEBUG(logger, "Sending trigger for action: %s", action.c_str());
    nlohmann::json message = {{"type", "trigger"}, {"action", action}};
    zmq_send_message(message);
}

void zmq_set_callback() {
    // Start a thread to listen to the dealer socket and handle incoming messages
    std::thread([]() {
        thread_zmqcb_running.store(true);
        while (!stop_threads) {
            try {
                zmq::message_t message;
                auto recv_result = zmq_dealer_socket.recv(message, zmq::recv_flags::dontwait);
                if (!recv_result) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sleep to avoid busy waiting
                    // This could be normal, there is no message to receive
                    //RCLCPP_ERROR(logger, "Failed to receive message from dealer socket.");
                    continue;
                }

                std::string message_str(static_cast<char*>(message.data()), message.size());
                RCLCPP_DEBUG(logger, "Received message: %s", message_str.c_str());

                // Parse the message and call the appropriate callback
                try {
                    json parsed_message = json::parse(message_str);
                    if (parsed_message.contains("type")) {
                        std::string type = parsed_message["type"];
                        if (type == "block") {
                            std::string topic = parsed_message["topic"];
                            // Add the topic to the blocked list
                            blocked_topics.insert(topic);
                            RCLCPP_DEBUG(logger, "Blocking topic: %s", topic.c_str());
                        } else if (type == "unblock") {
                            std::string topic = parsed_message["topic"];
                            // Remove the topic from the blocked list
                            blocked_topics.erase(topic);
                            RCLCPP_DEBUG(logger, "Unblocking topic: %s", topic.c_str());
                        } else if (type == "watch") {
                            std::string topic = parsed_message.value("topic", "");
                            std::string action = parsed_message.value("action", "");
                            if (!topic.empty()) {
                                // Add the topic to the watch list
                                if (trigger_topics.find(topic) != trigger_topics.end()) {
                                    RCLCPP_WARN(logger, "Topic %s is already being watched.", topic.c_str());
                                    continue;
                                }
                                trigger_topics[topic] = std::chrono::system_clock::time_point();
                                RCLCPP_DEBUG(logger, "Watching topic: %s", topic.c_str());
                            } else if (!action.empty()) {
                                // Add the action to the watch list
                                if (trigger_actions.find(action) != trigger_actions.end()) {
                                    RCLCPP_WARN(logger, "Action %s is already being watched.", action.c_str());
                                    continue;
                                }
                                trigger_actions.insert(action);
                                // Check if the action is already in the actions map and recently triggered
                                auto now = std::chrono::system_clock::now();
                                if (actions.find(action) != actions.end()) {
                                    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - actions[action]).count();
                                    RCLCPP_DEBUG(logger, "Elapsed time since last trigger: %ld ms", elapsed_time);
                                    if (elapsed_time < 333) {
                                        zmq_send_action_trigger(action + ":start");
                                    }
                                }
                                RCLCPP_DEBUG(logger, "Watching action: %s", action.c_str());
                            }
                        } else if (type == "unwatch") {
                            std::string topic = parsed_message.value("topic", "");
                            std::string action = parsed_message.value("action", "");
                            if (!topic.empty()) {
                                // Remove the topic from the watch list
                                trigger_topics.erase(topic);
                                RCLCPP_DEBUG(logger, "Unwatching topic: %s", topic.c_str());
                            } else if (!action.empty()) {
                                // Remove the action from the watch list
                                trigger_actions.erase(action);
                                RCLCPP_DEBUG(logger, "Unwatching action: %s", action.c_str());
                            }
                        }
                        else {
                            RCLCPP_ERROR(logger, "Unknown message type: %s", type.c_str());
                        }
                    }
                } catch (const json::exception &e) {
                RCLCPP_ERROR(logger, "Failed to parse message: %s", e.what());
                }
            } catch (const zmq::error_t &e) {
                RCLCPP_ERROR(logger, "Dealer socket error: %s", e.what());
            }
        }
        thread_zmqcb_running.store(false);
    }).detach();
}

class connect_monitor_t : public zmq::monitor_t {
    public:
        void on_event_connected(const zmq_event_t& event,
                                const char* addr) override
        {
            (void) event;
            broker_connected = true;
            RCLCPP_INFO(logger, "Connected to %s", addr);

            zmq_send_publishers();
            zmq_send_nodename();
        }    
        void on_event_disconnected(const zmq_event_t& event, const char* addr) override
        {
            (void) event;
            broker_connected = false;
            RCLCPP_INFO(logger, "Disconnected from %s", addr);
            // Clear the sent status
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                for (auto &topic_pair : published_topics) {
                    topic_pair.second = false;  // Set the boolean to false
                }
                for (auto &topic_pair : nodenames) {
                    topic_pair.second = false;  // Set the boolean to false
                }
            }
        }
};


// Load function from the external RMW library
void load_rmw_library() {
    if (!rmw_lib_handle) {

        // Assign a temporary value to rmw_lib_handle to avoid concurrent calls
        static void *temp_handle = reinterpret_cast<void *>(0x1);
        rmw_lib_handle = temp_handle;

        const char* rmw_implementation_wrap = std::getenv("RMW_IMPLEMENTATION_WRAP");
        if (!rmw_implementation_wrap) {
            rmw_implementation_wrap = "rmw_fastrtps_cpp";
        }
        std::string library_name = "lib" + std::string(rmw_implementation_wrap) + ".so";

        rmw_lib_handle = dlopen(library_name.c_str(), RTLD_LAZY);
        if (!rmw_lib_handle) {
            RCLCPP_ERROR(logger, "Failed to load library '%s': %s", library_name.c_str(), dlerror());
            exit(EXIT_FAILURE);
        }

        // Load all RMW functions
        LOAD_REAL_SYMBOL(rmw_get_implementation_identifier);
        LOAD_REAL_SYMBOL(rmw_get_serialization_format);
        LOAD_REAL_SYMBOL(rmw_create_node);
        LOAD_REAL_SYMBOL(rmw_destroy_node);
        LOAD_REAL_SYMBOL(rmw_node_get_graph_guard_condition);
        LOAD_REAL_SYMBOL(rmw_init_publisher_allocation);
        LOAD_REAL_SYMBOL(rmw_fini_publisher_allocation);
        LOAD_REAL_SYMBOL(rmw_create_publisher);
        LOAD_REAL_SYMBOL(rmw_init_options_init);
        LOAD_REAL_SYMBOL(rmw_init_options_fini);
        LOAD_REAL_SYMBOL(rmw_init_options_copy);
        LOAD_REAL_SYMBOL(rmw_init);
        LOAD_REAL_SYMBOL(rmw_create_guard_condition);
        LOAD_REAL_SYMBOL(rmw_destroy_guard_condition);
        LOAD_REAL_SYMBOL(rmw_create_wait_set);
        LOAD_REAL_SYMBOL(rmw_destroy_wait_set);
        LOAD_REAL_SYMBOL(rmw_destroy_publisher);
        LOAD_REAL_SYMBOL(rmw_publisher_count_matched_subscriptions);
        LOAD_REAL_SYMBOL(rmw_publisher_assert_liveliness);
        LOAD_REAL_SYMBOL(rmw_publisher_wait_for_all_acked);
        LOAD_REAL_SYMBOL(rmw_publisher_get_actual_qos);
        LOAD_REAL_SYMBOL(rmw_borrow_loaned_message);
        LOAD_REAL_SYMBOL(rmw_return_loaned_message_from_publisher);
        LOAD_REAL_SYMBOL(rmw_create_service);
        LOAD_REAL_SYMBOL(rmw_destroy_service);
        LOAD_REAL_SYMBOL(rmw_service_request_subscription_get_actual_qos);
        LOAD_REAL_SYMBOL(rmw_service_response_publisher_get_actual_qos);
        LOAD_REAL_SYMBOL(rmw_trigger_guard_condition);
        LOAD_REAL_SYMBOL(rmw_get_gid_for_publisher);
        LOAD_REAL_SYMBOL(rmw_publisher_event_init);
        LOAD_REAL_SYMBOL(rmw_publish);
        LOAD_REAL_SYMBOL(rmw_create_subscription);
        LOAD_REAL_SYMBOL(rmw_destroy_subscription);
        LOAD_REAL_SYMBOL(rmw_subscription_get_actual_qos);
        LOAD_REAL_SYMBOL(rmw_subscription_event_init);
        LOAD_REAL_SYMBOL(rmw_publish_loaned_message);
        LOAD_REAL_SYMBOL(rmw_publish_serialized_message);
        LOAD_REAL_SYMBOL(rmw_get_serialized_message_size);
        LOAD_REAL_SYMBOL(rmw_serialize);
        LOAD_REAL_SYMBOL(rmw_deserialize);
        LOAD_REAL_SYMBOL(rmw_init_subscription_allocation);
        LOAD_REAL_SYMBOL(rmw_fini_subscription_allocation);
        LOAD_REAL_SYMBOL(rmw_subscription_count_matched_publishers);
        LOAD_REAL_SYMBOL(rmw_subscription_set_content_filter);
        LOAD_REAL_SYMBOL(rmw_subscription_get_content_filter);
        LOAD_REAL_SYMBOL(rmw_take);
        LOAD_REAL_SYMBOL(rmw_take_with_info);
        LOAD_REAL_SYMBOL(rmw_take_sequence);
        LOAD_REAL_SYMBOL(rmw_take_serialized_message);
        LOAD_REAL_SYMBOL(rmw_take_serialized_message_with_info);
        LOAD_REAL_SYMBOL(rmw_take_loaned_message);
        LOAD_REAL_SYMBOL(rmw_take_loaned_message_with_info);
        LOAD_REAL_SYMBOL(rmw_return_loaned_message_from_subscription);
        LOAD_REAL_SYMBOL(rmw_create_client);
        LOAD_REAL_SYMBOL(rmw_destroy_client);
        LOAD_REAL_SYMBOL(rmw_send_request);
        LOAD_REAL_SYMBOL(rmw_take_response);
        LOAD_REAL_SYMBOL(rmw_client_request_publisher_get_actual_qos);
        LOAD_REAL_SYMBOL(rmw_client_response_subscription_get_actual_qos);
        LOAD_REAL_SYMBOL(rmw_take_request);
        LOAD_REAL_SYMBOL(rmw_send_response);
        LOAD_REAL_SYMBOL(rmw_wait);
        LOAD_REAL_SYMBOL(rmw_get_node_names);
        LOAD_REAL_SYMBOL(rmw_get_node_names_with_enclaves);
        LOAD_REAL_SYMBOL(rmw_count_publishers);
        LOAD_REAL_SYMBOL(rmw_count_subscribers);
        LOAD_REAL_SYMBOL(rmw_count_clients);
        LOAD_REAL_SYMBOL(rmw_count_services);
        LOAD_REAL_SYMBOL(rmw_get_gid_for_client);
        LOAD_REAL_SYMBOL(rmw_compare_gids_equal);
        LOAD_REAL_SYMBOL(rmw_service_server_is_available);
        LOAD_REAL_SYMBOL(rmw_set_log_severity);
        LOAD_REAL_SYMBOL(rmw_subscription_set_on_new_message_callback);
        LOAD_REAL_SYMBOL(rmw_service_set_on_new_request_callback);
        LOAD_REAL_SYMBOL(rmw_client_set_on_new_response_callback);
        LOAD_REAL_SYMBOL(rmw_event_set_callback);
        LOAD_REAL_SYMBOL(rmw_shutdown);
        LOAD_REAL_SYMBOL(rmw_context_fini);
        LOAD_REAL_SYMBOL(rmw_take_event);
        LOAD_REAL_SYMBOL(rmw_get_publisher_names_and_types_by_node);
        LOAD_REAL_SYMBOL(rmw_get_subscriber_names_and_types_by_node);
        LOAD_REAL_SYMBOL(rmw_get_service_names_and_types_by_node);
        LOAD_REAL_SYMBOL(rmw_get_client_names_and_types_by_node);
        LOAD_REAL_SYMBOL(rmw_get_topic_names_and_types);
        LOAD_REAL_SYMBOL(rmw_get_service_names_and_types);
        LOAD_REAL_SYMBOL(rmw_get_publishers_info_by_topic);
        LOAD_REAL_SYMBOL(rmw_get_subscriptions_info_by_topic);
        LOAD_REAL_SYMBOL(rmw_qos_profile_check_compatible);
        LOAD_REAL_SYMBOL(rmw_publisher_get_network_flow_endpoints);
        LOAD_REAL_SYMBOL(rmw_subscription_get_network_flow_endpoints);

        RCLCPP_INFO(logger, "rmw_bandit %s is in action!", VERSION);
    }
}

extern "C" {

    const char *rmw_get_implementation_identifier() {
        load_rmw_library();
        return "rmw_bandit";  // Ensure this returns the correct identifier
        //return real_rmw_get_implementation_identifier();
    }

    const char *rmw_get_serialization_format(void) {
        load_rmw_library();
        return real_rmw_get_serialization_format();
    }

    rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t *wait_set) {
        load_rmw_library();
        return real_rmw_destroy_wait_set(wait_set);
    }

    rmw_ret_t rmw_destroy_node(rmw_node_t *node) {
        load_rmw_library();
        return real_rmw_destroy_node(node);
    }

    const rmw_guard_condition_t *rmw_node_get_graph_guard_condition(const rmw_node_t *node) {
        load_rmw_library();
        return real_rmw_node_get_graph_guard_condition(node);
    }

    rmw_ret_t rmw_init_publisher_allocation(const rosidl_message_type_support_t *type_support,
        const rosidl_runtime_c__Sequence__bound *message_bounds,
        rmw_publisher_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_init_publisher_allocation(type_support, message_bounds, allocation);
    }

    rmw_ret_t rmw_fini_publisher_allocation(rmw_publisher_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_fini_publisher_allocation(allocation);
    }

    rmw_publisher_t *rmw_create_publisher(
        const rmw_node_t *node,
        const rosidl_message_type_support_t *type_support,
        const char *topic_name,
        const rmw_qos_profile_t *qos_policies,
        const rmw_publisher_options_t *publisher_options
    ) {
        load_rmw_library();
        RCLCPP_INFO(logger, "Publisher initialized on topic: %s", topic_name);
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            published_topics.push_back(std::pair<std::string, bool>(topic_name, false));
            topic_type_support_map[topic_name] = type_support;  // Store the type support
            topic_traffic_map[std::string(topic_name)] = TopicTrafficTracker();  // Initialize traffic tracker for blocked
        }
        zmq_send_publishers();

        return real_rmw_create_publisher(node, type_support, topic_name, qos_policies, publisher_options);
    }

    rmw_ret_t rmw_init_options_init(rmw_init_options_t *init_options, rcutils_allocator_t allocator) {
        load_rmw_library();
        return real_rmw_init_options_init(init_options, allocator);
    }

    rmw_ret_t rmw_init_options_fini(rmw_init_options_t *init_options) {
        load_rmw_library();
        return real_rmw_init_options_fini(init_options);
    }

    rmw_ret_t rmw_init_options_copy(const rmw_init_options_t *src, rmw_init_options_t *dst) {
        load_rmw_library();
        return real_rmw_init_options_copy(src, dst);
    }

    rmw_ret_t rmw_init(const rmw_init_options_t *options, rmw_context_t *context) {
        const char* debug_env = std::getenv("ROS2BANDIT_DEBUG");
        bool debug = debug_env && std::strcmp(debug_env, "1") == 0;

        if (debug) {
            auto ret = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            (void) ret;
            RCLCPP_INFO(logger, "Debug mode enabled");
        }

        // This is not the first call on the rmw stack. 
        RCLCPP_DEBUG(logger, "RMW init");

        // Get ROS2BANDIT_BROKER and ROS2BANDIT_BROKER_PORT from environment variables
        const char* broker_env = std::getenv("ROS2BANDIT_BROKER");
        const char* broker_port_env = std::getenv("ROS2BANDIT_BROKER_PORT");
        const char* bandit_d_env = std::getenv("ROS2BANDIT_DOMAIN");
        const char* bandit_token_env = std::getenv("ROS2BANDIT_TOKEN");

        std::string broker = broker_env ? broker_env : "127.0.0.1";
        std::string broker_port = broker_port_env ? broker_port_env : "1884";
        bandit_domain = bandit_d_env ? bandit_d_env : "";
        bandit_token = bandit_token_env ? bandit_token_env : "";

        RCLCPP_INFO(logger, "Using broker: %s:%s", broker.c_str(), broker_port.c_str());
        broker_addr = "tcp://" + broker + ":" + broker_port;
        zmq_dealer_socket.connect(broker_addr); // Never returns error

        // Set callback for the dealer socket
        zmq_set_callback();

        std::thread([]() {
            connect_monitor_t monitor;
            monitor.init(zmq_dealer_socket, "inproc://monitor.sock");
            thread_zmqcm_running.store(true);
            while(!stop_threads) {
                monitor.check_event(1000);  // Check for events every 1000ms (events will return immediately)
            }
            thread_zmqcm_running.store(false);
        }).detach();
 
        // Start a thread to periodically display bandwidth and frequency information for all topics
        std::thread([]() {
            thread_stat_running.store(true);
            while (!stop_threads) {
                nlohmann::json topic_data_json;
                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    for (auto& [topic_name, tracker] : topic_traffic_map) {
                        size_t bw = tracker.get_bw();
                        size_t hz = tracker.get_hz();
                        RCLCPP_DEBUG(logger, "Topic: %s | Bandwidth: %zu bytes/sec | Frequency: %zu Hz", topic_name.c_str(), bw, hz);
                        
                        if (bw != 0 && hz != 0) {
                            // Add topic data to JSON object
                            
                            topic_data_json["topic"] = topic_name;
                            topic_data_json["bandwidth"] = bw;
                            topic_data_json["frequency"] = hz;
                            topic_data_json["blocked"] = blocked_topics.find(topic_name) != blocked_topics.end();
                            topic_data_json["type"] = "traffic";
                            zmq_send_message(topic_data_json);                                    }
                    }
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            thread_stat_running.store(false);
        }).detach();


        load_rmw_library();
        return real_rmw_init(options, context);
    }

    rmw_guard_condition_t *rmw_create_guard_condition(rmw_context_t *context) {
        load_rmw_library();
        return real_rmw_create_guard_condition(context);
    }

    rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t *guard_condition) {
        load_rmw_library();
        return real_rmw_destroy_guard_condition(guard_condition);
    }

    rmw_wait_set_t *rmw_create_wait_set(rmw_context_t *context, size_t max_conditions) {
        load_rmw_library();
        return real_rmw_create_wait_set(context, max_conditions);
    }

    rmw_node_t *rmw_create_node(rmw_context_t *context, const char *name, const char *namespace_) {

        load_rmw_library();
        RCLCPP_DEBUG(logger, "Creating node. Name: %s, Namespace: %s", name, namespace_);

        std::string nodename(name);
        // namespace_ starts with a slash
        if (!((namespace_[0] == '/') && (namespace_[1] == '\0'))) {
            nodename = std::string(namespace_) + std::string("/") + std::string(name);
        }

        RCLCPP_INFO(logger, "Node identifier set to: %s", nodename.c_str());
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            nodenames.push_back(std::pair<std::string, bool>(nodename, false));
        }
        zmq_send_nodename();

        return real_rmw_create_node(context, name, namespace_);
    }

    rmw_ret_t rmw_destroy_publisher(rmw_node_t *node, rmw_publisher_t *publisher) {
        load_rmw_library();
        return real_rmw_destroy_publisher(node, publisher);
    }

    rmw_ret_t rmw_publisher_get_actual_qos(const rmw_publisher_t *publisher, rmw_qos_profile_t *qos) {
        load_rmw_library();
        return real_rmw_publisher_get_actual_qos(publisher, qos);
    }

    rmw_ret_t rmw_publisher_count_matched_subscriptions(const rmw_publisher_t *publisher, size_t *subscription_count) {
        load_rmw_library();
        return real_rmw_publisher_count_matched_subscriptions(publisher, subscription_count);
    }

    rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t *publisher) {
        load_rmw_library();
        return real_rmw_publisher_assert_liveliness(publisher);
    }

    rmw_ret_t rmw_publisher_wait_for_all_acked(const rmw_publisher_t *publisher, rmw_time_t wait_timeout) {
        load_rmw_library();
        return real_rmw_publisher_wait_for_all_acked(publisher, wait_timeout);
    }

    rmw_ret_t rmw_borrow_loaned_message(const rmw_publisher_t *publisher,
                                        const rosidl_message_type_support_t *type_support,
                                        void **ros_message) {
        load_rmw_library();
        return real_rmw_borrow_loaned_message(publisher, type_support, ros_message);
    }

    rmw_ret_t rmw_return_loaned_message_from_publisher(const rmw_publisher_t *publisher, void *loaned_message) {
        load_rmw_library();
        return real_rmw_return_loaned_message_from_publisher(publisher, loaned_message);
    }

    rmw_service_t *rmw_create_service(
        const rmw_node_t *node,
        const rosidl_service_type_support_t *type_support,
        const char *service_name,
        const rmw_qos_profile_t *qos_policies) {
        load_rmw_library();
        RCLCPP_DEBUG(logger, "Creating service: %s", service_name);

        std::string action_name_start = remove_suffix_if_exists(service_name, action_start_suffix);
        std::string action_name_end = remove_suffix_if_exists(service_name, action_end_suffix);
        std::string action_name;
        if (!action_name_start.empty()) {
            action_name = action_name_start + ":start";
        } else if (!action_name_end.empty()) {
            action_name = action_name_end + ":end";
        }

        if (!action_name.empty()) {
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                action_servers.push_back(std::pair<std::string, bool>(action_name, false));
            }
            zmq_send_actions();
            RCLCPP_INFO(logger, "Action server created: %s", action_name.c_str());
        }

        return real_rmw_create_service(node, type_support, service_name, qos_policies);
    }


    rmw_ret_t rmw_destroy_service(rmw_node_t *node, rmw_service_t *service) {
        load_rmw_library();

        std::string action_name = remove_suffix_if_exists(service->service_name, action_start_suffix);
        if (!action_name.empty()) {
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                action_servers.erase(std::remove_if(action_servers.begin(), action_servers.end(),
                    [&action_name](const std::pair<std::string, bool>& pair) {
                        return pair.first == action_name;
                    }), action_servers.end());
            }
            // Send a message to the broker to indicate that the action client has been destroyed
            // ...
        }

        return real_rmw_destroy_service(node, service);
    }

    rmw_ret_t rmw_service_request_subscription_get_actual_qos(
        const rmw_service_t *service,
        rmw_qos_profile_t *qos) {
        load_rmw_library();
        return real_rmw_service_request_subscription_get_actual_qos(service, qos);
    }

    rmw_ret_t rmw_service_response_publisher_get_actual_qos(
        const rmw_service_t *service,
        rmw_qos_profile_t *qos) {
        load_rmw_library();
        return real_rmw_service_response_publisher_get_actual_qos(service, qos);
    }

    rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t *guard_condition) {
        load_rmw_library();
        return real_rmw_trigger_guard_condition(guard_condition);
    }

    rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t *publisher, rmw_gid_t *gid) {
        load_rmw_library();
        return real_rmw_get_gid_for_publisher(publisher, gid);
    }

    rmw_ret_t rmw_publisher_event_init(
        rmw_event_t *event,
        const rmw_publisher_t *publisher,
        rmw_event_type_t event_type) {
        load_rmw_library();
        return real_rmw_publisher_event_init(event, publisher, event_type);
    }

    rmw_ret_t rmw_publish(
        const rmw_publisher_t *publisher,
        const void *ros_message,
        rmw_publisher_allocation_t *allocation) {
        load_rmw_library();

        std::string topic_name = publisher->topic_name;

        // Skip special topics
        if (topic_name == "ros_discovery_info") {
            return real_rmw_publish(publisher, ros_message, allocation);
        }

        const rosidl_message_type_support_t * type_support = nullptr;
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            auto it = topic_type_support_map.find(topic_name);
            if (it != topic_type_support_map.end()) {
                type_support = it->second;
            } else {
                RCLCPP_ERROR(logger, "Topic type support not found for topic: %s", topic_name.c_str());
                return RMW_RET_ERROR;
            }
        }

        // Initialize buffer for serialization
        rcutils_uint8_array_t serialized = rcutils_get_zero_initialized_uint8_array();
        rcutils_allocator_t allocator = rcutils_get_default_allocator();
        rcutils_ret_t rcu_ret;
        rmw_ret_t rmw_ret;
        
        rcu_ret = rcutils_uint8_array_init(&serialized, 0, &allocator);
        if (rcu_ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(logger, "Failed to initialize serialized message buffer: %s", rcutils_get_error_string().str);
            return real_rmw_publish(publisher, ros_message, allocation);
        }

        // Serialize the message
        rmw_ret = real_rmw_serialize(ros_message, type_support, &serialized);
        if (rmw_ret != RMW_RET_OK) {
            RCLCPP_WARN(logger, "Failed to serialize message: %s", rmw_get_error_string().str);
            rcu_ret = rcutils_uint8_array_fini(&serialized);
            return real_rmw_publish(publisher, ros_message, allocation);
        }
        size_t serialized_size = serialized.buffer_length;

        bool blocked = (blocked_topics.find(topic_name) != blocked_topics.end());

        //RCLCPP_DEBUG(logger, "Publishing message of size: %zu on topic %s", serialized_size, topic_name.c_str());
        topic_traffic_map[topic_name].update(serialized_size);  // Update traffic tracker

        // Check if the topic is in the trigger list
        auto it = trigger_topics.find(topic_name);
        if (it != trigger_topics.end()) {
            auto now = std::chrono::system_clock::now();
            auto &last_trigger_time = it->second;

            // Check if the topic was triggered recently
            if (last_trigger_time.time_since_epoch().count() == 0) {
                last_trigger_time = now;  // Set the initial trigger time
            } else {
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_trigger_time);
                if (elapsed_time.count() > 333) {  // 1/3 seconds threshold
                    if (!blocked) zmq_send_trigger(topic_name);
                    last_trigger_time = now;  // Update the trigger time
                }
            }
        }

        // Check if the topic is in the blocked list
        bool logged = (logged_topics.find(topic_name) != logged_topics.end());

        if (blocked) {            
            if (!logged) {
                // Log the warning only once for each topic
                logged_topics.insert(topic_name);
                RCLCPP_INFO(logger, "Publishing to blocked topic: %s. Message will not be sent.", topic_name.c_str());
            }
            rcu_ret = rcutils_uint8_array_fini(&serialized);
            return RMW_RET_OK;  // Pretend publish succeeded without actually sending
        } else {
            if (logged) {
                // Remove the topic from the logged list if it is no longer blocked
                logged_topics.erase(topic_name);
                RCLCPP_INFO(logger, "Publishing to unblocked topic: %s. Message will be sent.", topic_name.c_str());
            }
        }

        // Send serialized message to rmw
        rmw_ret = real_rmw_publish_serialized_message(publisher, &serialized, allocation);
        rcu_ret = rcutils_uint8_array_fini(&serialized);

        if (rmw_ret != RMW_RET_OK) {
            RCLCPP_WARN(logger, "Failed to publish serialized message: %s", rmw_get_error_string().str);
            return real_rmw_publish(publisher, ros_message, allocation);
        }

        return rmw_ret;
    }

    rmw_subscription_t *rmw_create_subscription(
        const rmw_node_t *node,
        const rosidl_message_type_support_t *type_support,
        const char *topic_name,
        const rmw_qos_profile_t *qos_policies,
        const rmw_subscription_options_t *subscription_options) {
        load_rmw_library();
        return real_rmw_create_subscription(node, type_support, topic_name, qos_policies, subscription_options);
    }

    rmw_ret_t rmw_destroy_subscription(rmw_node_t *node, rmw_subscription_t *subscription) {
        load_rmw_library();
        return real_rmw_destroy_subscription(node, subscription);
    }

    rmw_ret_t rmw_subscription_get_actual_qos(
        const rmw_subscription_t *subscription,
        rmw_qos_profile_t *qos) {
        load_rmw_library();
        return real_rmw_subscription_get_actual_qos(subscription, qos);
    }

    rmw_ret_t rmw_subscription_event_init(
        rmw_event_t *event,
        const rmw_subscription_t *subscription,
        rmw_event_type_t event_type) {
        load_rmw_library();
        return real_rmw_subscription_event_init(event, subscription, event_type);
    }

    rmw_ret_t rmw_publish_loaned_message(
        const rmw_publisher_t *publisher,
        void *ros_message,
        rmw_publisher_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_publish_loaned_message(publisher, ros_message, allocation);
    }

    rmw_ret_t rmw_publish_serialized_message(
        const rmw_publisher_t *publisher,
        const rmw_serialized_message_t *serialized_message,
        rmw_publisher_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_publish_serialized_message(publisher, serialized_message, allocation);
    }

    rmw_ret_t rmw_get_serialized_message_size(
        const rosidl_message_type_support_t *type_support,
        const rosidl_runtime_c__Sequence__bound *message_bounds,
        size_t *size) {
        load_rmw_library();
        return real_rmw_get_serialized_message_size(type_support, message_bounds, size);
    }

    rmw_ret_t rmw_serialize(
        const void *ros_message,
        const rosidl_message_type_support_t *type_support,
        rmw_serialized_message_t *serialized_message) {
        load_rmw_library();
        return real_rmw_serialize(ros_message, type_support, serialized_message);
    }

    rmw_ret_t rmw_deserialize(
        const rmw_serialized_message_t *serialized_message,
        const rosidl_message_type_support_t *type_support,
        void *ros_message) {
        load_rmw_library();
        return real_rmw_deserialize(serialized_message, type_support, ros_message);
    }

    rmw_ret_t rmw_init_subscription_allocation(
        const rosidl_message_type_support_t *type_support,
        const rosidl_runtime_c__Sequence__bound *message_bounds,
        rmw_subscription_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_init_subscription_allocation(type_support, message_bounds, allocation);
    }

    rmw_ret_t rmw_fini_subscription_allocation(rmw_subscription_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_fini_subscription_allocation(allocation);
    }

    rmw_ret_t rmw_subscription_count_matched_publishers(
        const rmw_subscription_t *subscription,
        size_t *publisher_count) {
        load_rmw_library();
        return real_rmw_subscription_count_matched_publishers(subscription, publisher_count);
    }

    rmw_ret_t rmw_subscription_set_content_filter(
        rmw_subscription_t *subscription,
        const rmw_subscription_content_filter_options_t *options) {
        load_rmw_library();
        return real_rmw_subscription_set_content_filter(subscription, options);
    }

    rmw_ret_t rmw_subscription_get_content_filter(
        const rmw_subscription_t *subscription,
        rcutils_allocator_t *allocator,
        rmw_subscription_content_filter_options_t *options) {
        load_rmw_library();
        return real_rmw_subscription_get_content_filter(subscription, allocator, options);
    }

    rmw_ret_t rmw_take( // Consider rewriting as of rmw_take_with_info() !!!
        const rmw_subscription_t *subscription,
        void *ros_message,
        bool *taken,
        rmw_subscription_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_take(subscription, ros_message, taken, allocation);
    }

    rmw_ret_t rmw_take_with_info(
        const rmw_subscription_t *subscription,
        void *ros_message,
        bool *taken,
        rmw_message_info_t *message_info,
        rmw_subscription_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_take_with_info(subscription, ros_message, taken, message_info, allocation);
    }

    rmw_ret_t rmw_take_sequence(
        const rmw_subscription_t *subscription,
        size_t count,
        rmw_message_sequence_t *message_sequence,
        rmw_message_info_sequence_t *message_info_sequence,
        size_t *taken,
        rmw_subscription_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_take_sequence(subscription, count, message_sequence, message_info_sequence, taken, allocation);
    }

    rmw_ret_t rmw_take_serialized_message(
        const rmw_subscription_t *subscription,
        rmw_serialized_message_t *serialized_message,
        bool *taken,
        rmw_subscription_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_take_serialized_message(subscription, serialized_message, taken, allocation);
    }

    rmw_ret_t rmw_take_serialized_message_with_info(
        const rmw_subscription_t *subscription,
        rmw_serialized_message_t *serialized_message,
        bool *taken,
        rmw_message_info_t *message_info,
        rmw_subscription_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_take_serialized_message_with_info(subscription, serialized_message, taken, message_info, allocation);
    }

    rmw_ret_t rmw_take_loaned_message(
        const rmw_subscription_t *subscription,
        void **loaned_message,
        bool *taken,
        rmw_subscription_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_take_loaned_message(subscription, loaned_message, taken, allocation);
    }

    rmw_ret_t rmw_take_loaned_message_with_info(
        const rmw_subscription_t *subscription,
        void **loaned_message,
        bool *taken,
        rmw_message_info_t *message_info,
        rmw_subscription_allocation_t *allocation) {
        load_rmw_library();
        return real_rmw_take_loaned_message_with_info(subscription, loaned_message, taken, message_info, allocation);
    }

    rmw_ret_t rmw_return_loaned_message_from_subscription(
        const rmw_subscription_t *subscription,
        void *loaned_message) {
        load_rmw_library();
        return real_rmw_return_loaned_message_from_subscription(subscription, loaned_message);
    }

    rmw_client_t *rmw_create_client(
        const rmw_node_t *node,
        const rosidl_service_type_support_t *type_support,
        const char *service_name,
        const rmw_qos_profile_t *qos_policies) {
        RCLCPP_DEBUG(logger, "Create client %s", service_name);
        load_rmw_library();
        return real_rmw_create_client(node, type_support, service_name, qos_policies);
    }

    rmw_ret_t rmw_destroy_client(
        rmw_node_t *node,
        rmw_client_t *client) {
        RCLCPP_DEBUG(logger, "Destroying client %s", client->service_name);
        load_rmw_library();
        return real_rmw_destroy_client(node, client);
    }

    rmw_ret_t rmw_send_request(
        const rmw_client_t *client,
        const void *ros_request,
        int64_t *sequence_id) {
        RCLCPP_DEBUG(logger, "Request on %s", client->service_name);
        load_rmw_library();
        return real_rmw_send_request(client, ros_request, sequence_id);
    }

    rmw_ret_t rmw_take_response(
        const rmw_client_t *client,
        rmw_service_info_t *request_header,
        void *ros_response,
        bool *taken) {
        RCLCPP_DEBUG(logger, "Take response %s", client->service_name);
        load_rmw_library();
        return real_rmw_take_response(client, request_header, ros_response, taken);
    }

    rmw_ret_t rmw_client_request_publisher_get_actual_qos(
        const rmw_client_t *client,
        rmw_qos_profile_t *qos) {
        load_rmw_library();
        return real_rmw_client_request_publisher_get_actual_qos(client, qos);
    }

    rmw_ret_t rmw_client_response_subscription_get_actual_qos(
        const rmw_client_t *client,
        rmw_qos_profile_t *qos) {
        load_rmw_library();
        return real_rmw_client_response_subscription_get_actual_qos(client, qos);
    }

    rmw_ret_t rmw_take_request(
        const rmw_service_t *service,
        rmw_service_info_t *request_header,
        void *ros_request,
        bool *taken) {
        RCLCPP_DEBUG(logger, "Take request %s", service->service_name);
        load_rmw_library();
        return real_rmw_take_request(service, request_header, ros_request, taken);
    }

    rmw_ret_t rmw_send_response(
        const rmw_service_t *service,
        rmw_request_id_t *request_header,
        void *ros_response) {
        RCLCPP_DEBUG(logger, "Send response %s", service->service_name);
        load_rmw_library();

        std::string action_name_start = remove_suffix_if_exists(service->service_name, action_start_suffix);
        std::string action_name_end = remove_suffix_if_exists(service->service_name, action_end_suffix);
        std::string action_name;
        if (!action_name_start.empty()) {
            action_name = action_name_start + ":start";
        } else if (!action_name_end.empty()) {
            action_name = action_name_end + ":end";
        }
        if (!action_name.empty()) {
            RCLCPP_INFO(logger, "Action response sent: %s", action_name.c_str());
            actions[action_name] = std::chrono::system_clock::now();
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                if (trigger_actions.find(action_name) != trigger_actions.end()) {
                    zmq_send_action_trigger(action_name);
                }
            }
        }

        return real_rmw_send_response(service, request_header, ros_response);
    }

    rmw_ret_t rmw_wait(
        rmw_subscriptions_t *subscriptions,
        rmw_guard_conditions_t *guard_conditions,
        rmw_services_t *services,
        rmw_clients_t *clients,
        rmw_events_t *events,
        rmw_wait_set_t *wait_set,
        const rmw_time_t *wait_timeout) {
        load_rmw_library();
        return real_rmw_wait(subscriptions, guard_conditions, services, clients, events, wait_set, wait_timeout);
    }

    rmw_ret_t rmw_get_node_names(
        const rmw_node_t *node,
        rcutils_string_array_t *node_names,
        rcutils_string_array_t *node_namespaces) {
        load_rmw_library();
        return real_rmw_get_node_names(node, node_names, node_namespaces);
    }

    rmw_ret_t rmw_get_node_names_with_enclaves(
        const rmw_node_t *node,
        rcutils_string_array_t *node_names,
        rcutils_string_array_t *node_namespaces,
        rcutils_string_array_t *enclaves) {
        load_rmw_library();
        return real_rmw_get_node_names_with_enclaves(node, node_names, node_namespaces, enclaves);
    }

    rmw_ret_t rmw_count_publishers(
        const rmw_node_t *node,
        const char *topic_name,
        size_t *count) {
        load_rmw_library();
        return real_rmw_count_publishers(node, topic_name, count);
    }

    rmw_ret_t rmw_count_subscribers(
        const rmw_node_t *node,
        const char *topic_name,
        size_t *count) {
        load_rmw_library();
        return real_rmw_count_subscribers(node, topic_name, count);
    }

    rmw_ret_t rmw_count_clients(
        const rmw_node_t *node,
        const char *service_name,
        size_t *count) {
        load_rmw_library();
        return real_rmw_count_clients(node, service_name, count);
    }

    rmw_ret_t rmw_count_services(const rmw_node_t *node, const char *service_name, size_t *count) {
        load_rmw_library();
        return real_rmw_count_services(node, service_name, count);
    }

    rmw_ret_t rmw_get_gid_for_client(const rmw_client_t *client, rmw_gid_t *gid) {
        load_rmw_library();
        return real_rmw_get_gid_for_client(client, gid);
    }

    rmw_ret_t rmw_compare_gids_equal(const rmw_gid_t *gid1, const rmw_gid_t *gid2, bool *result) {
        load_rmw_library();
        return real_rmw_compare_gids_equal(gid1, gid2, result);
    }

    rmw_ret_t rmw_service_server_is_available(const rmw_node_t *node, const rmw_client_t *client, bool *is_available) {
        load_rmw_library();
        return real_rmw_service_server_is_available(node, client, is_available);
    }

    rmw_ret_t rmw_set_log_severity(rmw_log_severity_t severity) {
        load_rmw_library();
        return real_rmw_set_log_severity(severity);
    }

    rmw_ret_t rmw_subscription_set_on_new_message_callback(
        rmw_subscription_t *subscription, rmw_event_callback_t callback, const void *user_data) {
        load_rmw_library();
        return real_rmw_subscription_set_on_new_message_callback(subscription, callback, user_data);
    }

    rmw_ret_t rmw_service_set_on_new_request_callback(
        rmw_service_t *service, rmw_event_callback_t callback, const void *user_data) {
        load_rmw_library();
        return real_rmw_service_set_on_new_request_callback(service, callback, user_data);
    }

    rmw_ret_t rmw_client_set_on_new_response_callback(
        rmw_client_t *client, rmw_event_callback_t callback, const void *user_data) {
        load_rmw_library();
        return real_rmw_client_set_on_new_response_callback(client, callback, user_data);
    }

    rmw_ret_t rmw_event_set_callback(
        rmw_event_t *event, rmw_event_callback_t callback, const void *user_data) {
        load_rmw_library();
        return real_rmw_event_set_callback(event, callback, user_data);
    }
    //

    rmw_ret_t rmw_shutdown(rmw_context_t *context) {
        // Starting shut down, stop all threads
        RCLCPP_DEBUG(logger, "Shutdown");
        stop_threads = true;
        //load_rmw_library();
        return real_rmw_shutdown(context);
    }

    rmw_ret_t rmw_context_fini(rmw_context_t *context) {
        // This one is called last, after all other rmw functions
        RCLCPP_DEBUG(logger, "Context fini");
        while (thread_zmqcb_running.load() || thread_zmqcm_running.load() || thread_stat_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        zmq_dealer_socket.disconnect(broker_addr);
        zmq_dealer_socket.close();

        //load_rmw_library();
        return real_rmw_context_fini(context);
    }

    rmw_ret_t rmw_take_event(const rmw_event_t *event, void *event_info, bool *taken) {
        load_rmw_library();
        return real_rmw_take_event(event, event_info, taken);
    }

    rmw_ret_t rmw_get_publisher_names_and_types_by_node(
        const rmw_node_t *node,
        rcutils_allocator_t *allocator,
        const char *node_name,
        const char *node_namespace,
        bool no_demangle,
        rmw_names_and_types_t *names_and_types) {
        load_rmw_library();
        return real_rmw_get_publisher_names_and_types_by_node(node, allocator, node_name, node_namespace, no_demangle, names_and_types);
    }

    rmw_ret_t rmw_get_subscriber_names_and_types_by_node(
        const rmw_node_t *node,
        rcutils_allocator_t *allocator,
        const char *node_name,
        const char *node_namespace,
        bool no_demangle,
        rmw_names_and_types_t *names_and_types) {
        load_rmw_library();
        return real_rmw_get_subscriber_names_and_types_by_node(node, allocator, node_name, node_namespace, no_demangle, names_and_types);
    }

    rmw_ret_t rmw_get_service_names_and_types_by_node(
        const rmw_node_t *node,
        rcutils_allocator_t *allocator,
        const char *node_name,
        const char *node_namespace,
        rmw_names_and_types_t *names_and_types) {
        load_rmw_library();
        return real_rmw_get_service_names_and_types_by_node(node, allocator, node_name, node_namespace, names_and_types);
    }

    rmw_ret_t rmw_get_client_names_and_types_by_node(
        const rmw_node_t *node,
        rcutils_allocator_t *allocator,
        const char *node_name,
        const char *node_namespace,
        rmw_names_and_types_t *names_and_types) {
        load_rmw_library();
        return real_rmw_get_client_names_and_types_by_node(node, allocator, node_name, node_namespace, names_and_types);
    }

    rmw_ret_t rmw_get_topic_names_and_types(
        const rmw_node_t *node,
        rcutils_allocator_t *allocator,
        bool no_demangle,
        rmw_names_and_types_t *topic_names_and_types) {
        load_rmw_library();
        return real_rmw_get_topic_names_and_types(node, allocator, no_demangle, topic_names_and_types);
    }

    rmw_ret_t rmw_get_service_names_and_types(
        const rmw_node_t *node,
        rcutils_allocator_t *allocator,
        rmw_names_and_types_t *service_names_and_types) {
        load_rmw_library();
        return real_rmw_get_service_names_and_types(node, allocator, service_names_and_types);
    }

    rmw_ret_t rmw_get_publishers_info_by_topic(
        const rmw_node_t *node,
        rcutils_allocator_t *allocator,
        const char *topic_name,
        bool no_mangle,
        rmw_topic_endpoint_info_array_t *publishers_info) {
        load_rmw_library();
        return real_rmw_get_publishers_info_by_topic(node, allocator, topic_name, no_mangle, publishers_info);
    }

    rmw_ret_t rmw_get_subscriptions_info_by_topic(
        const rmw_node_t *node,
        rcutils_allocator_t *allocator,
        const char *topic_name,
        bool no_mangle,
        rmw_topic_endpoint_info_array_t *subscriptions_info) {
        load_rmw_library();
        return real_rmw_get_subscriptions_info_by_topic(node, allocator, topic_name, no_mangle, subscriptions_info);
    }

    rmw_ret_t rmw_qos_profile_check_compatible(
        const rmw_qos_profile_t qos_profile1,
        const rmw_qos_profile_t qos_profile2,
        rmw_qos_compatibility_type_t *compatibility,
        char *reason,
        size_t reason_size) {
        load_rmw_library();
        return real_rmw_qos_profile_check_compatible(qos_profile1, qos_profile2, compatibility, reason, reason_size);
    }

    rmw_ret_t rmw_publisher_get_network_flow_endpoints(
        const rmw_publisher_t *publisher,
        rcutils_allocator_t *allocator,
        rmw_network_flow_endpoint_array_t *network_flow_endpoints) {
        load_rmw_library();
        return real_rmw_publisher_get_network_flow_endpoints(publisher, allocator, network_flow_endpoints);
    }

    rmw_ret_t rmw_subscription_get_network_flow_endpoints(
        const rmw_subscription_t *subscription,
        rcutils_allocator_t *allocator,
        rmw_network_flow_endpoint_array_t *network_flow_endpoints) {
        load_rmw_library();
        return real_rmw_subscription_get_network_flow_endpoints(subscription, allocator, network_flow_endpoints);
    }
}
