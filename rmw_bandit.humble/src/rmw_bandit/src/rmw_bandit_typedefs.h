// Function pointers for dynamically loaded RMW functions
typedef const char* (*rmw_get_implementation_identifier_t)();
typedef const char* (*rmw_get_serialization_format_t)(void);
typedef rmw_node_t* (*rmw_create_node_t)(rmw_context_t *, const char *, const char *);
typedef rmw_ret_t (*rmw_destroy_node_t)(rmw_node_t *);
typedef const rmw_guard_condition_t* (*rmw_node_get_graph_guard_condition_t)(const rmw_node_t *);
typedef rmw_ret_t (*rmw_init_publisher_allocation_t)(const rosidl_message_type_support_t *, const rosidl_runtime_c__Sequence__bound *, rmw_publisher_allocation_t *);
typedef rmw_ret_t (*rmw_fini_publisher_allocation_t)(rmw_publisher_allocation_t *);
typedef rmw_publisher_t* (*rmw_create_publisher_t)(const rmw_node_t *, const rosidl_message_type_support_t *, const char *, const rmw_qos_profile_t *, const rmw_publisher_options_t *);
typedef rmw_ret_t (*rmw_init_options_init_t)(rmw_init_options_t *, rcutils_allocator_t);
typedef rmw_ret_t (*rmw_init_options_fini_t)(rmw_init_options_t *);
typedef rmw_ret_t (*rmw_init_options_copy_t)(const rmw_init_options_t *, rmw_init_options_t *);
typedef rmw_ret_t (*rmw_init_t)(const rmw_init_options_t *, rmw_context_t *);
typedef rmw_guard_condition_t* (*rmw_create_guard_condition_t)(rmw_context_t *);
typedef rmw_ret_t (*rmw_destroy_guard_condition_t)(rmw_guard_condition_t *);
typedef rmw_wait_set_t* (*rmw_create_wait_set_t)(rmw_context_t *, size_t);
typedef rmw_ret_t (*rmw_destroy_wait_set_t)(rmw_wait_set_t *);
typedef rmw_ret_t (*rmw_destroy_publisher_t)(rmw_node_t *, rmw_publisher_t *);
typedef rmw_ret_t (*rmw_publisher_count_matched_subscriptions_t)(const rmw_publisher_t *, size_t *);
typedef rmw_ret_t (*rmw_publisher_assert_liveliness_t)(const rmw_publisher_t *);
typedef rmw_ret_t (*rmw_publisher_wait_for_all_acked_t)(const rmw_publisher_t *, rmw_time_t);
typedef rmw_ret_t (*rmw_publisher_get_actual_qos_t)(const rmw_publisher_t *, rmw_qos_profile_t *);
typedef rmw_ret_t (*rmw_borrow_loaned_message_t)(const rmw_publisher_t *, const rosidl_message_type_support_t *, void **);
typedef rmw_ret_t (*rmw_return_loaned_message_from_publisher_t)(const rmw_publisher_t *, void *);
typedef rmw_service_t* (*rmw_create_service_t)(const rmw_node_t *, const rosidl_service_type_support_t *, const char *, const rmw_qos_profile_t *);
typedef rmw_ret_t (*rmw_destroy_service_t)(rmw_node_t *, rmw_service_t *);
typedef rmw_ret_t (*rmw_service_request_subscription_get_actual_qos_t)(const rmw_service_t *, rmw_qos_profile_t *);
typedef rmw_ret_t (*rmw_service_response_publisher_get_actual_qos_t)(const rmw_service_t *, rmw_qos_profile_t *);
typedef rmw_ret_t (*rmw_trigger_guard_condition_t)(const rmw_guard_condition_t *);
typedef rmw_ret_t (*rmw_get_gid_for_publisher_t)(const rmw_publisher_t *, rmw_gid_t *);
typedef rmw_ret_t (*rmw_publisher_event_init_t)(rmw_event_t *, const rmw_publisher_t *, rmw_event_type_t);
typedef rmw_ret_t (*rmw_publish_t)(const rmw_publisher_t *, const void *, rmw_publisher_allocation_t *);
typedef rmw_subscription_t* (*rmw_create_subscription_t)(const rmw_node_t *, const rosidl_message_type_support_t *, const char *, const rmw_qos_profile_t *, const rmw_subscription_options_t *);
typedef rmw_ret_t (*rmw_destroy_subscription_t)(rmw_node_t *, rmw_subscription_t *);
typedef rmw_ret_t (*rmw_subscription_get_actual_qos_t)(const rmw_subscription_t *, rmw_qos_profile_t *);
typedef rmw_ret_t (*rmw_subscription_event_init_t)(rmw_event_t *, const rmw_subscription_t *, rmw_event_type_t);
typedef rmw_ret_t (*rmw_publish_loaned_message_t)(const rmw_publisher_t *, void *, rmw_publisher_allocation_t *);
typedef rmw_ret_t (*rmw_publish_serialized_message_t)(const rmw_publisher_t *, const rmw_serialized_message_t *, rmw_publisher_allocation_t *);
typedef rmw_ret_t (*rmw_get_serialized_message_size_t)(const rosidl_message_type_support_t *, const rosidl_runtime_c__Sequence__bound *, size_t *);
typedef rmw_ret_t (*rmw_serialize_t)(const void *, const rosidl_message_type_support_t *, rmw_serialized_message_t *);
typedef rmw_ret_t (*rmw_deserialize_t)(const rmw_serialized_message_t *, const rosidl_message_type_support_t *, void *);
typedef rmw_ret_t (*rmw_init_subscription_allocation_t)(const rosidl_message_type_support_t *, const rosidl_runtime_c__Sequence__bound *, rmw_subscription_allocation_t *);
typedef rmw_ret_t (*rmw_fini_subscription_allocation_t)(rmw_subscription_allocation_t *);
typedef rmw_ret_t (*rmw_subscription_count_matched_publishers_t)(const rmw_subscription_t *, size_t *);
typedef rmw_ret_t (*rmw_subscription_set_content_filter_t)(rmw_subscription_t *, const rmw_subscription_content_filter_options_t *);
typedef rmw_ret_t (*rmw_subscription_get_content_filter_t)(const rmw_subscription_t *, rcutils_allocator_t *, rmw_subscription_content_filter_options_t *);
typedef rmw_ret_t (*rmw_take_t)(const rmw_subscription_t *, void *, bool *, rmw_subscription_allocation_t *);
typedef rmw_ret_t (*rmw_take_with_info_t)(const rmw_subscription_t *, void *, bool *, rmw_message_info_t *, rmw_subscription_allocation_t *);
typedef rmw_ret_t (*rmw_take_sequence_t)(const rmw_subscription_t *, size_t, rmw_message_sequence_t *, rmw_message_info_sequence_t *, size_t *, rmw_subscription_allocation_t *);
typedef rmw_ret_t (*rmw_take_serialized_message_t)(const rmw_subscription_t *, rmw_serialized_message_t *, bool *, rmw_subscription_allocation_t *);
typedef rmw_ret_t (*rmw_take_serialized_message_with_info_t)(const rmw_subscription_t *, rmw_serialized_message_t *, bool *, rmw_message_info_t *, rmw_subscription_allocation_t *);
typedef rmw_ret_t (*rmw_take_loaned_message_t)(const rmw_subscription_t *, void **, bool *, rmw_subscription_allocation_t *);
typedef rmw_ret_t (*rmw_take_loaned_message_with_info_t)(const rmw_subscription_t *, void **, bool *, rmw_message_info_t *, rmw_subscription_allocation_t *);
typedef rmw_ret_t (*rmw_return_loaned_message_from_subscription_t)(const rmw_subscription_t *, void *);
typedef rmw_client_t *(*rmw_create_client_t)(const rmw_node_t *, const rosidl_service_type_support_t *, const char *, const rmw_qos_profile_t *);
typedef rmw_ret_t (*rmw_destroy_client_t)(rmw_node_t *, rmw_client_t *);
typedef rmw_ret_t (*rmw_send_request_t)(const rmw_client_t *, const void *, int64_t *);
typedef rmw_ret_t (*rmw_take_response_t)(const rmw_client_t *, rmw_service_info_t *, void *, bool *);
typedef rmw_ret_t (*rmw_client_request_publisher_get_actual_qos_t)(const rmw_client_t *, rmw_qos_profile_t *);
typedef rmw_ret_t (*rmw_client_response_subscription_get_actual_qos_t)(const rmw_client_t *, rmw_qos_profile_t *);
typedef rmw_ret_t (*rmw_take_request_t)(const rmw_service_t *, rmw_service_info_t *, void *, bool *);
typedef rmw_ret_t (*rmw_send_response_t)(const rmw_service_t *, rmw_request_id_t *, void *);
typedef rmw_ret_t (*rmw_wait_t)(rmw_subscriptions_t *, rmw_guard_conditions_t *, rmw_services_t *, rmw_clients_t *, rmw_events_t *, rmw_wait_set_t *, const rmw_time_t *);
typedef rmw_ret_t (*rmw_get_node_names_t)(const rmw_node_t *, rcutils_string_array_t *, rcutils_string_array_t *);
typedef rmw_ret_t (*rmw_get_node_names_with_enclaves_t)(const rmw_node_t *, rcutils_string_array_t *, rcutils_string_array_t *, rcutils_string_array_t *);
typedef rmw_ret_t (*rmw_count_publishers_t)(const rmw_node_t *, const char *, size_t *);
typedef rmw_ret_t (*rmw_count_subscribers_t)(const rmw_node_t *, const char *, size_t *);
typedef rmw_ret_t (*rmw_count_clients_t)(const rmw_node_t *, const char *, size_t *);
typedef rmw_ret_t (*rmw_count_services_t)(const rmw_node_t *, const char *, size_t *);
typedef rmw_ret_t (*rmw_get_gid_for_client_t)(const rmw_client_t *, rmw_gid_t *);
typedef rmw_ret_t (*rmw_compare_gids_equal_t)(const rmw_gid_t *, const rmw_gid_t *, bool *);
typedef rmw_ret_t (*rmw_service_server_is_available_t)(const rmw_node_t *, const rmw_client_t *, bool *);
typedef rmw_ret_t (*rmw_set_log_severity_t)(rmw_log_severity_t);
typedef rmw_ret_t (*rmw_subscription_set_on_new_message_callback_t)(rmw_subscription_t *, rmw_event_callback_t, const void *);
typedef rmw_ret_t (*rmw_service_set_on_new_request_callback_t)(rmw_service_t *, rmw_event_callback_t, const void *);
typedef rmw_ret_t (*rmw_client_set_on_new_response_callback_t)(rmw_client_t *, rmw_event_callback_t, const void *);
typedef rmw_ret_t (*rmw_event_set_callback_t)(rmw_event_t *, rmw_event_callback_t, const void *);
typedef rmw_ret_t (*rmw_shutdown_t)(rmw_context_t *);
typedef rmw_ret_t (*rmw_context_fini_t)(rmw_context_t *);
typedef rmw_ret_t (*rmw_take_event_t)(const rmw_event_t *, void *, bool *);
typedef rmw_ret_t (*rmw_get_publisher_names_and_types_by_node_t)(const rmw_node_t *, rcutils_allocator_t *, const char *, const char *, bool, rmw_names_and_types_t *);
typedef rmw_ret_t (*rmw_get_subscriber_names_and_types_by_node_t)(const rmw_node_t *, rcutils_allocator_t *, const char *, const char *, bool, rmw_names_and_types_t *);
typedef rmw_ret_t (*rmw_get_service_names_and_types_by_node_t)(const rmw_node_t *, rcutils_allocator_t *, const char *, const char *, rmw_names_and_types_t *);
typedef rmw_ret_t (*rmw_get_client_names_and_types_by_node_t)(const rmw_node_t *, rcutils_allocator_t *, const char *, const char *, rmw_names_and_types_t *);
typedef rmw_ret_t (*rmw_get_topic_names_and_types_t)(const rmw_node_t *, rcutils_allocator_t *, bool, rmw_names_and_types_t *);
typedef rmw_ret_t (*rmw_get_service_names_and_types_t)(const rmw_node_t *, rcutils_allocator_t *, rmw_names_and_types_t *);
typedef rmw_ret_t (*rmw_get_publishers_info_by_topic_t)(const rmw_node_t *, rcutils_allocator_t *, const char *, bool, rmw_topic_endpoint_info_array_t *);
typedef rmw_ret_t (*rmw_get_subscriptions_info_by_topic_t)(const rmw_node_t *, rcutils_allocator_t *, const char *, bool, rmw_topic_endpoint_info_array_t *);
typedef rmw_ret_t (*rmw_qos_profile_check_compatible_t)(const rmw_qos_profile_t, const rmw_qos_profile_t, rmw_qos_compatibility_type_t *, char *, size_t);
typedef rmw_ret_t (*rmw_publisher_get_network_flow_endpoints_t)(const rmw_publisher_t *, rcutils_allocator_t *, rmw_network_flow_endpoint_array_t *);
typedef rmw_ret_t (*rmw_subscription_get_network_flow_endpoints_t)(const rmw_subscription_t *, rcutils_allocator_t *, rmw_network_flow_endpoint_array_t *);

// Declare function pointers
#define SET_FUNCTION_POINTER(name) \
    static name##_t real_##name = nullptr;

SET_FUNCTION_POINTER(rmw_get_implementation_identifier)
SET_FUNCTION_POINTER(rmw_get_serialization_format)
SET_FUNCTION_POINTER(rmw_create_node)
SET_FUNCTION_POINTER(rmw_destroy_node)
SET_FUNCTION_POINTER(rmw_node_get_graph_guard_condition)
SET_FUNCTION_POINTER(rmw_init_publisher_allocation)
SET_FUNCTION_POINTER(rmw_fini_publisher_allocation)
SET_FUNCTION_POINTER(rmw_create_publisher)
SET_FUNCTION_POINTER(rmw_init_options_init)
SET_FUNCTION_POINTER(rmw_init_options_fini)
SET_FUNCTION_POINTER(rmw_init_options_copy)
SET_FUNCTION_POINTER(rmw_init)
SET_FUNCTION_POINTER(rmw_create_guard_condition)
SET_FUNCTION_POINTER(rmw_destroy_guard_condition)
SET_FUNCTION_POINTER(rmw_create_wait_set)
SET_FUNCTION_POINTER(rmw_destroy_wait_set)
SET_FUNCTION_POINTER(rmw_publisher_count_matched_subscriptions)
SET_FUNCTION_POINTER(rmw_publisher_assert_liveliness)
SET_FUNCTION_POINTER(rmw_publisher_wait_for_all_acked)
SET_FUNCTION_POINTER(rmw_publisher_get_actual_qos)
SET_FUNCTION_POINTER(rmw_borrow_loaned_message)
SET_FUNCTION_POINTER(rmw_return_loaned_message_from_publisher)
SET_FUNCTION_POINTER(rmw_destroy_publisher)
SET_FUNCTION_POINTER(rmw_create_service)
SET_FUNCTION_POINTER(rmw_destroy_service)
SET_FUNCTION_POINTER(rmw_service_request_subscription_get_actual_qos)
SET_FUNCTION_POINTER(rmw_service_response_publisher_get_actual_qos)
SET_FUNCTION_POINTER(rmw_trigger_guard_condition)
SET_FUNCTION_POINTER(rmw_get_gid_for_publisher)
SET_FUNCTION_POINTER(rmw_publisher_event_init)
SET_FUNCTION_POINTER(rmw_publish)
SET_FUNCTION_POINTER(rmw_create_subscription)
SET_FUNCTION_POINTER(rmw_destroy_subscription)
SET_FUNCTION_POINTER(rmw_subscription_get_actual_qos)
SET_FUNCTION_POINTER(rmw_subscription_event_init)
SET_FUNCTION_POINTER(rmw_publish_loaned_message)
SET_FUNCTION_POINTER(rmw_publish_serialized_message)
SET_FUNCTION_POINTER(rmw_get_serialized_message_size)
SET_FUNCTION_POINTER(rmw_serialize)
SET_FUNCTION_POINTER(rmw_deserialize)
SET_FUNCTION_POINTER(rmw_init_subscription_allocation)
SET_FUNCTION_POINTER(rmw_fini_subscription_allocation)
SET_FUNCTION_POINTER(rmw_subscription_count_matched_publishers)
SET_FUNCTION_POINTER(rmw_subscription_set_content_filter)
SET_FUNCTION_POINTER(rmw_subscription_get_content_filter)
SET_FUNCTION_POINTER(rmw_take)
SET_FUNCTION_POINTER(rmw_take_with_info)
SET_FUNCTION_POINTER(rmw_take_sequence)
SET_FUNCTION_POINTER(rmw_take_serialized_message)
SET_FUNCTION_POINTER(rmw_take_serialized_message_with_info)
SET_FUNCTION_POINTER(rmw_take_loaned_message)
SET_FUNCTION_POINTER(rmw_take_loaned_message_with_info)
SET_FUNCTION_POINTER(rmw_return_loaned_message_from_subscription)
SET_FUNCTION_POINTER(rmw_create_client)
SET_FUNCTION_POINTER(rmw_destroy_client)
SET_FUNCTION_POINTER(rmw_send_request)
SET_FUNCTION_POINTER(rmw_take_response)
SET_FUNCTION_POINTER(rmw_client_request_publisher_get_actual_qos)
SET_FUNCTION_POINTER(rmw_client_response_subscription_get_actual_qos)
SET_FUNCTION_POINTER(rmw_take_request)
SET_FUNCTION_POINTER(rmw_send_response)
SET_FUNCTION_POINTER(rmw_wait)
SET_FUNCTION_POINTER(rmw_get_node_names)
SET_FUNCTION_POINTER(rmw_get_node_names_with_enclaves)
SET_FUNCTION_POINTER(rmw_count_publishers)
SET_FUNCTION_POINTER(rmw_count_subscribers)
SET_FUNCTION_POINTER(rmw_count_clients)
SET_FUNCTION_POINTER(rmw_count_services)
SET_FUNCTION_POINTER(rmw_get_gid_for_client)
SET_FUNCTION_POINTER(rmw_compare_gids_equal)
SET_FUNCTION_POINTER(rmw_service_server_is_available)
SET_FUNCTION_POINTER(rmw_set_log_severity)
SET_FUNCTION_POINTER(rmw_subscription_set_on_new_message_callback)
SET_FUNCTION_POINTER(rmw_service_set_on_new_request_callback)
SET_FUNCTION_POINTER(rmw_client_set_on_new_response_callback)
SET_FUNCTION_POINTER(rmw_event_set_callback)
SET_FUNCTION_POINTER(rmw_shutdown)
SET_FUNCTION_POINTER(rmw_context_fini)
SET_FUNCTION_POINTER(rmw_take_event)
SET_FUNCTION_POINTER(rmw_get_publisher_names_and_types_by_node)
SET_FUNCTION_POINTER(rmw_get_subscriber_names_and_types_by_node)
SET_FUNCTION_POINTER(rmw_get_service_names_and_types_by_node)
SET_FUNCTION_POINTER(rmw_get_client_names_and_types_by_node)
SET_FUNCTION_POINTER(rmw_get_topic_names_and_types)
SET_FUNCTION_POINTER(rmw_get_service_names_and_types)
SET_FUNCTION_POINTER(rmw_get_publishers_info_by_topic)
SET_FUNCTION_POINTER(rmw_get_subscriptions_info_by_topic)
SET_FUNCTION_POINTER(rmw_qos_profile_check_compatible)
SET_FUNCTION_POINTER(rmw_publisher_get_network_flow_endpoints)
SET_FUNCTION_POINTER(rmw_subscription_get_network_flow_endpoints)
