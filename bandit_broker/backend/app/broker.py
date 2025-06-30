import zmq
import json
from shared_memory import get_shared_resources
import os
import time
from multiprocessing import Manager
from shared_memory_dict import SharedMemoryDict
import pickle
from threading import Lock
import threading

guard = Lock()

publishers = {} # Dictionary to store host_id lists for domain/topics. 
nodes = {}  # Dictionary to store host_id-s for domain/nodes
controllers = {} # Dictionary to store host_id-s for domain/controllers
controllers_wc = {} # Dictionary to store host_id-s for wildcard  controllers (all domains)
domains = {} # Dictionary to store domains for host_id-s
topics_watched = [] # Watched domain/topics
actions_watched = [] # Watched domain/actions
blocked = [] # Blocked domain/topics
action_servers = {} # Dictionary to store host_id-s for domain/action servers
action_clients = {} # Dictionary to store host_id-s for domain/action clients

trigger_types = ["trigger"]
ctrl_types = ["block", "unblock", "watch", "unwatch", "watch_action", "unwatch_action"]
state_types = ["traffic", "statemachines_description", "state_change", "trigger_change"]
action_types = ["action_server", "action_client"]

shared_topic_traffic = SharedMemoryDict(name="topic_traffic", size=10240)
shared_states_desc = SharedMemoryDict(name="statemachines_description", size=10240)
shared_states = SharedMemoryDict(name="states", size=10240)
shared_triggers = SharedMemoryDict(name="triggers", size=10240)
shared_actions = SharedMemoryDict(name="actions", size=10240)

cb_incoming = 0 # Byte counter for incoming messages
cb_outgoing = 0 # Byte counter for outgoing messages


def broker_main(shared = None, lock = None):
    global cb_incoming, cb_outgoing

    if shared is None or lock is None:
        # If run standalone, set up our own
        shared, lock = get_shared_resources()
        print("[Broker] Running standalone with internal shared memory")

    # Create a ZeroMQ context
    context = zmq.Context()

    # Create a ROUTER socket
    socket = context.socket(zmq.ROUTER)

    # Bind the socket to the specified address and port
    port = os.getenv("BROKER_PORT", "1884")
    socket.bind("tcp://0.0.0.0:" + port)
    print(f"ZMQ Router listening on tcp://0.0.0.0:{port}")

    # Create a log file in the current folder using the current timestamp
    log_filename = time.strftime("broker_%y%m%d-%H%M%S.log", time.localtime())
    log_path = os.path.join(os.path.dirname(__file__), 'log', log_filename)
    log_dir = os.path.dirname(log_path)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_file = open(log_path, "a")

    def log(msg):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log_file.write(f"[{timestamp}] {msg}\n")
        log_file.flush()
        print(f"[{timestamp}] {msg}")

    token = os.getenv("ROS2BANDIT_TOKEN", "")

    def process_traffic(parsed_data, domain):
        topic = parsed_data.get("topic", None)
        if topic is None:
            return
        d_topic = f"{domain}{topic}" if domain else topic
        data = {}
        data["bw"] = parsed_data.get("bandwidth", 0)
        data["hz"] = parsed_data.get("frequency", 0)
        data["blocked"] = parsed_data.get("blocked", False)
        data["update"] = time.time()
        with guard:
            shared_topic_traffic[topic] = pickle.dumps(data)
        log(f"Updated traffic for {d_topic}: {data}")

    def process_statemachines_description(parsed_data, domain):
        state_machine = parsed_data.get("state_machine", None)
        states = parsed_data.get("states", None)
        if state_machine is None or states is None:
            log(f"Invalid state machine or states: {parsed_data}")
            return
        state_machine = f"{domain}/{state_machine}" if domain else state_machine
        with guard:
            shared_states_desc[state_machine] = pickle.dumps(states)
        log(f"Updated states for {state_machine}: {states}")

    def get_domain(id):
            if id in domains:
                return domains[id]
            else:
                return ""

    def message_add_domain(message, domain):
        message = message.copy()
        # Add the domain to the message
        if not domain:
            return message
        
        if message.get("topic"):
            message["topic"] = f"{domain}{message['topic']}"
        if message.get("action"):
            message["action"] = f"{domain}{message['action']}"
        return message
    
    def message_remove_domain(message):
        message = message.copy()
        domain = ""
        # Remove the domain from the message
        if message.get("topic"):
            if not message["topic"].startswith("/"):
                domain, _, topic = message["topic"].partition("/")
                domain = f"{domain}" if domain else ""
                message["topic"] = f"/{topic}" if topic else ""
        if message.get("action"):
            if not message["action"].startswith("/"):
                domain, _, action = message["action"].partition("/")
                domain = f"{domain}" if domain else ""
                message["action"] = f"/{action}" if action else ""
        return message, domain

    def send_message(socket, host_id, message):
        global cb_outgoing
        # Send a message to the host_id
        message["token"] = token
        log(f"Sending message to {host_id.hex()}: {message}")
        msg = json.dumps(message).encode('utf-8')
        socket.send_multipart([host_id, msg])

        cb_outgoing += len(msg) + len(host_id)

    def print_counters():
        global cb_incoming, cb_outgoing
        prev_in = 0
        prev_out = 0
        while True:
            time.sleep(1)
            delta_in = cb_incoming - prev_in
            delta_out = cb_outgoing - prev_out
            log(f"[Broker] Incoming bytes delta: {delta_in}, Outgoing bytes delta: {delta_out}")
            prev_in = cb_incoming
            prev_out = cb_outgoing

    counter_thread = threading.Thread(target=print_counters, daemon=True)
    counter_thread.start()

    try:
        while True:
            # Receive a message
            message = socket.recv_multipart()
            # Send back an ACK
            #socket.send_multipart([message[0], b'ACK'])
            host_id = message[0]
            parsed_data = None

            log(f"Received message from {host_id.hex()}: {message[1]}")

            cb_incoming += len(message[1]) + len(message[0])

            try:
                parsed_data = json.loads(message[1].decode('utf-8'))
                #print(f"Parsed data: {parsed_data}")
            except json.JSONDecodeError as e:
                log(f"Failed to parse JSON: {e} for message: {message[1]}")
                continue                

            # Check token
            if token and parsed_data.get("token", "") != token:
                log(f"Invalid token: {parsed_data}")
                continue

            if parsed_data.get("type") == "publisher":
                domain = parsed_data.get("domain", "")
                domains[host_id] = domain
                topic = parsed_data.get("topic")
                d_topic = f"{domain}{topic}" if domain else topic   # topic name starts with /
                if not d_topic in publishers:
                    publishers[d_topic] = [host_id]
                else:
                    if host_id not in publishers[d_topic]:
                        publishers[d_topic].append(host_id)
                log(f"Registered publisher {host_id.hex()} for topic: {d_topic}")

                if d_topic in blocked:
                    # Send a block message to the publisher
                    log(f"Blocking publisher {host_id.hex()} for topic {d_topic}")
                    message = {"topic": topic, "type": "block"}
                    send_message(socket, host_id, message)
                if d_topic in topics_watched:
                    # Send a watch message to the publisher
                    log(f"Watching publisher {host_id.hex()} for topic {d_topic}")
                    message = {"topic": topic, "type": "watch"}
                    send_message(socket, host_id, message)

            elif parsed_data.get("type") == "nodename":
                domain = parsed_data.get("domain", "")
                domains[host_id] = domain
                nodename = parsed_data.get("nodename")
                d_nodename = f"{domain}/{nodename}" if domain else nodename
                # Check if the nodename is already registered
                if d_nodename in nodes:
                    log(f"Node {d_nodename} already registered with host_id: {nodes[d_nodename].hex()} Clearing old entry.")
                    old_host_id = nodes[d_nodename]
                    # Find all publishers with the old host_id and remove them
                    for d_topic, pub_host_ids in list(publishers.items()):
                        if old_host_id in pub_host_ids:
                            log(f"Removing old publisher {old_host_id.hex()} for topic {d_topic}")
                            pub_host_ids.remove(old_host_id)
                nodes[d_nodename] = host_id
                log(f"Registered node {host_id.hex()} with nodename: {d_nodename}")

            elif parsed_data.get("type") == "controller":
                domain = parsed_data.get("domain", "")
                controller_id = parsed_data.get("controller_id")
                controller_id = f"{domain}/{controller_id}" if domain else controller_id
                if domain == "*":
                    # Wildcard controller, register it for all domains
                    controllers_wc[controller_id] = host_id
                else:
                    domains[host_id] = domain
                    controllers[controller_id] = host_id
                log(f"Registered controller {host_id.hex()} with controller_id: {controller_id}, domain: {domain}")

            elif parsed_data.get("type") == "action_server":
                domain = parsed_data.get("domain", "")
                domains[host_id] = domain
                action = parsed_data.get("action")
                d_action = f"{domain}{action}" if domain else action
                action_servers[d_action] = host_id
                log(f"Registered action server {host_id.hex()} with action: {d_action}, domain: {domain}")
                node = next((key for key, value in nodes.items() if value == host_id), None)
                with guard:
                    shared_actions[d_action] = node

                if d_action in actions_watched:
                    # Send a watch message to the action server
                    log(f"Watching action {d_action} on host {host_id.hex()}")
                    message = {"action": action, "type": "watch"}
                    send_message(socket, host_id, message)

            elif parsed_data.get("type") == "action_client":
                domain = parsed_data.get("domain", "")
                domains[host_id] = domain
                action = parsed_data.get("action")
                d_action = f"{domain}{action}" if domain else action
                if d_action not in action_clients:
                    action_clients[d_action] = []
                if host_id not in action_clients[d_action]:
                    action_clients[d_action].append(host_id)
                log(f"Registered action client {host_id.hex()} with action: {d_action}, domain: {domain}")

            elif parsed_data.get("type") == "traffic":
                log(f"Received traffic info: {parsed_data}")
                domain = get_domain(host_id)
                d_parsed_data = message_add_domain(parsed_data, domain)
                process_traffic(d_parsed_data, domain)

            elif parsed_data.get("type") == "statemachines_description":
                log(f"Received statemachines description: {parsed_data}")
                domain = get_domain(host_id)
                process_statemachines_description(parsed_data, domain)

            elif parsed_data.get("type") == "state_change":
                log(f"Received state change: {parsed_data}")
                domain = get_domain(host_id)
                with guard:
                    state_machine = parsed_data.get("state_machine")
                    old_state = parsed_data.get("old_state", "unknown")
                    new_state = parsed_data.get("new_state")
                    if state_machine is None or new_state is None:
                        log(f"Invalid state change data: {parsed_data}")
                        continue
                    old_state_full = f"{domain}/{state_machine}/{old_state}" if domain else f"{state_machine}/{old_state}"
                    new_state_full = f"{domain}/{state_machine}/{new_state}" if domain else f"{state_machine}/{new_state}"
                    # Update the state in shared memory
                    shared_states[old_state_full] = False
                    shared_states[new_state_full] = True

            elif parsed_data.get("type") == "trigger_change":
                log(f"Received trigger change: {parsed_data}")
                domain = get_domain(host_id)
                with guard:
                    trigger = parsed_data.get("trigger")
                    state = parsed_data.get("state")
                    state_machine = parsed_data.get("state_machine")
                    active = parsed_data.get("active")
                    if trigger is None or state is None or state_machine is None:
                        log(f"Invalid trigger change data: {parsed_data}")
                        continue
                    # Update the trigger in shared memory
                    trigger_full = f"{domain}/{state_machine}/{state}/{trigger}" if domain else f"{state_machine}/{state}/{trigger}"
                    shared_triggers[trigger_full] = active

            elif parsed_data.get("type") in trigger_types:
                log(f"Received trigger: {parsed_data}")
                c = 0
                domain = get_domain(host_id)
                d_parsed_data = message_add_domain(parsed_data, domain)
                for ctrl, controller_id in controllers_wc.items():
                    # Send to all wildcard controllers
                    send_message(socket, controller_id, d_parsed_data)
                    log(f"Informing wildcard controller {controller_id.hex()} with type {parsed_data.get('type')}")                    
                    c += 1
                for ctrl, controller_id in controllers.items():
                    # Send to all controllers in the same domain
                    if not domain == get_domain(controller_id):
                        continue
                    # Send message to the controller
                    send_message(socket, controller_id, d_parsed_data)
                    log(f"Informing controller {controller_id.hex()} with type {parsed_data.get('type')}")                    
                    c += 1
                if c == 0:
                    log(f"No controller found for trigger: {parsed_data}, domain: {domain}")

            elif parsed_data.get("type") in ctrl_types:
                log(f"Received control: {parsed_data}")
                dd_parsed_data, dd_domain = message_remove_domain(parsed_data)
                domain = get_domain(host_id)
                topic = dd_parsed_data.get("topic")
                d_topic = parsed_data.get("topic")
                action = dd_parsed_data.get("action")
                d_action = parsed_data.get("action")
                if topic is None and action is None:
                    log(f"Invalid control message: {parsed_data}")
                    continue

                if d_topic is not None:
                    if d_topic in publishers:
                        # Send a controll message to the publishers
                        for publisher_id in publishers[d_topic]:
                            log(f"Controling publisher {publisher_id.hex()} for topic {topic} with type {parsed_data.get('type')}")
                            message = {"topic": topic, "type": parsed_data.get("type")}
                            send_message(socket, publisher_id, message)
                    else:
                        log(f"No publisher found for control: {parsed_data}, topic: {topic}, domain: {domain}")

                    if parsed_data.get("type") == "block":
                        log(f"Blocking topic {d_topic} for controller {host_id.hex()}")
                        # Add the host_id to the blockers list
                        if d_topic not in blocked:
                            blocked.append(d_topic)
                    elif parsed_data.get("type") == "unblock":
                        log(f"Unblocking topic {d_topic} for controller {host_id.hex()}")
                        # Remove the host_id from the blockers list
                        if d_topic in blocked:
                            blocked.remove(d_topic)
                    elif parsed_data.get("type") == "watch":
                        log(f"Watching topic {d_topic} for controller {host_id.hex()}")
                        # Add the host_id to the topic watchers list
                        if d_topic not in topics_watched:
                            topics_watched.append(d_topic)
                    elif parsed_data.get("type") == "unwatch":
                        log(f"Unwatching topic {d_topic} for controller {host_id.hex()}")
                        # Remove the host_id from the watchers list
                        if d_topic in topics_watched:
                            topics_watched.remove(d_topic)

                elif d_action is not None:
                    if d_action in action_servers:
                        # Send a controll message to the action server
                        action_server = action_servers[d_action]
                        log(f"Controling action server {action_server.hex()} for action {action} with type {parsed_data.get('type')}")
                        message = {"action": action, "type": parsed_data.get("type")}
                        send_message(socket, action_server, message)
                    else:
                        log(f"No action server found for control: {parsed_data}, action: {action}, domain: {domain}")

                    if parsed_data.get("type") == "watch":
                        log(f"Watching action {d_action} for controller {host_id.hex()}")
                        # Add the host_id to the action watchers list
                        if d_action not in actions_watched:
                            actions_watched.append(d_action)
                    elif parsed_data.get("type") == "unwatch":
                        log(f"Unwatching action {d_action} for controller {host_id.hex()}")
                        # Remove the host_id from the watchers list
                        if d_action in actions_watched:
                            actions_watched.remove(d_action)

    except KeyboardInterrupt:
        log("Shutting down...")
    finally:
        # Clean up
        socket.close()
        context.term()
        counter_thread.join()
        log_file.close()

if __name__ == "__main__":
    broker_main()
