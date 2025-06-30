import os
import zmq
import zmq.utils.monitor as zmq_monitor
import threading
import json

class Connection:

    def __init__(self, node):
        broker_address = os.getenv("ROS2BANDIT_BROKER", "127.0.0.1")
        broker_port = os.getenv("ROS2BANDIT_BROKER_PORT", "1884")
        self.broker_address = f"tcp://{broker_address}:{broker_port}"
        self.token = os.getenv("ROS2BANDIT_TOKEN", "")
        self.domain = os.getenv("ROS2BANDIT_DOMAIN", "")

        self._node = node

        context = zmq.Context()
        self.dealer_socket = context.socket(zmq.DEALER)

        self.cb_on_message = []
        self.cb_on_connect = []

        self.sendlock = threading.Lock()

        def monitor_connection(monitor_socket):
            while True:
                event = zmq_monitor.recv_monitor_message(monitor_socket)
                evt = event['event']
                if evt == zmq.EVENT_CONNECTED:
                    # Send id
                    message = {"controller_id": self._node.get_name(), "type": "controller", "domain": self.domain}
                    self.send(message)
                    self._node.get_logger().info(f"Sent controller message with controller_id {self._node.get_name()}")

                    for oc in self.cb_on_connect:
                        oc()

        def listener(socket):
            poller = zmq.Poller()
            poller.register(socket, zmq.POLLIN)

            while True:
                events = dict(poller.poll())
                if socket in events:
                    msg = socket.recv_multipart()
                    message = None
                    try:
                        message = json.loads(msg[0].decode())
                    except json.JSONDecodeError as e:
                        self._node.get_logger().warning(f"Failed to decode message: {e}")

                    self._node.get_logger().info(f"Receiving message {message}")
                    for om in self.cb_on_message:
                        om(message)

        # Monitor socket events
        dealer_monitor = self.dealer_socket.get_monitor_socket()
        threading.Thread(target=monitor_connection, args=(dealer_monitor,), daemon=True).start()

        # Start the dealer listener in a separate thread
        threading.Thread(target=listener, args=(self.dealer_socket,), daemon=True).start()

        self.dealer_socket.connect(self.broker_address)
        self._node.get_logger().info(f"Connected to broker at {self.broker_address}")


    def register_onmessage(self, callback):
        if callback not in self.cb_on_message:
            self.cb_on_message.append(callback)

    def register_onconnect(self, callback):
        if callback not in self.cb_on_connect:
            self.cb_on_connect.append(callback)

    def send(self, message):
        # Use a lock to ensure thread-safe access to the socket
        with self.sendlock:
            message["token"] = self.token
            if self.domain:
                message["domain"] = self.domain
            self._node.get_logger().info(f"Sending message {message}")
            message = json.dumps(message).encode('utf-8')
            self.dealer_socket.send(message)

# --- Singleton instance ---
_instance = None

def get_connection_handler(node):
    global _instance
    if _instance is None:
        _instance = Connection(node)
    return _instance