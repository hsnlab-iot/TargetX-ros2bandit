# video_control_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient

class VideoControlNode(Node):
    def __init__(self):
        super().__init__('video_control_node')
        self.subscription = self.create_subscription(
            Image,
            'video_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused warning

        self.control_pub = self.create_publisher(String, 'control', 10)
        self.timer = self.create_timer(0.01, self.control_callback)  # 100 Hz
        self.action_timer = self.create_timer(20.0, self.action) # 0.1 Hz
        self.get_logger().info("Video Control Node started")

    def listener_callback(self, msg):
        # Video data received, currently dropped
        self.get_logger().debug("Received video frame")

    def control_callback(self):
        # Publish mock control message
        msg = String()
        msg.data = "low_traffic_signal"
        self.control_pub.publish(msg)

    def action(self):
        self.fibonacci_client = ActionClient(self, Fibonacci, 'fibonacci')

        def send_fibonacci_goal(order):
            if not self.fibonacci_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Fibonacci action server not available!')
                return

            goal_msg = Fibonacci.Goal()
            goal_msg.order = order

            self.get_logger().info(f'Sending Fibonacci goal with order: {order}')
            self.fibonacci_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)

        def feedback_callback(feedback_msg):
            self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

        # Example usage
        send_fibonacci_goal(10)

def main(args=None):
    rclpy.init(args=args)
    node = VideoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
