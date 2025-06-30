# simulated_rover_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity

class SimulatedRover(Node):
    def __init__(self):
        super().__init__('simulated_rover_node')
        self.publisher_ = self.create_publisher(Image, 'video_topic', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        timer_period = 0.04  # 25 Hz video
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info("Simulated Rover Node started")

    def timer_callback(self):
        # Simulated video frame (a simple dynamic pattern)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        now = self.get_clock().now().seconds_nanoseconds()
        t = now[0] - self.start_time
        cv2.putText(frame, f"Mock Traffic - Time: {t}", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img_msg)

        # Circular motion parameters
        radius = 2.0  # meters
        speed = 1.0  # meters per second
        angular_speed = speed / radius  # radians per second
        total_circles = 3
        circle_duration = (2 * np.pi * radius) / speed  # time for one circle
        total_duration = total_circles * circle_duration

        stop = False
        t = t + now[1] / 1e9
        elapsed_time = t % (total_duration + 30.0) # include 30 seconds stop time

        if elapsed_time < total_duration:
            # Publish cmd_vel for circular motion
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = speed
            cmd_vel_msg.angular.z = angular_speed
            self.cmd_vel_publisher.publish(cmd_vel_msg)
        elif elapsed_time < total_duration + 1:
            # Stop for 30 seconds, publish for 1 second
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        if elapsed_time >= total_duration:
            stop = True

        # TF between map and base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'base_link'
        # Calculate x and y values for circular motion
        angle = angular_speed * elapsed_time
        tf_msg.transform.translation.z = 0.0
        # Calculate rotation for circular motion
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        if not stop:
            tf_msg.transform.translation.x = radius * np.cos(angle)
            tf_msg.transform.translation.y = radius * np.sin(angle)
            tf_msg.transform.rotation.z = np.sin(angle / 2.0)
            tf_msg.transform.rotation.w = np.cos(angle / 2.0)
        else:
            tf_msg.transform.translation.x = radius
            tf_msg.transform.translation.y = 0.0
            tf_msg.transform.rotation.z = 0.0
            tf_msg.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedRover()
    node.get_logger().set_level(LoggingSeverity.DEBUG)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
