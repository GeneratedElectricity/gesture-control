#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time


class PController(Node):
    def __init__(self):
        super().__init__('p_controller_node')
        self.angle_sub = self.create_subscription(Float64, '/hand_angle', self.angle_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.kp = 0.05  # Proportional gain
        self.last_angle_time = time.time()
        self.no_hand_timeout = 2.0  # Timeout in seconds for no hand detected

    def angle_callback(self, msg):
        angle = msg.data
        self.last_angle_time = time.time()

        # Calculate control output
        if abs(angle) < 5:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.5  # Adjust the speed as needed
            cmd_vel.angular.z = 0.0
            print("Going straight")
        elif angle < 0:
            # If angle is negative, turn left
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.4
            cmd_vel.angular.z = -self.kp * angle  # P control
            print("Turning left")
        else:
            # If angle is positive, turn right
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.4
            cmd_vel.angular.z = -self.kp * angle  # P control
            print("Turning right")

        # Publish the control command
        self.cmd_vel_pub.publish(cmd_vel)
        print(f"Published cmd_vel: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}")

    def check_no_hand_timeout(self):
        if time.time() - self.last_angle_time > self.no_hand_timeout:
            # Stop the robot if no angle received for a certain time
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            print("No hand detected. Stopping the robot.")

def main(args=None):
    rclpy.init(args=args)
    p_controller = PController()
    timer_period = 0.1  # Check every 0.1 seconds
    timer = p_controller.create_timer(timer_period, p_controller.check_no_hand_timeout)
    rclpy.spin(p_controller)
    p_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
