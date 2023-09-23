#!/usr/bin/env python3

"""
ROS node for 2D odometry dump
"""

import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from tf_transformations import euler_from_quaternion
import math

class SquarePath(Node):
    """ Node class """
    def __init__(self):
        super().__init__('square_path')
        # self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # msg = Twist(linear=Vector3(x=0., y=0., z=0.), angular=Vector3(x=0., y=0., z=0.))

        # msg = Twist()
        # msg.linear.x = 0.
        # msg.linear.y = 0.
        # msg.linear.z = 0.
        # msg.angular.x = 0.
        # msg.angular.y = 0.
        # msg.angular.z = 0.
        # self.publisher_.publish(msg)

        # self.create_rate(2).sleep()

        # Create a timer that fires every quarter second
        timer_period = 20.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.state = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.
        msg.linear.y = 0.
        msg.linear.z = 0.
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = 0.
        if 0 == self.state:
            msg.linear.x = 2. / 20.
            self.publisher_.publish(msg)
            self.state = 1
            print('x')
        elif 1 == self.state:
            msg.linear.x = 0.
            msg.angular.z = math.pi/2. / 20.
            self.publisher_.publish(msg)
            self.state = 0
            print('yaw')
        # self.get_logger().info("timer has fired")

        # self.get_logger().info('Publishing: "%s"' % msg)
        # /cmd_vel geometry_msgs/msg/Twist "{linear: {x-0.182, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.182}}"

    # def odom_cb(self, msg):
    #     """ Odometry subscriber callback """
    #     quat = msg.pose.pose.orientation
    #     orientation_list = [quat.x, quat.y, quat.z, quat.w]
    #     (_, _, yaw) = euler_from_quaternion(orientation_list)
    #     seconds_nanoseconds = self.get_clock().now().seconds_nanoseconds()

    #     # print(str(self.get_clock().now().to_msg()) + '\t' + \

    #     out_values = []
    #     out_values.append(str(seconds_nanoseconds[0]))
    #     out_values.append(str(seconds_nanoseconds[1]))
    #     out_values.append(str(msg.pose.pose.position.x))
    #     out_values.append(str(msg.pose.pose.position.y))
    #     out_values.append(str(yaw))
    #     out_values.append(str(msg.twist.twist.linear.x))
    #     out_values.append(str(msg.twist.twist.angular.z))
    #     out_text = ','.join(out_values)
    #     print(out_text)
    #     self.get_logger().debug(out_text)

def main(args=None):
    rclpy.init(args=sys.argv)
    # rclpy.logging.se()

    node = SquarePath()

    try:
        rclpy.spin(node)
    except RuntimeError:                 # <--- process the exception
        print("Quitting")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()