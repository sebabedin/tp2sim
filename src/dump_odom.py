#!/usr/bin/env python3

"""
ROS node for 2D odometry dump
"""

import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class DumpOdom(Node):
    """ Node class """
    def __init__(self):
        super().__init__('odom_dump')
        # Node subscribers
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

    def odom_cb(self, msg):
        """ Odometry subscriber callback """
        quat = msg.pose.pose.orientation
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        seconds_nanoseconds = self.get_clock().now().seconds_nanoseconds()

        # print(str(self.get_clock().now().to_msg()) + '\t' + \

        out_values = []
        out_values.append(str(seconds_nanoseconds[0]))
        out_values.append(str(seconds_nanoseconds[1]))
        out_values.append(str(msg.pose.pose.position.x))
        out_values.append(str(msg.pose.pose.position.y))
        out_values.append(str(yaw))
        out_values.append(str(msg.twist.twist.linear.x))
        out_values.append(str(msg.twist.twist.angular.z))
        out_text = ','.join(out_values)
        print(out_text)
        # self.get_logger().debug(out_text)

        # print(str(seconds_nanoseconds[0]) + ',' + str(seconds_nanoseconds[1]) + ',' + \
        # str(msg.pose.pose.position.x) + ',' + \
        # str(msg.pose.pose.position.y) + ',' + str(yaw) + ',' + \
        # str(msg.twist.twist.linear.x) + ',' + \
        # str(msg.twist.twist.angular.z))

def main(args=None):
    rclpy.init(args=sys.argv)
    # node = rclpy.create_node('dump_odom')

    dump_odom_node = DumpOdom()

    try:
        rclpy.spin(dump_odom_node)
    except RuntimeError:                 # <--- process the exception
        print("Quitting")
        # rclpy.logging.get_logger("Quitting").info('Done')

    dump_odom_node.destroy_node()
    rclpy.shutdown()
    
    # dump_odom_node.get_clock().now()

if __name__ == '__main__':
    main()