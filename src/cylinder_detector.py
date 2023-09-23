#!/usr/bin/env python3

"""
ROS node for 2D odometry dump
"""

import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
# from std_msgs.msg import 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf_transformations import euler_from_quaternion
import math

# Type: sensor_msgs/msg/LaserScan
# Publisher count: 1
# Subscription count: 1

import csv

def write_csv_(file_path, data_list, header=None):
    try:
        with open(file_path, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)

            # Write the header if provided
            if header:
                csv_writer.writerow(header)

            # Write the data from the list of tuples
            for row in data_list:
                csv_writer.writerow(row)
        
        # print(f"CSV file '{file_path}' has been successfully written.")
    except Exception as e:
        print(f"An error occurred: {e}")

def distance_between_points(a, b):
    xa = a[0]
    ya = a[1]
    xb = b[0]
    yb = b[1]
    dx = xa - xb
    dy = ya - yb
    return math.sqrt(dx*dx + dy*dy)

def average_coordinate(points, index):
    ave = 0
    for point in points:
        ave += point[index]
    ave /= len(points)
    return ave

def average_min_max_coordinate(points, index):
    min_value = min([point[index] for point in points])
    max_value = max([point[index] for point in points])
    return (min_value + max_value) / 2

def center_point(points):
    # x = average_coordinate(points, 0)
    # y = average_coordinate(points, 1)

    x = average_min_max_coordinate(points, 0)
    y = average_min_max_coordinate(points, 1)
    return x, y

def cylinder_calculate(points):
    p1 = points[0]
    p2 = points[int(len(points) / 2)]
    p3 = points[-1]

    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x3 = p3[0]
    y3 = p3[1]

    # ad12 = -2 * (y2 - y1)
    # a1 = (x1*x1 - x2*x2) / ad12
    # a2 = (y1*y1 - y2*y2) / ad12
    # a3 = (2 * (x2 - x1)) / ad12

    # ad13 = -2 * (y3 - y1)
    # a4 = (x1*x1 - x3*x3) / ad13
    # a5 = (y1*y1 - y3*y3) / ad13
    # a6 = (2 * (x3 - x1)) / ad13
    # h = (a1 - a4 + a2 - a5) / (a6 - a3)

    # b1 = a1
    # b2 = a3
    # b3 = a6
    # k = b1 + b2 + b3 * h

    ad12 = -2 * (y2 - y1)
    a1 = (x1*x1 - x2*x2) / ad12
    a2 = (y1*y1 - y2*y2) / ad12
    a3 = (2 * (x2 - x1)) / ad12

    ad13 = -2 * (y3 - y1)
    a4 = (x1*x1 - x3*x3) / ad13
    a5 = (y1*y1 - y3*y3) / ad13
    a6 = (2 * (x3 - x1)) / ad13
    h = (a1 - a4 + a2 - a5) / (a6 - a3)
    # print(h)

    b1 = a1
    b2 = a2
    b3 = a3
    k = b1 + b2 + b3 * h

    c1 = pow(x1-h, 2)
    c2 = pow(y1-k, 2)
    r = math.sqrt(c1 + c2)

    # x = average_min_max_coordinate(points, 0)
    # y = average_min_max_coordinate(points, 1)
    return h, k, r

class CylinderDetector(Node):
    """ Node class """
    def __init__(self):
        super().__init__('cylinder_detector')
        self.create_subscription(LaserScan, 'scan', self.on_scan_cb_, 0)

#  1 ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


        self.publisher_ = self.create_publisher(MarkerArray, 'cylinders_marker', 0)
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
        # timer_period = 20.0
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.state = 0

    # def timer_callback(self):
    #     msg = Twist()
    #     msg.linear.x = 0.
    #     msg.linear.y = 0.
    #     msg.linear.z = 0.
    #     msg.angular.x = 0.
    #     msg.angular.y = 0.
    #     msg.angular.z = 0.
    #     if 0 == self.state:
    #         msg.linear.x = 2. / 20.
    #         self.publisher_.publish(msg)
    #         self.state = 1
    #         print('x')
    #     elif 1 == self.state:
    #         msg.linear.x = 0.
    #         msg.angular.z = math.pi/2. / 20.
    #         self.publisher_.publish(msg)
    #         self.state = 0
    #         print('yaw')
        # self.get_logger().info("timer has fired")

        # self.get_logger().info('Publishing: "%s"' % msg)
        # /cmd_vel geometry_msgs/msg/Twist "{linear: {x-0.182, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.182}}"

# header:
#   stamp:
#     sec: 107
#     nanosec: 740000000
#   frame_id: base_scan
# angle_min: 0.0
# angle_max: 6.28000020980835
# angle_increment: 0.01749303564429283
# time_increment: 0.0
# scan_time: 0.0
# range_min: 0.11999999731779099
# range_max: 20.0
# ranges:


    def on_scan_cb_(self, msg):
        # msg = LaserScan()
        # stamp = msg.header.stamp
        # angle_min = msg.angle_min
        # angle_max = msg.angle_max
        # angle_increment = msg.angle_increment
        # ranges = msg.ranges
        # nranges = len(ranges)

        points = []

        angle = msg.angle_min
        for range in msg.ranges:
            if msg.range_min <= range and range <= msg.range_max:
                # print(range)
                x = range * math.cos(angle)
                y = range * math.sin(angle)
                point = (x, y, angle, range)
                points.append(point)
                print(point)

            angle += msg.angle_increment
        
        d_max = msg.range_max * math.sin(msg.angle_increment)
        d_max *= 1.1
        cylinder_points = []
        cylinders = []
        for point in points:
            if 0 == len(cylinder_points):
                print('ref')
                cylinder_points.append(point)
                ref_point = point
            else:
                d = distance_between_points(point, ref_point)
                if d < d_max:
                    cylinder_points.append(point)
                    ref_point = point
                else:
                    # x, y = center_point(cylinder_points)
                    x, y, r = cylinder_calculate(cylinder_points)
                    cylinder = (x, y, r)
                    # print('center')
                    # print(cylinder)
                    cylinders.append(cylinder)
                    cylinder_points = []

        if 0 < len(cylinder_points):
            # x, y = center_point(cylinder_points)
            x, y, r = cylinder_calculate(cylinder_points)
            cylinder = (x, y, r)
            print('center')
            print(cylinder)
            cylinders.append(cylinder)
            cylinder_points = []

        root_path = '/home/seb/ros2ws/rbtp2_ws/src/tp2sim'
        csv_file = root_path + '/resource/cylinders_points.csv'
        write_csv_(csv_file, points)

        root_path = '/home/seb/ros2ws/rbtp2_ws/src/tp2sim'
        csv_file = root_path + '/resource/cylinders.csv'
        write_csv_(csv_file, cylinders)

        markerArray_msg = MarkerArray()
        # marker_msg = Marker()
        # marker_msg.action = marker_msg.DELETEALL
        # markerArray_msg.markers.append(marker_msg)

        id = 0
        for cylinder in cylinders:
            marker_msg = Marker()
            # msg.CYLINDER

            marker_msg.header = msg.header
            # marker_msg.header.stamp = "base_san";
            # marker_msg.header.stamp = rclpy.;
            marker_msg.id = id
            marker_msg.ns = str(id)
            marker_msg.type = marker_msg.CYLINDER
            marker_msg.action = marker_msg.ADD
            # marker_msg.pose = Pose()
            marker_msg.pose.position.x = cylinder[0]
            marker_msg.pose.position.y = cylinder[1]
            marker_msg.pose.position.z = 0.
            marker_msg.pose.orientation.x = 0.0
            marker_msg.pose.orientation.y = 0.0
            marker_msg.pose.orientation.z = 0.0
            marker_msg.pose.orientation.w = 1.0
            marker_msg.scale.x = 2 * cylinder[2]
            marker_msg.scale.y = 2 * cylinder[2]
            marker_msg.scale.z = 0.1
            marker_msg.color.a = 0.25
            marker_msg.color.r = 0.0
            marker_msg.color.g = 1.0
            marker_msg.color.b = 0.0
            # luos_color = ColorRGBA()
            # marker_msg.color = luos_color

            markerArray_msg.markers.append(marker_msg)
            id += 1

        self.publisher_.publish(markerArray_msg)

        # self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 0)

        # self.destroy_node()


        # quat = msg.pose.pose.orientation
        # orientation_list = [quat.x, quat.y, quat.z, quat.w]
        # (_, _, yaw) = euler_from_quaternion(orientation_list)
        # seconds_nanoseconds = self.get_clock().now().seconds_nanoseconds()

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

    node = CylinderDetector()

    try:
        rclpy.spin(node)
    except RuntimeError:                 # <--- process the exception
        print("Quitting")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()