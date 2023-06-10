#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, sqrt
from numpy import matrix
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_publisher")
        self.publisher = self.create_publisher(MarkerArray, "visualization_marker_array", 10)
        timer_period = 3 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cube_position = [0.2, -0.04, -0.03]
        self.cube_orientation = [0.0, 0.0, 0.0, 1.0]
        self.sheet_position = [0.2, -0.03, -0.04]
        self.sheet_orientation = [0.0, 0.0, 0.0, 1.0]

    def set_cube_position(self, position):
        self.cube_position = position

    def set_cube_orientation(self, orientation):
        self.cube_orientation = orientation

    def set_sheet_position(self, position):
        self.sheet_position = position
        
    def set_sheet_orientation(self, orientation):
        self.sheet_orientation = orientation

    def publish_markers(self):
        cube = Marker()
        cube.header.frame_id = "base_link"
        # cube.header.stamp = self.get_clock().now().to_msg()
        cube.header.stamp.sec = self.get_clock().now().to_msg().sec
        cube.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        cube.ns = "marker_namespace"
        cube.id = 0
        cube.type = 1
        cube.action = 0
        # cube.lifetime.sec = 0
        # cube.lifetime.nanosec = 0
        cube.frame_locked = True
        cube.pose.position.x = self.cube_position[0]
        cube.pose.position.y = self.cube_position[1]
        cube.pose.position.z = self.cube_position[2]
        cube.pose.orientation.x = self.cube_orientation[0]
        cube.pose.orientation.y = self.cube_orientation[1]
        cube.pose.orientation.z = self.cube_orientation[2]
        cube.pose.orientation.w = self.cube_orientation[3]
        cube.scale.x = 0.02
        cube.scale.y = 0.02
        cube.scale.z = 0.02
        cube.color.a = 1.0
        cube.color.r = 1.0
        cube.color.g = 0.5
        cube.color.b = 0.0


        sheet = Marker()
        sheet.header.frame_id = "base_link"
        # sheet.header.stamp = self.get_clock().now().to_msg()
        sheet.header.stamp.sec = self.get_clock().now().to_msg().sec
        sheet.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        sheet.ns = "marker_namespace"
        sheet.id = 1
        sheet.type = 1
        sheet.action = 0
        # sheet.lifetime.sec = 0
        # sheet.lifetime.nanosec = 0
        sheet.frame_locked = True
        sheet.pose.position.x = self.sheet_position[0]
        sheet.pose.position.y = self.sheet_position[1]
        sheet.pose.position.z = self.sheet_position[2]
        sheet.pose.orientation.x = self.sheet_orientation[0]
        sheet.pose.orientation.y = self.sheet_orientation[1]
        sheet.pose.orientation.z = self.sheet_orientation[2]
        sheet.pose.orientation.w = self.sheet_orientation[3]
        sheet.scale.x = 0.05
        sheet.scale.y = 0.1
        sheet.scale.z = 0.001
        sheet.color.a = 1.0
        sheet.color.r = 1.0
        sheet.color.g = 1.0
        sheet.color.b = 1.0

        marker_list = MarkerArray()
        marker_list.markers.append(sheet)
        marker_list.markers.append(cube)
        self.publisher.publish(marker_list)

        self.get_logger().info("markers published")

    def timer_callback(self):
        self.publish_markers()

def main(arg=None):
    rclpy.init(args=arg)
    node = MarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
