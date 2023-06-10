#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from math import cos, sin, sqrt
from numpy import matrix
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
from ros2_aruco_interfaces.msg import ArucoMarkers
import numpy as np

class MarkerBroker(Node):
    def __init__(self):
        super().__init__("marker_broker")
        self.publisher = self.create_publisher(MarkerArray, "visualization_marker_array", 10)
        self.pos_gripper = self.create_subscription(PoseStamped, 'gripper_pose', self.gripper_callback, 10)
        self.aruco_sub = self.create_subscription(ArucoMarkers, "aruco_markers", self.aruco_markers_callback, 10)
        timer_period = 3 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cube_position = [0.2, -0.04, -0.03]
        self.cube_orientation = [0.0, 0.0, 0.0, 1.0]
        self.sheet_position = [0.2, -0.03, -0.04]
        self.sheet_orientation = [0.0, 0.0, 0.0, 1.0]

        self.cube_pose = None
        self.sheet_pose = None
        self.gripper_pos = None
        self.T_inv = None
    
    def gripper_callback(self, pos):
        self.gripper_pos = pos
        x = pos.pose.orientation.x
        y = pos.pose.orientation.y
        z = pos.pose.orientation.z
        w = pos.pose.orientation.w

        xp = pos.pose.position.x
        yp = pos.pose.position.y
        zp = pos.pose.position.z
        rot = matrix([[2*(w**2+x**2)-1, 2*(x*y-w*z), 2*(x*z+w*y)],
                [2*(x*y+w*z), 2*(w**2+y**2)-1, 2*(y*z-w*x)],
                [2*(x*z-w*y), 2*(y*z+w*x), 2*(w**2+z**2)-1]])

        self.T_inv = np.linalg.inv(rot)



    def aruco_markers_callback(self, marker):
        i = 0
        if self.T_inv is not None:
            for pose in marker.poses:
                if marker.marker_ids[i] == 14:
                    self.cube_pose = self.transpose_point(pose)
                if marker.marker_ids[i] == 7:
                    self.sheet_pose = self.transpose_point(pose)
                i += 1

            self.publish_markers()

    def transpose_point(self, pos):
        # vect = np.matrix([pos.position.x, pos.position.y,pos.position.z,1])
        tr = self.T_inv[0, 0] + self.T_inv[1, 1] + self.T_inv[2, 2]
        # position = (vect * self.T_inv).flatten()
        if tr > 0:
            S = sqrt(tr + 1.0) * 2
            qx = (self.T_inv[2, 1] - self.T_inv[1, 2])/S
            qy = (self.T_inv[0, 2] - self.T_inv[2, 0])/S
            qz = (self.T_inv[1, 0] - self.T_inv[0, 1])/S
            qw = S/4
             
        elif (self.T_inv[0, 0] > self.T_inv[1, 1]) and (self.T_inv[0, 0] > self.T_inv[2, 2]):
            S = sqrt(1.0 + self.T_inv[0, 0] - self.T_inv[1, 1] - self.T_inv[2, 2]) * 2
            qx = S/4
            qy = (self.T_inv[0, 1] + self.T_inv[1, 0])/S
            qz = (self.T_inv[0, 2] + self.T_inv[2, 0])/S
            qw = (self.T_inv[2, 1] - self.T_inv[1, 2])/S
            
        elif (self.T_inv[1, 1] > self.T_inv[2, 2]):
            S = sqrt(1.0 + self.T_inv[1, 1] - self.T_inv[0, 0] - self.T_inv[2, 2]) * 2
            qx = (self.T_inv[0, 1] + self.T_inv[1, 0])/S
            qy = S/4
            qz = (self.T_inv[1, 2] + self.T_inv[2, 1])/S
            qw = (self.T_inv[0, 2] - self.T_inv[2, 0])/S

        else:
            S = sqrt(1.0 + self.T_inv[2, 2] - self.T_inv[0, 0] - self.T_inv[1, 1]) * 2
            qx = (self.T_inv[0, 2] + self.T_inv[2, 0])/S
            qy = (self.T_inv[1, 2] + self.T_inv[2, 1])/S
            qz = S/4
            qw = (self.T_inv[1, 0] - self.T_inv[0, 1])/S

        # new_pos = Pose()
        new_pos = Pose()

        # print(position[0])
        # new_pos.position.x = float(position[0])
        # new_pos.position.y = float(position[1])
        # new_pos.position.z = float(position[2])
        # new_pos.orientation.x = float(qx)
        # new_pos.orientation.y = float(qy)
        # new_pos.orientation.z = float(qz)
        # new_pos.orientation.w  =float(qw)
        new_pos.orientation.x = pos.orientation.z
        new_pos.orientation.y = -pos.orientation.x
        new_pos.orientation.z = -pos.orientation.y
        new_pos.orientation.w  = pos.orientation.w
        new_pos.position.x = pos.position.z
        new_pos.position.y = -pos.position.x
        new_pos.position.z = -pos.position.y
        return new_pos

    def set_cube_position(self, position):
        self.cube_position = position

    def set_cube_orientation(self, orientation):
        self.cube_orientation = orientation

    def set_sheet_position(self, position):
        self.sheet_position = position
        
    def set_sheet_orientation(self, orientation):
        self.sheet_orientation = orientation

    def publish_markers(self):

        marker_list = MarkerArray()
        if self.cube_pose is not None:
            cube = Marker()
            cube.header.frame_id = "camera_link"
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
            # cube.pose.position.x = self.cube_position[0]
            # cube.pose.position.y = self.cube_position[1]
            # cube.pose.position.z = self.cube_position[2]
            # cube.pose.orientation.x = self.cube_orientation[0]
            # cube.pose.orientation.y = self.cube_orientation[1]
            # cube.pose.orientation.z = self.cube_orientation[2]
            # cube.pose.orientation.w = self.cube_orientation[3]
            cube.pose = self.cube_pose
            cube.scale.x = 0.02
            cube.scale.y = 0.02
            cube.scale.z = 0.02
            cube.color.a = 1.0
            cube.color.r = 1.0
            cube.color.g = 0.5
            cube.color.b = 0.0
            marker_list.markers.append(cube)


        if self.sheet_pose is not None:
            sheet = Marker()
            sheet.header.frame_id = "camera_link"
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
            # sheet.pose.position.x = self.sheet_position[0]
            # sheet.pose.position.y = self.sheet_position[1]
            # sheet.pose.position.z = self.sheet_position[2]
            # sheet.pose.orientation.x = self.sheet_orientation[0]
            # sheet.pose.orientation.y = self.sheet_orientation[1]
            # sheet.pose.orientation.z = self.sheet_orientation[2]
            # sheet.pose.orientation.w = self.sheet_orientation[3]
            sheet.pose = self.sheet_pose
            sheet.scale.x = 0.001
            sheet.scale.y = 0.1
            sheet.scale.z = 0.05
            sheet.color.a = 1.0
            sheet.color.r = 1.0
            sheet.color.g = 1.0
            sheet.color.b = 1.0

            marker_list.markers.append(sheet)
        self.publisher.publish(marker_list)

        self.get_logger().info("markers published")

    def timer_callback(self):
        self.publish_markers()

def main(arg=None):
    rclpy.init(args=arg)
    node = MarkerBroker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
