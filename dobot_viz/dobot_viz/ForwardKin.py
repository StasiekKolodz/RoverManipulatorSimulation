#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, sqrt
from numpy import matrix


class ForwardKinNode(Node):
    def __init__(self):
        super().__init__("ForwardKin")
        self.publisher = self.create_publisher(PoseStamped, 'gripper_pose', 10)
        self.subscriber = self.create_subscription(
            JointState, "joint_states", self.position_callback, 10)

        self.dh = 0.1
        self.uh = 0.125
        self.mhh = 0.06
        self.position = [0, 0, 0, 0]
        self.rotation = [0, 0, 0, 0]
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def position_callback(self, pos):
        base = pos.position[0]
        down_manip = pos.position[1]
        up_manip = pos.position[2]
        manip_hand = pos.position[3]
        tool = pos.position[4]

        base_point = matrix([[0], [0], [0], [1]])

        H1 = matrix([[cos(base), -sin(base), 0, 0],
                     [sin(base), cos(base), 0, 0],
                     [0, 0, 1, 0.043],
                     [0, 0, 0, 1]])

        H2 = matrix([[cos(down_manip), 0, sin(down_manip), 0],
                     [0, 1, 0, 0],
                     [-sin(down_manip), 0, cos(down_manip), 0.058],
                     [0, 0, 0, 1]])

        H3 = matrix([[cos(up_manip+1.57), 0, sin(up_manip+1.57), 0],
                     [0, 1, 0, 0],
                     [-sin(up_manip+1.57), 0, cos(up_manip+1.57), self.dh],
                     [0, 0, 0, 1]])

        H4 = matrix([[cos(manip_hand), 0, sin(manip_hand), 0],
                     [0, 1, 0, 0],
                     [-sin(manip_hand), 0, cos(manip_hand), self.uh],
                     [0, 0, 0, 1]])

        H5 = matrix([[1, 0, 0, 0.0175],
                     [0, cos(tool), -sin(tool), 0],
                     [0, sin(tool), cos(tool), 0.03],
                     [0, 0, 0, 1]])

        H6 = matrix([[1, 0, 0, 0.03],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

        H = H1*H2*H3*H4*H5*H6
        self.position = H*base_point

        tr = H[0, 0] + H[1, 1] + H[2, 2]
        if tr > 0:
            S = sqrt(tr + 1.0) * 2
            qx = (H[2, 1] - H[1, 2])/S
            qy = (H[0, 2] - H[2, 0])/S
            qz = (H[1, 0] - H[0, 1])/S
            qw = S/4
             
        elif (H[0, 0] > H[1, 1]) and (H[0, 0] > H[2, 2]):
            S = sqrt(1.0 + H[0, 0] - H[1, 1] - H[2, 2]) * 2
            qx = S/4
            qy = (H[0, 1] + H[1, 0])/S
            qz = (H[0, 2] + H[2, 0])/S
            qw = (H[2, 1] - H[1, 2])/S
            
        elif (H[1, 1] > H[2, 2]):
            S = sqrt(1.0 + H[1, 1] - H[0, 0] - H[2, 2]) * 2
            qx = (H[0, 1] + H[1, 0])/S
            qy = S/4
            qz = (H[1, 2] + H[2, 1])/S
            qw = (H[0, 2] - H[2, 0])/S

        else:
            S = sqrt(1.0 + H[2, 2] - H[0, 0] - H[1, 1]) * 2
            qx = (H[0, 2] + H[2, 0])/S
            qy = (H[1, 2] + H[2, 1])/S
            qz = S/4
            qw = (H[1, 0] - H[0, 1])/S

        self.rotation[0] = qx
        self.rotation[1] = qy
        self.rotation[2] = qz
        self.rotation[3] = qw

    def timer_callback(self):
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.018
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.header.stamp.sec = self.get_clock().now().to_msg().sec
        msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg.header.frame_id = ('camera_link')
        self.publisher.publish(msg)


def main(arg=None):
    rclpy.init(args=arg)
    node = ForwardKinNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
