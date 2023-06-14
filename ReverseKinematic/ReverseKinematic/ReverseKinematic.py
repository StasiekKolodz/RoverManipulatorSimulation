#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from math import cos, sin, sqrt, tan
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
# from numpy import matrix
from math import atan, acos
from std_msgs.msg import Int32MultiArray, Float32MultiArray


class ReverseKinematicNode(Node):
    def __init__(self):
        super().__init__("ReverseKinematic")
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.speed_subscription = self.create_subscription(Float32MultiArray, 'gripper_state', self.gripper_callback, 10)

        # self.publisher = self.create_publisher(JointTrajectory, 'set_joint_trajectory', 10)

        self.subscriber = self.create_subscription(
            PointStamped,"joy_point", self.point_callback, 10)
        self.dh = 0.135
        self.uh = 0.147
        self.actual_x = 0.147
        self.actual_z = 0.273
        self.actual_y = 0.0
        # self.mhh = 0.06
        # self.position = [0, 0, 0, 0]
        # self.rotation = [0, 0, 0, 0]
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.theta4 = 0.0
        self.theta5 = 0.0
 
        self.prev_theta2 = 0.0
        self.prev_theta3 = 0.0
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.gripper_mode = 0

    def gripper_callback(self, gripper_state):
        self.gripper_mode = gripper_state.data[2]
        if self.gripper_mode == 0:
            self.theta4 = gripper_state.data[0]
            self.theta5 = gripper_state.data[1]

    def point_callback(self, point):
        # self.get_logger().info("calback")
        x = point.point.x
        y = point.point.y
        z = point.point.z
        # self.get_logger().info(str(point.point))

        # t4 = 0
        # # t5 = 3.14
  
        if (x < 0):
            t1 = atan(y/x) + 3.14
        else:
            t1 = atan(y/x)

        xn = x - 0.03/sqrt(1+tan(t1)**2)
        yn = y - tan(t1)*(x-xn)
        

        yp = z - (0.098) + 0.05
        xp = sqrt(xn**2+yn**2)

        a = xp**2 + yp**2 + self.dh**2 - (self.uh**2)

        try:
            delta = 16*(a**2)*(xp**2) - 16*((xp**2)+(yp**2))*((a**2)-4*(yp**2)*(self.dh**2))
            x0 = (4*a*xp-sqrt(delta))/(8*(xp**2)+8*(yp**2))
            # x0_2 = (4*a*xp+sqrt(delta))/(8*(xp**2)+8*(yp**2))
            if (yp < 0):
                # delta = 16*(a**2)*(xp**2) - 16*((xp**2)-(yp**2))*((a**2)+4*(yp**2)*(self.dh**2))
                # x0_1 = (4*a*xp-sqrt(delta))/(8*(xp**2)-8*(yp**2))
                x0 = (4*a*xp+sqrt(delta))/(8*(xp**2)+8*(yp**2))
                # x0 = min(abs(x0_1), abs(x0_2))
            if ((self.dh**2-x0**2) < 0):
                delta = 16*(a**2)*(xp**2) - 16*((xp**2)-(yp**2))*((a**2)+4*(yp**2)*(self.dh**2))
                x0 = (4*a*xp+sqrt(delta))/(8*(xp**2)-8*(yp**2))
            if ((self.dh**2-x0**2) < 0) and yp < 0:
                delta = 16*(a**2)*(xp**2) - 16*((xp**2)-(yp**2))*((a**2)+4*(yp**2)*(self.dh**2))
                x0 = (4*a*xp-sqrt(delta))/(8*(xp**2)-8*(yp**2))


            y0 = sqrt((self.dh**2) - (x0**2))
            self.theta1 = t1
            self.theta2 = atan(x0/y0)
            self.theta3 = acos((x0*(xp-x0)+y0*(yp-y0))/(sqrt((x0**2+y0**2)*((xp-x0)**2+(yp-y0)**2)))) - 1.57
            if self.gripper_mode == 1:
                self.theta4 = -self.theta3-self.theta2
                self.theta5 = t1

        except:
            self.theta1 = t1
            self.theta2 = 0.0
            self.theta3 = 0.0
            if self.gripper_mode == 1:
                self.theta4 = 0.0
                self.theta5 = 0.0

    def timer_callback(self):
        new_msg = JointState()
        new_msg.name = [
            "rotate_base__base",
            "down_manipulator__rotation_base",
            "up_manipulator_joint__down_manipulator_joint",
            "manipulator_hand_joint__up_manipulator_joint",
            "tool_joint__manipulator_hand_joint"
        ]
        new_msg.header.frame_id = "base_link"
        new_msg.header.stamp.sec = self.get_clock().now().to_msg().sec
        new_msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        new_msg.position = [self.theta1,
                            self.theta2,
                            self.theta3,
                            self.theta4,
                            self.theta5]
        # --------------------------------------------------------------
        # new_msg = JointTrajectory()
        # dur = Duration()
        # dur.sec = 0
        # dur.nanosec = 0
        # new_msg.header.frame_id = "world"
        # new_msg.header.stamp.sec = self.get_clock().now().to_msg().sec
        # new_msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        # # new_msg.joint_names = [
        #     # "rotate_base__base",
        #     # "down_manipulator__rotation_base",
        #     # "up_manipulator_joint__down_manipulator_joint",
        #     # "manipulator_hand_joint__up_manipulator_joint",
        #     # "tool_joint__manipulator_hand_joint"
        # # ]
        # # new_msg.joint_names = [
        # #     "manipulator_hand_joint__up_manipulator_joint"
        # # ]
        # # point = JointTrajectoryPoint()
        # # point.positions = [0.0]
        # # point.time_from_start = dur
        # # point.velocities = []
        # # point.accelerations = []
        # # point.effort = []
        # # new_msg.points.append(point)
        # # self.get_logger().info(f"theta: {self.theta1}, {self.theta2}, {self.theta3}, {self.theta4}, {self.theta5}")
        # # self.get_logger().info(f"vel: {point.velocities}")
        # msg = JointTrajectory(
        #     header=Header(stamp=Time(sec=0, nanosec=0), frame_id='world'), 
        #     joint_names=[
        #         "rotate_base__base",
        #         "down_manipulator__rotation_base",
        #         "up_manipulator_joint__down_manipulator_joint",
        #         "manipulator_hand_joint__up_manipulator_joint",
        #         "tool_joint__manipulator_hand_joint",
        #     ], 
        #     points=[JointTrajectoryPoint(positions=[
        #         self.theta1,
        #         self.theta2,
        #         self.theta3,
        #         self.theta4,
        #         self.theta5,
        #     ], velocities=[], accelerations=[], effort=[], time_from_start=Duration(sec=0, nanosec=0))])


        # msg = JointTrajectory(
        #     header=Header(stamp=Time(sec=0, nanosec=0), frame_id='world'), 
        #     joint_names=[
        #         "base_joint"
        #     ], 
        #     points=[JointTrajectoryPoint(positions=[
        #         2.0
        #     ], velocities=[], accelerations=[], effort=[], time_from_start=Duration(sec=0, nanosec=0))])
        self.publisher.publish(new_msg)


def main(arg=None):
    rclpy.init(args=arg)
    node = ReverseKinematicNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
