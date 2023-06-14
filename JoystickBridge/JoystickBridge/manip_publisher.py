#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from math import cos, sin, sqrt, tan
from geometry_msgs.msg import PoseStamped
import serial
# from numpy import matrix
from math import atan, acos
import time
import sys
import socket
from std_msgs.msg import Int32MultiArray, Float32MultiArray


class ManipPublisher(Node):
    def __init__(self):
        super().__init__("manip_publisher")
        self.publisher = self.create_publisher(PointStamped, 'joy_point', 10)
        self.gripper_publisher = self.create_publisher(Float32MultiArray, 'gripper_state', 10)
        self.flags_publisher = self.create_publisher(Int32MultiArray, 'error_flags', 10)

        self.speed_subscription = self.create_subscription(Int32MultiArray, 'joy_speed', self.speed_callback, 10)
        # self.subscriber = self.create_subscription(
        #     PointStamped,"joy_point", self., 10)


        # self.opesn_serial()
        # self.read_tim = self.create_timer(0.1, self.read_serial)
        self.publish_tim = self.create_timer(0.001, self.publish_points)
        self.speed_prescaler = 1500000
        self.x_speed = 0.0
        self.y_speed = 0.0
        self.z_speed = 0.0
        self.gripper_hand = 0.0
        self.gripper_tool = 0.0
        self.gripper_hand_speed = 0.0
        self.gripper_tool_speed = 0.0
        self.x_position = 0.1
        self.y_position = 0.01
        self.z_position = 0.15
        self.goto_flag = 0
        self.x_err_flag = 0
        self.y_err_flag = 0
        self.z_err_flag = 0
        self.prev_x_err_flag = 0
        self.prev_y_err_flag = 0
        self.prev_z_err_flag = 0      
       
    def speed_callback(self, speed_arr):
        speeds = []
        id = []
        for i in speed_arr.data[:3]:
            speeds.append(i) 
        for i in speed_arr.data[3:]:
            id.append(i)
        if id == [2, 1]:
            # arm control
            self.x_speed = speeds[0]
            self.y_speed = speeds[1]
            self.z_speed = speeds[2]
        if id == [2, 2]:
            # gripper
            self.gripper_hand_speed = speeds[0]
            self.gripper_tool_speed = speeds[1]

        if id == [3, 0]:
            # goto home
            self.goto_flag = True
            self.goto_point([0.1, 0.01, 0.15])

    def goto_point(self, point):
        x_step = -(self.x_position - point[0])/20000
        y_step = -(self.y_position - point[1])/20000
        z_step = -(self.z_position - point[2])/20000
        for i in range(20000):
            self.x_position += x_step
            self.y_position += y_step
            self.z_position += z_step

            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = self.x_position
            point.point.y = self.y_position
            point.point.z = self.z_position
            self.publisher.publish(point)
        self.goto_flag = False
    def publish_points(self):
        if self.goto_flag == False:
            point = PointStamped()
            grip_arr = Float32MultiArray()
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = self.x_position
            point.point.y = self.y_position
            point.point.z = self.z_position
            self.x_position += self.x_speed/self.speed_prescaler
            self.y_position += self.y_speed/self.speed_prescaler
            self.z_position += self.z_speed/self.speed_prescaler

            if sqrt(self.x_position**2 + self.y_position**2) > 0.28:
                self.x_position -= self.x_speed/self.speed_prescaler
                self.y_position -= self.y_speed/self.speed_prescaler

            self.gripper_hand += self.gripper_hand_speed/self.speed_prescaler*10
            self.gripper_tool += self.gripper_tool_speed/self.speed_prescaler*30

            # boundaries
            # TODO adjust max and min values
            self.x_position = min(max(self.x_position, 0.05), 0.25)
            self.y_position = min(max(self.y_position, -0.2), 0.2)
            self.z_position = min(max(self.z_position, 0.01), 0.18)
            
            # if self.x_position >= 0.24 or self.x_position <= 0.06:
            #     self.x_err_flag = 1
            # else:
            #     self.x_err_flag = 0

            # if self.y_position >= 0.19 or self.y_position <= -0.19:
            #     self.y_err_flag = 1
            # else:
            #     self.y_err_flag = 0

            # if self.z_position >= 0.17 or self.z_position <= 0.02:
            #     self.z_err_flag = 1
            # else:
            #     self.z_err_flag = 0
            # self.check_flags()
            # self.gripper_hand = min(max(self.x_position, -1.7), 1.7)
            grip_arr.data = [float(self.gripper_hand), float(self.gripper_tool)]
            
            # self.get_logger().info("Publishing point")
            # self.get_logger().info(f"x_position: {self.x_position}   y_position: {self.y_position} z_position: {self.z_position}")
            self.publisher.publish(point)
            self.gripper_publisher.publish(grip_arr)
    
    def check_flags(self):
        if self.prev_x_err_flag != self.x_err_flag or self.prev_y_err_flag != self.y_err_flag or self.prev_z_err_flag != self.z_err_flag:
            flags_msg = Int32MultiArray()
            flags_msg.data = [self.x_err_flag, self.y_err_flag, self.z_err_flag]

            self.get_logger().info(f"Flags changed, cur state: {[self.x_err_flag, self.y_err_flag, self.z_err_flag]}")
            
            self.flags_publisher.publish(flags_msg)
            self.prev_x_err_flag = self.x_err_flag
            self.prev_y_err_flag = self.y_err_flag
            self.prev_z_err_flag = self.z_err_flag

def main(arg=None):
    rclpy.init(args=arg)
    node = ManipPublisher()
    rclpy.spin(node)
    rclpy.shutdown()




if __name__ == "_main_":
    main()