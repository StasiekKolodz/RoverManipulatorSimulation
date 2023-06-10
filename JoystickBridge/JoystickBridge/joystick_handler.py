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

class JoystickHandler(Node):
    def __init__(self):
        super().__init__("joystick_handler")
        self.publisher = self.create_publisher(PointStamped, 'joy_point', 10)
        # self.subscriber = self.create_subscription(
        #     PointStamped,"joy_point", self.point_callback, 10)

        self.port_name = "/dev/ttyACM0"
        self.get_logger().info("joystick handler created")
        self.open_serial()
        self.read_tim = self.create_timer(1.0, self.read_serial)
        self.publish_tim = self.create_timer(0.1, self.publish_points)
        self.x_speed = 0.05
        self.y_speed = 0.0
        self.z_speed = 0.0
        self.x_position = 0.01
        self.y_position = 0.0
        self.z_position = 0.1

    def open_serial(self):
        try:
            self.port = serial.Serial(self.port_name, baudrate=115200)
            self.port.reset_input_buffer()

            self.get_logger().info(f"Opened serial: {self.port.name}")

        except Exception as e:
            self.get_logger().info(f"error open serial port: {str(e)}")


    def read_serial(self):
        print("Reading data")
        data = self.port.read(1)
        if data:
            print("HEX: " + str(data.hex()))
            print("xd: " + str(int.from_bytes(data, byteorder=sys.byteorder)))
            print("idx: " + str(data[0]))
            print("bin: " + str(bin(data[0])))
            self.x_speed = int.from_bytes(data, byteorder=sys.byteorder)
            
            self.get_logger().info(f"x_speed: {self.x_speed}")
            # print("decode: " + str(data.decode()))
        else:
            print("no data")


    def publish_points(self):
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = self.x_position
        point.point.y = self.y_position
        point.point.z = self.z_position
        if self.x_position > 0.3:
            self.x_position = 0
        self.x_position += self.x_speed/60000
        self.y_position += self.y_speed/50000
        self.z_position += self.z_speed/50000
        self.get_logger().info("Publishing point")
        self.get_logger().info(f"x_position: {self.x_position}")
        self.publisher.publish(point)
    



def main(arg=None):
    rclpy.init(args=arg)
    node = JoystickHandler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
