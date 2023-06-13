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

class JoystickHandler(Node):
    def __init__(self):
        super().__init__("joystick_handler")
        self.publisher = self.create_publisher(PointStamped, 'joy_point', 10)
        # self.subscriber = self.create_subscription(
        #     PointStamped,"joy_point", self.point_callback, 10)

        self.port_name = "/dev/ttyACM0"
        self.get_logger().info("joystick handler created")
        self.eth_address = ("192.168.1.88", 5000)
        self.start_comunication()


        # self.opesn_serial()
        # self.read_tim = self.create_timer(0.1, self.read_serial)
        # self.publish_tim = self.create_timer(0.1, self.publish_points)
        self.speed_prescaler = 5000
        self.x_speed = 0.0
        self.y_speed = 0.0
        self.z_speed = 0.0
        self.x_position = 0.05
        self.y_position = 0.25
        self.z_position = 0.05

    def start_comunication(self, interface='eth'):
        if interface == 'eth' or interface == 'ethernet':
            self.connect_ethernet()
            self.read_tim = self.create_timer(0.1, self.read_ethernet)

        elif interface =='serial' or interface == 'uart':
            self.open_serial()
            self.read_tim = self.create_timer(0.1, self.read_serial)
        else:
            self.get_logger().info(f"Unknown interface: {interface}")

    def open_serial(self):
        try:
            self.port = serial.Serial(self.port_name, baudrate=115200)
            self.port.reset_input_buffer()

            self.get_logger().info(f"Opened serial: {self.port.name}")

        except Exception as e:
            self.get_logger().info(f"error open serial port: {str(e)}")

    def connect_ethernet(self):
        self.get_logger().info("connecting to ethernet...")   
        try:
            self.ethernet = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ethernet.connect(self.eth_address)
            self.get_logger().info(f"Ethernet connected succesfully to : {self.eth_address}")     
        except Exception as e:
            self.get_logger().info(f"error open ethernet socket: {str(e)}")      

    def read_ethernet(self):
        buffer = []
        start_msg = self.ethernet.recv(1)
        # self.get_logger().info(f"Received incrrect data frame: {start_msg.hex}")
        try:
            if start_msg.decode("ascii")== '#':
                buffer.append('#')
                    # self.get_logger().info(f"Received incrrect data frame: {msg[3]}")
        #             self.get_logger().info(f"Received incrrect data frame: {msg[2:5]}")
        #             self.get_logger().info(f"Received incrrect data frame: {msg[2:5].hex()}")
                for i in range(1,19):
                    msg = self.ethernet.recv(1)
                    buffer.append(msg.hex())
                self.x_speed = int(buffer[3], 16)
                self.x_speed = 
                # self.y_speed = int.from_hex(buffer[4])
                # self.z_speed = int.from_hex(buffer[5])
                self.get_logger().info(f"x: {self.x_speed}, y: {self.y_speed}, z: {self.z_speed}")
                self.get_logger().info(f"Received incrrect data frame: {buffer}")
        except Exception as e:
            self.get_logger().info(f"Exeption: {str(e)}")
        #     self.get_logger().info(f"Data: {start_msg}")
        #     msg = self.ethernet.recv(18)
        #     # self.get_logger().info(f"Received incrrect data frame: {msg}")
        #     # self.get_logger().info(f"Frame ID: {msg[0:2]}")
        #     # self.get_logger().info(f"Frame  decode: {msg.decode()}")
        #     if msg[0:2].decode() == "21":
        #         self.get_logger().info(f"Frame : {msg[5:].decode(encoding='ascii')}")
        #         self.get_logger().info(f"Frame : {msg.decode(encoding='ascii')}")

        #         self.get_logger().info(f"Received incrrect data frame: {msg[2:5].hex()}")
        #         self.get_logger().info(f"Received incrrect data frame: {msg}")

        #         self.get_logger().info("manip frame received")
                # self.get_logger().info(f"x_speed: {msg[2]}")
                # self.get_logger().info(f"y_speed: {msg[3]}")
                # self.get_logger().info(f"z_speed: {msg[4]}")
                # self.get_logger().info(f"dupa: {int.from_bytes(msg[2:5], byteorder=sys.byteorder)}")
        #     self.x_speed = int.from_bytes(msg[1], byteorder=sys.byteorder)
        #     self.y_speed = int.from_bytes(msg[2], byteorder=sys.byteorder)
        #     self.z_speed = int.from_bytes(msg[3], byteorder=sys.byteorder)
                    #     self.get_logger().info(f"Received incrrect data frame: {msg}")

            # lub
            # self.x_speed = imsg[1]
            # self.y_speed = msg[2]
            # self.z_speed = msg[3]
        # else:
        #     self.get_logger().info(f"Received incrrect data frame: {msg}")

    def read_serial(self):
        msg = s.recv(19)
        print("Reading data")
        self.port.reset_input_buffer()

        data = self.port.read(1)
        if data:
            print("HEX: " + str(data.hex()))
            print("xd: " + str(int.from_bytes(data, byteorder=sys.byteorder)))
            print("idx: " + str(data[0]))
            print("bin: " + str(bin(data[0])))
            self.x_speed = int.from_bytes(data, byteorder=sys.byteorder)
            self.x_position = self.x_speed/1000 + 0.05
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

        self.x_position += self.x_speed/self.speed_prescaler
        self.y_position += self.y_speed/self.speed_prescaler
        self.z_position += self.z_speed/self.speed_prescaler

        # boundaries
        # TODO adjust max and min values
        self.x_position = min(max(self.x_position, 0.0), 0.3)
        self.y_position = min(max(self.x_position, -0.3), 0.3)
        self.z_position = min(max(self.z_position, 0), 0.3)

        # self.get_logger().info("Publishing point")
        # self.get_logger().info(f"x_position: {self.x_position}")
        self.publisher.publish(point)
    



def main(arg=None):
    rclpy.init(args=arg)
    node = JoystickHandler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
