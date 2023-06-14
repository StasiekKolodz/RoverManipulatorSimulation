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



class JoystickHandler(Node):
    def __init__(self):
        super().__init__("joystick_handler")
        self.publisher = self.create_publisher(Int32MultiArray, 'joy_speed', 10)
        self.flags_subscriber = self.create_subscription(Int32MultiArray, "error_flags", self.flags_callback, 10)
        # self.subscriber = self.create_subscription(
        #     PointStamped,"joy_point", self.point_callback, 10)
        # self.speed_publisher = self.create_publisher(Int32MultiArray,"joy_speed_prescaler", 10)
        self.port_name = "/dev/ttyACM0"
        self.get_logger().info("joystick handler created")
        self.eth_address = ("192.168.1.88", 5000)
        self.start_comunication()


        # self.opesn_serial()
        # self.read_tim = self.create_timer(0.1, self.read_serial)
        # self.publish_tim = self.create_timer(0.001, self.publish_points)
        self.speed_prescaler = 15000
        self.x_speed = 0.0
        self.y_speed = 0.0
        self.z_speed = 0.0
        self.x_position = 0.05
        self.y_position = 0.01
        self.z_position = 0.1
        self.current_id = []
        self.sending_flag = False

    def flags_callback(self, flags):
        self.sending_flag = True
        self.get_logger().info(f"flags callback: {flags.data}")
        data_list = []
        for i in range(19):
            if i == 1:
                data = '#'
            elif i == 2:
                data = str(flags.data[0])
            elif i == 3:
                data = str(flags.data[1])
            elif i == 4:
                data = str(flags.data[2])
            else:
                data = 'x'
            # self.get_logger().info(f"data: {len(bytes(data, 'utf-8'))}")
            while self.ethernet.sendall(bytes(data, 'utf-8')) is not None:
                pass
            time.sleep(1)
            data = "x"
            self.ethernet.sendall(bytes(data, 'utf-8'))
        # buffer = []
        # for i in range(19):
        #     ret = self.ethernet.recv(1)
        #     buffer.append(ret.hex())
        # self.get_logger().info(f"ret buffer: {buffer}")
        self.sending_flag = False

    def start_comunication(self, interface='eth'):
        if interface == 'eth' or interface == 'ethernet':
            ok = self.connect_ethernet()
            if ok:
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
            return True 
        except Exception as e:
            return False
            self.get_logger().info(f"error open ethernet socket: {str(e)}")      

    def read_ethernet(self):
        if self.sending_flag == False:
            buffer = []
            self.current_id = []
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
                        buffer.append(msg)
                        if i in [1, 2]:
                            # self.current_id.append(int.from_bytes(msg, byteorder='little' ,signed="False"))
                            self.current_id.append(int(msg.decode("ascii")))


                    # self.x_speed = int(buffer[3], 16)
                    # self.y_speed = int(buffer[4],16)
                    # self.z_speed = int(buffer[5],16)

                    # self.get_logger().info(f"id: {self.current_id}")
                    if self.current_id == [2,1] or self.current_id == [2,2]:
                        self.x_speed = int.from_bytes(buffer[3], byteorder='little' ,signed="True")
                        self.y_speed = int.from_bytes(buffer[4], byteorder='little',signed="True")
                        self.z_speed = int.from_bytes(buffer[5], byteorder='little', signed="True")
                        c = buffer[6].decode('ascii')
                        if c == 'x':
                            button_flag = 0
                        else:
                            button_flag = int(buffer[6].decode('ascii'))
                        # self.get_logger().info(f"button flag: {button_flag}")
                        # self.get_logger().info(f"x: {self.x_speed}, y: {self.y_speed}, z: {self.z_speed}")
                        # self.get_logger().info(f"Received incrrect data frame: {buffer}")
                        int_msg = Int32MultiArray()
                        int_msg.data = [self.x_speed, self.y_speed, self.z_speed, self.current_id[0], self.current_id[1], button_flag]
                        self.publisher.publish(int_msg)
                    if self.current_id == [3, 0] or self.current_id == [3, 1]:
                        int_msg = Int32MultiArray()
                        button_flag = int(buffer[3].decode('ascii'))
                        # self.get_logger().info(f"button flag: {button_flag}")
                        int_msg.data = [button_flag, 0, 0, self.current_id[0], self.current_id[1]]
                        self.publisher.publish(int_msg)
                        # self.get_logger().info(f"BUTTON: {buffer}")
                    

            except Exception as e:
                self.get_logger().info(f"Exeption: {str(e)}")


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

    def decode(self, number):
        binNum = bin(number)
        if binNum[0] == '1':
            # binNum = ~binNum + 1
            ret = ""
            ret  = "".join('1' if  b =='0' else '0' for b in binNum)
            binNum = ret
        return int(binNum[2:], base=2)

    def twos_comp(self, val, bits):
        """compute the 2's complement of int value val"""
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val 


    # def publish_points(self):
    #     point = PointStamped()
    #     point.header.stamp = self.get_clock().now().to_msg()
    #     point.point.x = self.x_position
    #     point.point.y = self.y_position
    #     point.point.z = self.z_position

    #     self.x_position += self.x_speed/self.speed_prescaler
    #     self.y_position += self.y_speed/self.speed_prescaler
    #     self.z_position += self.z_speed/self.speed_prescaler

    #     # boundaries
    #     # TODO adjust max and min values
    #     self.x_position = min(max(self.x_position, 0.01), 0.3)
    #     self.y_position = min(max(self.y_position, -0.3), 0.3)
    #     self.z_position = min(max(self.z_position, 0.01), 0.3)

    #     self.get_logger().info("Publishing point")
    #     self.get_logger().info(f"x_position: {self.x_position}   y_position: {self.y_position} z_position: {self.z_position}")
    #     self.publisher.publish(point)
    



def main(arg=None):
    rclpy.init(args=arg)
    node = JoystickHandler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "_main_":
    main()