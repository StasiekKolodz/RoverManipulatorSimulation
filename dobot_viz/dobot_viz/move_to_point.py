#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, sqrt
from numpy import matrix
from rclpy.action import ActionClient
from rclpy.task import Future
from dobot_msgs.action import PointToPoint
from geometry_msgs.msg import PointStamped


class move_to_point(Node):
    def __init__(self):
        super().__init__("move_to_point")
        self.get_logger().error("dziala")

        # self.subscriber = self.create_subscription(
        #     JointState, "dobot_joint_states", self.robot_position_callback, 10)
        # self.publisher_joint = self.create_publisher(JointState, 'joint_states', 10)

        self.point_subscriber = self.create_subscription(
            PointStamped, "clicked_point", self.point_callback, 10)
        self.act_cli = ActionClient(
            self, PointToPoint, "/PTP_action")
        self.goal = PointToPoint.Goal()

        self.dh = 0.135
        self.uh = 0.147
        self.mhh = 0.06
        self.position = [0, 0, 0, 0]
        self.rotation = [0, 0, 0, 0]
        timer_period = 0.01  # seconds
        self.rviz_joint = [0.0,0.0,0.0,0.0,0.0]
        self.get_logger().info("robot init")
        # self.timer = self.create_timer(timer_period, self.timer_callback)
   
    def point_callback(self, point):
        self.get_logger().info("calback")
        x = point.point.x
        y = point.point.y
        z = point.point.z
        self.get_logger().info(str(point.point))
        self.get_logger().info(str("x"))
        self.get_logger().info(str(x))
        x_offset = 0#-27.76
        y_offset = 0#12.6
        z_offset = 0#-51
        self.move_PTP(1, [float((x)*1000+x_offset),float((y)*1000+ y_offset),float((z)*1000 + z_offset),0.0])
        # self.move_PTP(1, [119.091, -156.947, 43.911, 0.0])
        
    def move_PTP(self, motion_type, target_pose):
        self.goal.motion_type = motion_type
        self.goal.target_pose = target_pose

        self.act_cli.wait_for_server()
        self.future = self.act_cli.send_goal_async(self.goal)
        self.future.add_done_callback(self.goal_was_send)

    def goal_was_send(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.done_callback)
 
    def done_callback(self, arg):
        self.get_logger().info('done_callback')

    # def position_callback(self, pos):


    # def robot_position_callback(self, pos):


move_to_point
def main(arg=None):
    rclpy.init(args=arg)
    node = move_to_point()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
