#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose as TurtlePose
from geometry_msgs.msg import Vector3, Quaternion
from ros2_text_encoder.srv import GetTurtlePose
import math

class TurtlePoseService(Node):
    def __init__(self):
        super().__init__('turtle_pose_service')
        # Subscribe to turtle1/pose
        self.pose_sub = self.create_subscription(TurtlePose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None
        # Create service
        self.srv = self.create_service(GetTurtlePose, 'get_turtle_pose', self.get_pose_callback)
        self.get_logger().info('Turtle pose service ready.')

    def pose_callback(self, msg: TurtlePose):
        self.pose = msg

    def get_pose_callback(self, request, response):
        if self.pose is None:
            self.get_logger().warn('No pose data yet.')
            response.position = Vector3()
            response.orientation = Quaternion(w=1.0)
            return response

        # Convert 2D pose (x, y, theta) to Vector3 + Quaternion
        response.position = Vector3(x=self.pose.x, y=self.pose.y, z=0.0)
        qz = math.sin(self.pose.theta / 2.0)
        qw = math.cos(self.pose.theta / 2.0)
        response.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        self.get_logger().info(
            f"Returned pose -> pos({self.pose.x:.2f}, {self.pose.y:.2f}) theta={self.pose.theta:.2f}"
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
