#!/usr/bin/env python3
import rclpy
import pandas as pd
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry

class Raceline(Node):

    def __init__(self):
        super().__init__('Raceline', automatically_declare_parameters_from_overrides=True)
        self.get_logger().info('yy')
        file = get_package_share_directory('rust_mpc') + "/data/Spielberg_raceline.csv"

        # TODO: should be a service

        self.raceline_sub = self.create_subscription(PoseArray,"raceline",self.save_raceline, 0)
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.calculate_control, 0)

        self.raceline = None

    def calculate_control(self, odom_msg: Odometry):
        print(odom_msg)


    def save_raceline(self, raceline_msg: PoseArray):
        self.raceline = raceline_msg


def main(args=None):
    rclpy.init(args=args)

    board_node = Raceline()
    rclpy.spin(board_node)

    board_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()