#!/usr/bin/env python3
import rclpy
import pandas as pd
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

class Raceline(Node):

    def __init__(self):
        super().__init__('Raceline', automatically_declare_parameters_from_overrides=True)
        self.get_logger().info('yy')
        file = get_package_share_directory('rust_mpc') + "/data/Spielberg_raceline.csv"
        self.publisher = self.create_publisher(PoseArray,"raceline",0)

        self._load_raceline(file)

    def _load_raceline(self, filename: str):
        self.get_logger().info('yy')
        self.raceline = pd.read_csv(filename, sep=';')
        msg = PoseArray()
        print(self.raceline.columns.values)
        for _, row in self.raceline.iterrows():
            pose = Pose()
            pose.position.x = row[' x_m']
            pose.position.y = row[' y_m']
            msg.poses.append(pose)

        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def publish_raceline(self):
        for point in self.raceline:
            return


def main(args=None):
    rclpy.init(args=args)

    board_node = Raceline()
    rclpy.spin(board_node)

    board_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()