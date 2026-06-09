#!/usr/bin/env python3
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

import matplotlib.image as mpimg
import numpy as np
 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# Some constants
ACIDITY_MAP_PATH = '/ws/src/acid_techno/resource/AcidityMap.jpg'
GRID_SIZE_X = 6.0
GRID_SIZE_Y = 6.0

class ReadAcidityNode(Node):
    acidity_map: np.ndarray
    map_height: int
    map_width: int

    def __init__(self):
        super().__init__('read_acidity_node')
 
        # self.sub = self.create_subscription(
        #     Odometry,
        #     '/odom',
        #     self.odom_callback,
        #     10
        # )

        self.sub = self.create_subscription(
            Pose2D,
            '/corrected_odom',
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(Float64, '/acidity', 20)

        self.acidity_map = mpimg.imread(ACIDITY_MAP_PATH)
        self.map_height, self.map_width, _ = self.acidity_map.shape

        self.get_logger().info('Acidity sensor operational')
 
    def odom_callback(self, msg: Pose2D):
        x = msg.x
        y = msg.y

        relative_x = x / (GRID_SIZE_X / 2) * (self.map_width / 2)
        relative_y = y / (GRID_SIZE_Y / 2) * (self.map_height / 2)

        if  abs(relative_x) >= self.map_width / 2 or abs(relative_y) > self.map_height / 2:
            self.get_logger().warn('Robot out of bounds')
            return
        
        pixel_x = int(relative_x + (self.map_width / 2)) 
        pixel_y = int(relative_y + (self.map_height / 2))

        brightness, _, _ = self.acidity_map[pixel_y, pixel_x]
        pos_ph = 6 + brightness * (3 / 255)

        #self.get_logger().info(f'Acidity at given location ({x}, {y}) is {pos_ph}ph')

        temp = Float64()
        temp.data = pos_ph
        self.pub.publish(temp)

def main(args=None):
    rclpy.init(args=args)
    node = ReadAcidityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
