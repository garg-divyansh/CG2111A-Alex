import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

class ScanMatcher(Node):
    def __init__(self):
        super().__init__('scan_matcher')

        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        self.prev_points = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def scan_to_points(self, scan):
        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if r > 0:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += scan.angle_increment
        return np.array(points)

    def callback(self, scan):
        curr_points = self.scan_to_points(scan)

        if self.prev_points is None:
            self.prev_points = curr_points
            return

        # VERY SIMPLE motion estimate (centroid shift)
        prev_centroid = np.mean(self.prev_points, axis=0)
        curr_centroid = np.mean(curr_points, axis=0)

        dx, dy = curr_centroid - prev_centroid

        self.x += dx
        self.y += dy

        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)

        self.pub.publish(msg)

        self.prev_points = curr_points


def main():
    rclpy.init()
    node = ScanMatcher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
