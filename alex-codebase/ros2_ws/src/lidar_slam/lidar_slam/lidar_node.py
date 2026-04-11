import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import sys
import numpy as np

sys.path.append('/ros2_ws')
from lidar.alex_lidar import (
    lidarConnect, lidarStatus,
    startScan, stopScan, process_scan
)

NUM_READINGS = 360
RANGE_MIN    = 0.15
RANGE_MAX    = 6.0

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.scan_generator = None
        self.scan_state = {"r": 0, "buff": [], "doScan": False}
        self.raw_count = 0

        self.get_logger().info("Connecting to LiDAR...")
        self.lidar = lidarConnect()
        lidarStatus(self.lidar)

        self._start_generator()
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info("Lidar node running.")

    def _start_generator(self):
        try:
            try:
                stopScan(self.lidar)
            except Exception:
                pass
            self.lidar.set_motor_pwm(660)
            gen_fn = startScan(self.lidar, mode=1)  # Express/CAPSULED — motion-stable
            self.scan_generator = gen_fn()
            self.scan_state = {"r": 0, "buff": [], "doScan": False}
            self.raw_count = 0
            self.get_logger().info("Scan generator started (mode 1 - Express).")
        except Exception as e:
            self.get_logger().error(f"Failed to start generator: {e}")
            self.scan_generator = None

    def update(self):
        if self.scan_generator is None:
            self._start_generator()
            return

        try:
            for _ in range(500):
                raw = next(self.scan_generator)
                self.raw_count += 1

                self.scan_state, result = process_scan(
                    (self.raw_count, raw), self.scan_state
                )

                if result is not None:
                    self._publish(result)
                    return  # one complete scan per timer tick

        except StopIteration:
            self.get_logger().warn("Generator exhausted, restarting...")
            self._start_generator()
        except Exception as e:
            self.get_logger().error(f"Update error: {e}")
            self._start_generator()

    def _publish(self, result):
        angles_deg, distances_mm, _ = result

        angles_deg   = np.array(angles_deg,   dtype=np.float64)
        distances_mm = np.array(distances_mm, dtype=np.float64)

        # Filter zero distances
        valid_mask   = distances_mm > 0
        angles_deg   = angles_deg[valid_mask]
        distances_mm = distances_mm[valid_mask]

        if len(angles_deg) < 50:
            self.get_logger().warn(f"Sparse scan: {len(angles_deg)} points, skipping")
            return

        # Bucket into fixed 360-point grid (one per degree)
        ranges  = np.full(NUM_READINGS, float('inf'))
        indices = np.floor(angles_deg % 360).astype(int)
        indices = np.clip(indices, 0, NUM_READINGS - 1)
        dist_m  = distances_mm / 1000.0

        for i in range(len(dist_m)):
            d   = dist_m[i]
            idx = indices[i]
            if RANGE_MIN < d < RANGE_MAX:
                if d < ranges[idx]:  # keep closest reading per degree bucket
                    ranges[idx] = d

        valid_count = int(np.sum(~np.isinf(ranges)))
        if valid_count < 30:
            self.get_logger().warn(f"Too few valid ranges: {valid_count}, skipping")
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp     = self.get_clock().now().to_msg()
        scan_msg.header.frame_id  = "laser"
        scan_msg.angle_min        = 0.0
        scan_msg.angle_max        = 2.0 * math.pi * (NUM_READINGS - 1) / NUM_READINGS
        scan_msg.angle_increment  = 2.0 * math.pi / NUM_READINGS
        scan_msg.time_increment   = 0.0
        scan_msg.scan_time        = 0.1
        scan_msg.range_min        = RANGE_MIN
        scan_msg.range_max        = RANGE_MAX
        scan_msg.ranges           = ranges.tolist()

        self.publisher.publish(scan_msg)
        self.get_logger().info(f"Published: {valid_count}/{NUM_READINGS} valid points")


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stopScan(node.lidar)
            node.lidar.set_motor_pwm(0)
            node.lidar.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()