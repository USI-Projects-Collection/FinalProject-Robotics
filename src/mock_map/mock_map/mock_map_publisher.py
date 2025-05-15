#!/usr/bin/env python3
import rclpy, numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

class MockMapPublisher(Node):
    def __init__(self):
        super().__init__('mock_map_publisher')
        res, w, h = 0.05, 200, 200         # 10 m × 10 m, 5 cm/cella
        ox, oy    = -5.0, -5.0

        grid = np.full((h, w), 0, dtype=np.int8)     # 0 = libero
        for x0, y0, dx, dy in [(-2, -1, 1, 6), (1.5, 1, 3, 1), (-0.5, -0.5, 1, 1)]:
        # for x0, y0, dx, dy in [(-2, -3, 0.5, 1)]:
            ix,  iy  = int((x0-ox)/res),  int((y0-oy)/res)
            ixe, iye = int((x0+dx-ox)/res), int((y0+dy-oy)/res)
            grid[iy:iye, ix:ixe] = 100                # 100 = occupato

        # ---- debug: quante celle sono state marcate a 100? ----
        occ_cells = int(np.count_nonzero(grid == 100))
        self.get_logger().info(f"Occupancy cells set to 100: {occ_cells}")

        self.msg = OccupancyGrid()
        self.msg.header.frame_id = 'map'
        self.msg.info.resolution = res
        self.msg.info.width      = w
        self.msg.info.height     = h
        self.msg.info.origin = Pose()
        self.msg.info.origin.position.x = ox
        self.msg.info.origin.position.y = oy
        self.msg.info.origin.orientation.w = 1.0
        self.msg.data = grid.flatten().tolist()

        self.pub   = self.create_publisher(OccupancyGrid, '/map', 1)
        self.timer = self.create_timer(1.0, self._tick)

    def _tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    rclpy.spin(MockMapPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()