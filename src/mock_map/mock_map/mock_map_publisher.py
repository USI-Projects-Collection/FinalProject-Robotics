#!/usr/bin/env python3
import rclpy, numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist

class MockMapPublisher(Node):
    def __init__(self):
        super().__init__('mock_map_publisher')
        # 5 cm resolution on a 100 × 100 grid → physical map size 5 m × 5 m
        res, w, h = 0.05, 100, 100
        ox, oy    = -2.5, -2.5

        grid = np.full((h, w), 0, dtype=np.int8)     # 0 = libero
        # for x0, y0, dx, dy in [(-2, -1, 1, 6), (1.5, 1, 3, 1), (-0.5, 2, 1, 1)]:
        self.tower_goals = [(1.3, -1.3, 0.15, 0.25), (1.325, 1.55, 0.15, 0.25), (-1.45, 1.575, 0.15, 0.25), (-1.4, -1.3, 0.15, 0.25)]
        for x0, y0, dx, dy in self.tower_goals:
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

        self.pub   = self.create_publisher(OccupancyGrid, '/map', 1) # mantieni in coda al massimo 1 messaggio”: se ne arriva un altro prima che il subscriber lo legga, il più vecchio viene scartato.
        self.timer = self.create_timer(1.0, self._tick) # intervallo (in secondi) fra due chiamate consecutive della callback

        self.pub_cmd   = self.create_subscription(Twist, '/rm0/cmd_vel', self._cmd_callback, 1)
        self.cmd_vel = None
        self.current_tower = 0
        self.flag = False


        # publisher to send goal pose at 10Hz
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.goal_timer = self.create_timer(2, self.publish_goal)

    def _tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

    def _cmd_callback(self, msg):
        self.cmd_vel = msg
        v = msg.linear.x
        w = msg.angular.z
        self.get_logger().info(f"Received cmd_vel: v={v:.3f} m/s, w={w:.3f} rad/s")

        # ---- detect arrival at tower ----
        # consider the robot "stopped" when both linear and angular speeds are ≈ 0
        stopped = abs(v) < 0.01 and abs(w) < 0.01

        if stopped and not self.flag:
            # advance to next tower (wrap around at the end)
            if self.current_tower < len(self.tower_goals) - 1:
                self.current_tower += 1
            else:
                self.current_tower = 0
            self.flag = True
        elif not stopped:
            # reset flag as soon as the robot moves again
            self.flag = False

        self.get_logger().info(f"cmd_vel: v={v:.3f}, w={w:.3f} → current_tower: {self.current_tower}")


    def publish_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.tower_goals[self.current_tower][0]
        msg.pose.position.y = self.tower_goals[self.current_tower][1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(MockMapPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()