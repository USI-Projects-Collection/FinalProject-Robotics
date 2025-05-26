#!/usr/bin/env python3
import rclpy, numpy as np
from rclpy.node import Node
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Bool

class MockMapPublisher(Node):
    def __init__(self):
        super().__init__('mock_map_publisher')

        res, w, h = 0.05, 100, 100
        ox, oy    = -2.5, -2.5

        grid = np.full((h, w), 0, dtype=np.int8)     # 0 = libero

        self.tower_goals = [(1.5, -1.5, 0.15, 0.25), (-1.5, 1.5, 0.15, 0.25), (1.5, 1.5, 0.15, 0.25), (-1.5, -1.5, 0.15, 0.25)]
        self.obstacles = [(0.0,  1.5, 0.2),(-1.25, -0.5, 0.3), (0.0, -1.5, 0.3), (0.5, 2.0, 0.2)] 

        for x0, y0, _, _ in self.tower_goals:
            r = 0.2
            cx = int((x0 - ox) / res)
            cy = int((y0 - oy) / res)
            rc = int(r / res) + 1  # raggio in celle

            for dy in range(-rc, rc + 1):
                for dx in range(-rc, rc + 1):
                    if dx*dx + dy*dy <= rc*rc:  # dentro il cerchio
                        # Disegna tutti i quadranti del cerchio
                        for gx, gy in [
                            (cx + dx, cy + dy),  # Quadrante 1
                            (cx - dx, cy + dy),  # Quadrante 2
                            (cx + dx, cy - dy),  # Quadrante 3
                            (cx - dx, cy - dy),  # Quadrante 4
                        ]:
                            if 0 <= gx < w and 0 <= gy < h:
                                grid[gy, gx] = 100

        # 2️⃣  Rimpiazza il ciclo che disegna i rettangoli con il cerchio
        for x0, y0, r in self.obstacles:
            cx = int((x0 - ox) / res)
            cy = int((y0 - oy) / res)
            rc = int(r / res) + 1  # raggio in celle

            for dy in range(-rc, rc + 1):
                for dx in range(-rc, rc + 1):
                    if dx*dx + dy*dy <= rc*rc:  # dentro il cerchio
                        # Disegna tutti i quadranti del cerchio
                        for gx, gy in [
                            (cx + dx, cy + dy),  # Quadrante 1
                            (cx - dx, cy + dy),  # Quadrante 2
                            (cx + dx, cy - dy),  # Quadrante 3
                            (cx - dx, cy - dy),  # Quadrante 4
                        ]:
                            if 0 <= gx < w and 0 <= gy < h:
                                grid[gy, gx] = 100


        # ---- debug: quante celle sono state marcate a 100? ----
        occ_cells = int(np.count_nonzero(grid == 100))
        self.get_logger().info(f"Occupancy cells set to 100: {occ_cells}")

        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.resolution = res
        self.map.info.width      = w
        self.map.info.height     = h
        self.map.info.origin = Pose()
        self.map.info.origin.position.x = ox
        self.map.info.origin.position.y = oy
        self.map.info.origin.orientation.w = 1.0
        self.map.data = grid.flatten().tolist()

        self.pub   = self.create_publisher(OccupancyGrid, '/map', 10) # mantieni in coda al massimo 1 messaggio”: se ne arriva un altro prima che il subscriber lo legga, il più vecchio viene scartato.
        self.map_cb()
        self.timer = self.create_timer(10.0, self.map_cb) # pubblica ogni 1s la mappa

        self.pub_cmd   = self.create_subscription(Twist, '/rm0/cmd_vel', self._cmd_callback, 1)
        self.cmd_vel = None
        self.current_tower = 2 # starting tower
        self.flag = False


        # publisher to send goal pose at 10Hz
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.goal_timer = self.create_timer(2, self.publish_goal)
        self.go_again = True
        self.receive_goal_again = self.create_subscription(Bool, '/go_again', self.go_again_cb, 1)
        self.goal_reached = False
        self.sub_goal_reached = self.create_subscription(Bool, '/goal_reached', self._goal_reached_cb, 1)
    
    def _goal_reached_cb(self, msg):
        """Invoked by PathPlannerNode when the robot has reached the current tower."""
        if msg.data and not self.goal_reached:
            self.goal_reached = True
            self.go_again = False
            self.get_logger().info("Tower reached – handing control to ControllerNode")

    def go_again_cb(self, msg):
        """Invoked by ControllerNode after the tower has been inspected and shot."""
        if msg.data:
            # advance cyclically to the next tower
            self.current_tower = (self.current_tower + 1) % len(self.tower_goals)
            self.goal_reached = False
            self.go_again = True
            self.get_logger().info(f"Proceeding to tower #{self.current_tower}")
            # publish the new goal immediately
            self.publish_goal()

    def map_cb(self):
        # if not self.go_again:
        #     return
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map)
        self.get_logger().info(f'Published map')
        # publish the goal pose for the robot to reach
        self.publish_goal()
        

    def _cmd_callback(self, map):
        return


    def publish_goal(self):
        if not self.go_again:
            return
        goal = PoseStamped()
        x = self.tower_goals[self.current_tower][0]
        y = self.tower_goals[self.current_tower][1]
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Published goal pose: {goal.pose.position.x}, {goal.pose.position.y}')


def main():
    rclpy.init()
    rclpy.spin(MockMapPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()