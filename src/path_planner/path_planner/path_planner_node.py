#!/usr/bin/env python3
import rclpy, numpy as np, heapq
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        self.grid = None
        self.res  = 0.05
        self.origin = (0, 0)

        self.sub_map   = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.sub_goal  = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.sub_odom  = self.create_subscription(PoseStamped, '/start_pose', self.pose_cb, 10)  # usa odom reale più avanti

        self.pub_path  = self.create_publisher(Path, '/plan', 10)
        self.pub_cmd   = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_pose = None

    # ---------- callback ----------
    def map_cb(self, msg):
        self.res     = msg.info.resolution
        self.origin  = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.grid    = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.get_logger().info('Mappa ricevuta')

    def pose_cb(self, msg):
        self.current_pose = msg

    def goal_cb(self, goal):
        if self.grid is None or self.current_pose is None:
            self.get_logger().warn('Mappa o posa non pronta')
            return
        start = self.world2grid(self.current_pose.pose.position)
        g     = self.world2grid(goal.pose.position)
        path  = self.astar(start, g)
        self.publish_path(path, goal.header.frame_id)

    # ---------- util ----------
    def world2grid(self, pos):
        gx = int((pos.x - self.origin[0]) / self.res)
        gy = int((pos.y - self.origin[1]) / self.res)
        return (gx, gy)

    def grid2world(self, cell):
        x = cell[0] * self.res + self.origin[0] + self.res / 2
        y = cell[1] * self.res + self.origin[1] + self.res / 2
        return (x, y)

    # ---------- A* ----------
    def astar(self, start, goal):
        h = lambda n: abs(n[0]-goal[0]) + abs(n[1]-goal[1])   # manhattan
        open_set = [(h(start), 0, start, None)]
        came = {}
        cost = {start: 0}
        dirs = [(1,0),(-1,0),(0,1),(0,-1)]

        while open_set:
            _, g_cost, cur, parent = heapq.heappop(open_set)
            if cur in came:
                continue
            came[cur] = parent
            if cur == goal:
                break
            for dx,dy in dirs:
                nxt = (cur[0]+dx, cur[1]+dy)
                if not (0 <= nxt[0] < self.grid.shape[1] and 0 <= nxt[1] < self.grid.shape[0]):
                    continue
                if self.grid[nxt[1], nxt[0]] >= 50:           # occupato
                    continue
                newc = g_cost + 1
                if newc < cost.get(nxt, 1e9):
                    cost[nxt] = newc
                    heapq.heappush(open_set, (newc + h(nxt), newc, nxt, cur))
        # ricostruisci
        path = []
        n = goal
        while n:
            path.append(n)
            n = came.get(n)
        return path[::-1]

    def publish_path(self, cells, frame_id):
        msg = Path()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        for c in cells:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = self.grid2world(c)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

        self.pub_path.publish(msg)
        self.get_logger().info(f'Path con {len(cells)} punti pubblicato')

def main():
    rclpy.init()
    rclpy.spin(AStarPlanner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()