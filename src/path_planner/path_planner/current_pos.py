import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sim
import time

class CoppeliaPosePublisher(Node):
    def __init__(self):
        super().__init__('coppelia_pose_pub')
        self.publisher = self.create_publisher(PoseStamped, '/current_pos', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)

        sim.simxFinish(-1)  # Terminate all open connections
        self.clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if self.clientID != -1:
            self.get_logger().info('Connesso a CoppeliaSim')
            _, self.robot_handle = sim.simxGetObjectHandle(self.clientID, 'robot', sim.simx_opmode_blocking)
            # Initialize streaming
            sim.simxGetObjectPosition(self.clientID, self.robot_handle, -1, sim.simx_opmode_streaming)
        else:
            self.get_logger().error('Connessione fallita con CoppeliaSim')

    def publish_pose(self):
        if self.clientID == -1:
            return
        _, position = sim.simxGetObjectPosition(self.clientID, self.robot_handle, -1, sim.simx_opmode_buffer)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.w = 1.0  # Assumiamo nessuna rotazione per semplicità
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = CoppeliaPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
