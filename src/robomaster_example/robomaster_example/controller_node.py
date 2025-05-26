import rclpy, math
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import time

import sys

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Create client object
        self.sim_client = RemoteAPIClient()
        self.sim = self.sim_client.getObject('sim')

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which RoboMaster should be controlled.

        self.timer_period = 1/60

        # Internal state for trajectory control
        self.elapsed_time = 0.0
        self.direction = 1  # 1 for left circle, -1 for right circle
        
        # Define durations
        self.circle_radius = 0.8    # meters
        self.circle_duration = 10.0  # seconds to complete each half of "8"

        # Precompute velocities
        self.angular_velocity = (2 * math.pi) / self.circle_duration  # [rad/s]
        self.linear_velocity  = self.circle_radius * self.angular_velocity  # [m/s]
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(self.timer_period, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def shoot_projectile(self, position, velocity):
        # Create a small sphere (projectile)
        radius = 0.05
        mass = 1
        projectile_handle = self.sim.createPureShape(
            1,  # primitiveType: sphere
            8,  # options
            [radius, radius, radius],  # size
            mass,
            None
        )
        # Position the projectile
        self.sim.setObjectPosition(projectile_handle, -1, position)
        
        # Set initial velocity — simulate shooting
        self.sim.setObjectFloatParam(projectile_handle, self.sim.shapefloatparam_init_velocity_x, velocity[0])
        self.sim.setObjectFloatParam(projectile_handle, self.sim.shapefloatparam_init_velocity_y, velocity[1])
        self.sim.setObjectFloatParam(projectile_handle, self.sim.shapefloatparam_init_velocity_z, velocity[2])

        self.get_logger().info("Projectile spawned and fired.")
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
        
    def update_callback(self):
        # Update elapsed time
        self.elapsed_time += self.timer_period
        
        # # Check if it's time to switch direction (every half-circle)
        # if self.elapsed_time >= self.circle_duration:
        #     self.direction *= -1  # Reverse direction
        #     self.elapsed_time = 0.0  # Reset timer

        if self.elapsed_time > 3.0 and not hasattr(self, 'has_shot'):
            # robot_pos = self.pose3d_to_2d(self.odom_pose)
            position = [0.17, 0, 0.2]  # slightly above ground
            velocity = [3.0, 0.0, 3.0]  # shoot forward and upward
            self.shoot_projectile(position, velocity)
            self.has_shot = True
                
        # # Generate velocity command
        # cmd_vel = Twist()
        # cmd_vel.linear.x  = self.linear_velocity  # computed [m/s]
        # cmd_vel.angular.z = self.angular_velocity * self.direction  # [rad/s]
        
        # # Publish the command
        # self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the RoboMaster is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
