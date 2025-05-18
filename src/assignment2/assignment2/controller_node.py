import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion
import math

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from sensor_msgs.msg import Image

import sys

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.bridge = CvBridge()
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Subscribe to the individual range sensors
        self.front_right_range_sub = self.create_subscription(Range, '/rm0/range_1', self.scan_range1_callback, 10)
        self.front_left_range_sub = self.create_subscription(Range, '/rm0/range_3', self.scan_range3_callback, 10)
        self.back_right_range_sub = self.create_subscription(Range, '/rm0/range_0', self.scan_range0_callback, 10)
        self.back_left_range_sub = self.create_subscription(Range, '/rm0/range_2', self.scan_range2_callback, 10)
        
        # Add attributes to store sensor readings
        self.range_0 = 10.0
        self.range_1 = 10.0
        self.range_2 = 10.0
        self.range_3 = 10.0

        self.last_camera_image = None
        # Add camera subscriber
        self.camera_subscriber = self.create_subscription(Image, '/rm0/camera/image_color', self.camera_callback, 10)

        self.state = "approach_wall"  # Initialize robot state
        self.look_around_start_yaw = None

    def scan_range0_callback(self, msg):
        self.range_0 = msg.range

    def scan_range1_callback(self, msg):
        self.range_1 = msg.range

    def scan_range2_callback(self, msg):
        self.range_2 = msg.range

    def scan_range3_callback(self, msg):
        self.range_3 = msg.range
        
    def camera_callback(self, msg):
        # Placeholder: You can process the image here to detect the tower.
        self.last_camera_image = msg
        self.get_logger().info("Camera image received", throttle_duration_sec=1.0)
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
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

    def normalize_angle(self, angle):
        """Normalize angle to be between [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def check_tower_visibility(self):
        if self.last_camera_image is None:
            return False

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.last_camera_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Camera image conversion failed: {e}")
            return False

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        avg_color = np.mean(hsv.reshape(-1, 3), axis=0)
        self.get_logger().info(f"Average HSV seen: H={avg_color[0]:.1f}, S={avg_color[1]:.1f}, V={avg_color[2]:.1f}", throttle_duration_sec=1.0)

        # Define red color range (tune this if your tower color differs)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        red_ratio = np.sum(mask > 0) / mask.size
        self.get_logger().info(f"Red pixel ratio: {red_ratio:.4f}", throttle_duration_sec=1.0)

        return red_ratio > 0.01  # Adjust threshold if needed

    def update_callback(self):
        cmd_vel = Twist()
        self.get_logger().info(f"State: {self.state}")
        
        if self.state == "approach_wall":
            target_distance = 1.0
            if self.range_1 <= target_distance or self.range_3 <= target_distance:
                cmd_vel.linear.x = 0.0
                if self.look_around_start_yaw is None:
                    self.look_around_start_yaw = self.pose3d_to_2d(self.odom_pose)[2]
                self.state = "rotate_around_wall"
            else:
                cmd_vel.linear.x = 0.2
        
        elif self.state == "rotate_around_wall":
            current_yaw = self.pose3d_to_2d(self.odom_pose)[2]
            yaw_error = self.normalize_angle((self.look_around_start_yaw + math.pi / 2) - current_yaw)
            self.get_logger().info(f"Current yaw: {current_yaw:.2f}, Yaw error: {yaw_error:.2f}", throttle_duration_sec=1.0)

            if self.look_around_start_yaw is not None:
                if abs(yaw_error) < 1.58:
                    self.get_logger().info(f"Yaw error: {yaw_error:.2f}")
                    # Turn left until 90 degrees is reached
                    cmd_vel.angular.z = 0.3
                    cmd_vel.linear.x = 0.0
                else:
                    # After 90 degrees, check for tower
                    self.get_logger().info("Reached target yaw. Checking for tower...", throttle_duration_sec=1.0)
                    if self.check_tower_visibility():
                        self.get_logger().info("Tower detected by camera!", throttle_duration_sec=1.0)
                    else:
                        self.get_logger().warn("Tower NOT detected!", throttle_duration_sec=1.0)
                    self.state = "done"
            else:
                self.get_logger().warn("look_around_start_yaw is not set.", throttle_duration_sec=1.0)
        
        elif self.state == "done":
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        
        self.vel_publisher.publish(cmd_vel)


def main():
    rclpy.init(args=sys.argv)
    node = ControllerNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()


if __name__ == '__main__':
    main()