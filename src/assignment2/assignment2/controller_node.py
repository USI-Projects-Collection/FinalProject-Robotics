import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from sensor_msgs.msg import Image
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg import Bool
import array
import sys
import itertools
from std_msgs.msg import Bool


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.sim_client = RemoteAPIClient()
        self.sim = self.sim_client.getObject('sim')
        self.bridge = CvBridge()

        self.goal_reached = False
        self.sub_goal_reached = self.create_subscription(Bool, '/goal_reached', self._goal_reached_cb, 10)

        self.turn_ended = self.create_publisher(Bool, '/go_again', 10)


        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a publisher for gimbal control using ChannelFloat32 instead of GimbalCommand
        self.gimbal_publisher = self.create_publisher(ChannelFloat32, '/rm0/cmd_gimbal', 10)
        
        # Also create a publisher for gimbal engagement
        self.gimbal_engage_publisher = self.create_publisher(Bool, '/rm0/gimbal/engage', 10)
        
        # Create a subscriber to the topic 'odom', which will call
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Subscribe to the individual range sensors
        self.front_right_range_sub = self.create_subscription(Range, '/rm0/range_1', self.scan_range1_callback, 10)
        self.front_left_range_sub = self.create_subscription(Range, '/rm0/range_3', self.scan_range3_callback, 10)
        self.back_right_range_sub = self.create_subscription(Range, '/rm0/range_0', self.scan_range0_callback, 10)
        self.back_left_range_sub = self.create_subscription(Range, '/rm0/range_2', self.scan_range2_callback, 10)
        
        # Add attributes to store sensor readings
        self.range_0 = 10.0  # back right
        self.range_1 = 10.0  # front right
        self.range_2 = 10.0  # back left
        self.range_3 = 10.0  # front left
        
        self.last_camera_image = None
        
        # Add camera subscriber
        self.camera_subscriber = self.create_subscription(Image, '/rm0/camera/image_color', self.camera_callback, 10)
        
        self.state = "find_tower"  # Initialize robot state to find tower first
        self.look_around_start_yaw = None
        
        # Target distance from the tower
        self.target_distance = 2.0
        self.obstacle_target_distance = 0.2
        self.has_almost_avoided_obstacle = False
        
        # PID controller parameters for distance control
        self.kp_distance = 0.5
        self.ki_distance = 0.0
        self.kd_distance = 0.1
        self.distance_error_sum = 0.0
        self.last_distance_error = 0.0
        
        # For tracking the orbiting progress
        self.orbit_start_yaw = None
        self.orbit_completed = False
        self.full_rotation_angle = 2 * math.pi
        
        # For smoothing the transitions between sensors
        self.closest_distance = 10.0
        self.active_sensor = None

        # Gimbal control variables - using ChannelFloat32
        self.gimbal_rotated = False  # Track if the gimbal has been rotated
        self.alignment_start_time = None  # Track when we entered align_to_shoot state
        self.gimbal_rotation_started = False  # Track if we've started the gimbal rotation
        self.gimbal_rotation_complete = False  # Track if the gimbal rotation has completed
        self.gimbal_rotation_duration = 2.0  # Duration to apply rotation command in seconds

        # Define constants for ChannelFloat32 gimbal control
        self.GIMBAL_RATE_MODE = 1.0  # Assuming 1.0 represents RATE mode
        
        self.blaster_trans = np.array([0.20, 0, 0.2])
        self.seen_tower = False
        self.old_tower_width = 0
        self.min_tower_width = np.inf
        self.max_tower_width = 0
        self.is_growing_tower_width = None
        self.check_if_growing_tower_width = 0
        self.check_if_shrinking_tower_width = 0


    def scan_range0_callback(self, msg):
        self.range_0 = msg.range
        
    def scan_range1_callback(self, msg):
        self.range_1 = msg.range
        
    def scan_range2_callback(self, msg):
        self.range_2 = msg.range
        
    def scan_range3_callback(self, msg):
        self.range_3 = msg.range
        
    def camera_callback(self, msg):
        self.last_camera_image = msg
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)

    def _goal_reached_cb(self, msg):
        if msg.data:
            self.goal_reached = True
            self.get_logger().info("Goal reached! Starting controller behavior...")
        
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
        
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        pose2d = self.pose3d_to_2d(self.odom_pose)
        # self.get_logger().info(
        #     "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
        #     throttle_duration_sec=0.5  # Throttle logging frequency to max 2Hz
        # )
        
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
            yaw  # theta orientation
        )
        return pose2
        
    def normalize_angle(self, angle):
        """Normalize angle to be between [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def set_gimbal_angle(self, yaw_speed, pitch_speed):
        """
        Control gimbal by setting angular speeds in degrees/second using ChannelFloat32.
        
        Args:
            yaw_speed: Angular speed around yaw axis in degrees/second
            pitch_speed: Angular speed around pitch axis in degrees/second
        """
        # Create ChannelFloat32 message
        gimbal_cmd = ChannelFloat32()
        gimbal_cmd.name = "gimbal_control"
        
        # Structure: [mode, yaw_speed, pitch_speed]
        # Convert degrees/second to radians/second for consistency
        values = [
            self.GIMBAL_RATE_MODE,  # Mode (1.0 = RATE mode)
            float(math.radians(yaw_speed)),  # Yaw in radians/second
            float(math.radians(pitch_speed))  # Pitch in radians/second
        ]
        
        gimbal_cmd.values = array.array('f', values)
        
        # Publish the message
        self.gimbal_publisher.publish(gimbal_cmd)
        self.get_logger().info(f"Setting gimbal speeds - yaw: {yaw_speed} deg/s, pitch: {pitch_speed} deg/s")
        
    def engage_gimbal(self, engage=True):
        """Engage or disengage the gimbal motors."""
        engage_msg = Bool()
        engage_msg.data = engage
        self.gimbal_engage_publisher.publish(engage_msg)
        self.get_logger().info(f"{'Engaging' if engage else 'Disengaging'} gimbal motors")
    
    def check_tower_visibility(self, threshold):
        """Check if the tower is visible in the camera image."""
        if self.last_camera_image is None:
            return False
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.last_camera_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Camera image conversion failed: {e}")
            return False
            
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define red color range (tune this if your tower color differs)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2
        
        red_ratio = np.sum(mask > 0) / mask.size
        # self.get_logger().info(f"Red pixel ratio: {red_ratio:.4f}", throttle_duration_sec=1.0)
        
        return red_ratio > threshold  # Adjust threshold if needed
    
    def check_tower_in_view(self, color_row):
        is_color = False
        if color_row[0].any():
            return False
        for pixel in color_row:
            if pixel.any():
                is_color = True
            if is_color:
                if not pixel.any():
                    return True

        return False
    
    def get_maximum_width(self, masked_image):
        max_width = 0
        max_row = []
        for i in range(masked_image.shape[0]-1, -1, -1):
            row = masked_image[i]
            row_color = 0
            for pixel in row:
                if any(pixel):
                    row_color += 1
            
            if row_color >= max_width:
                max_width = row_color
                max_row = row
            else:
                break
        return max_row, max_width

    def get_tower_position_from_camera(self):
        """Extract the tower position from the camera image."""
        if self.last_camera_image is None:
            return None
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.last_camera_image, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([179, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 | mask2

            visualize_mask = cv2.bitwise_and(hsv, hsv, mask=mask)
            # Show only if visualize_mask is colored
            if visualize_mask.any():
                cv2.imshow("Mask", visualize_mask)
                cv2.waitKey(1)

            # METHOD 1
            # Analyze tower shape to detect if facing a complete side
            # Find contours of the tower
            # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # max_row, width_color = self.get_maximum_width(visualize_mask)
            # if contours:
            #     # Get the largest contour (the tower)
            #     largest_contour = max(contours, key=cv2.contourArea)
                
            #     # Calculate bounding rectangle
            #     x, y, w, h = cv2.boundingRect(largest_contour)
                
            #     # Calculate aspect ratio (width/height)
            #     aspect_ratio = w / h if h > 0 else 0

            #     # Check if robot is facing a complete side of the tower
            #     if aspect_ratio > 0.5:
            #         # Tower is facing the robot
            #         self.get_logger().info("Tower is facing the robot")
            #         self.state = "align_tower"
            
            # METHOD 2
            max_row, width_color = self.get_maximum_width(visualize_mask)
            if self.check_tower_in_view(max_row):
                if width_color < self.min_tower_width:
                    self.min_tower_width = width_color 
                np.set_printoptions(threshold=np.inf)
                # self.get_logger().info(f"{visualize_mask[0]}")
                for i in range(visualize_mask.shape[0]-1, -1, -1):
                    row = visualize_mask[i]
                    # self.get_logger().info(f"{row}")
                    row_color = 0
                    if any(itertools.chain(*row)):
                        for pixel in row:
                            if any(pixel):
                                row_color += 1
                        # self.get_logger().info(f'row color: {row_color}, width color: {width_color}')
                        # self.get_logger().info(f"row color: {row_color}, max width: {width_color}")
                        if row_color >= width_color-5 and row_color <= width_color+5 and width_color > self.min_tower_width + 20:
                            self.state = "align_tower"
                            self.get_logger().info(f"min width: {self.min_tower_width}, width current: {width_color}")
                            # You may want to return True here or set a flag
                            break
                        else:
                            break
                    else:
                        continue

            moments = cv2.moments(mask)
            if moments["m00"] > 0:
                cx = int(moments["m10"] / moments["m00"])
                width = mask.shape[1]
                # Normalize to range [-1, 1] where 0 is center
                tower_position = (cx - width // 2) / (width // 2)
                return tower_position
                
        except Exception as e:
            self.get_logger().warn(f"Camera processing error: {e}", throttle_duration_sec=1.0)
            
        return None
    
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
    
    def orbit_around_tower(self, cmd_vel, tower_position, measured_distance):
        if self.range_3 <= 0.2:
            self.get_logger().info("Obstacle detected in front - stopping orbit")
            self.state = "avoid_obstacle"
            return
        """Orbit around the tower using range_0 sensor and the camera."""
        # Calculate distance error using range_0
        distance_error = measured_distance - self.target_distance  # Positive error means robot is too far away

        # PID controller for distance regulation using range_0
        p_term = self.kp_distance * distance_error 
        self.distance_error_sum += distance_error 
        i_term = self.ki_distance * self.distance_error_sum
        d_term = self.kd_distance * (distance_error - self.last_distance_error)
        self.last_distance_error = distance_error
        
        # Calculate distance control output
        distance_control = p_term + i_term + d_term
        
        # Base velocities for left-side orbiting (counterclockwise around tower)
        base_linear_velocity = 0.15   # Forward motion
        base_angular_velocity = -0.4   # Positive angular velocity for counterclockwise rotation
        
        # Distance Control: Adjust angular velocity based on distance error from range_0
        # If too far (positive error), turn more sharply toward tower (reduce angular velocity)
        # If too close (negative error), turn away from tower (increase angular velocity)
        distance_angular_correction = -distance_control * 0.3
        
        # Camera Control: Adjust angular velocity to keep tower centered
        # tower_position: negative = tower on left side of image, positive = tower on right side
        # If tower is on right side (positive), reduce angular velocity to turn right toward tower
        # If tower is on left side (negative), increase angular velocity to turn left toward tower
        camera_angular_correction = -tower_position * 0.5
        
        # Combine both controls
        angular_velocity = base_angular_velocity + distance_angular_correction + camera_angular_correction
        
        # Linear velocity adjustment based on distance error
        linear_velocity = base_linear_velocity
        if distance_error > 0.2:  # Too far
            linear_velocity = base_linear_velocity * 0.8  # Slow down to allow closer approach
        elif distance_error < -0.2:  # Too close
            linear_velocity = base_linear_velocity * 1.2  # Speed up to move away
        
        # Apply velocity limits
        linear_velocity = max(0.05, min(0.3, linear_velocity))
        angular_velocity = min(-0.1, angular_velocity)
        
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity

        self.get_logger().info(
            f"Orbiting (left): range_0={measured_distance:.2f}m, " +
            f"dist_error={distance_error:.2f}m, tower_pos={tower_position:.2f}, " +
            f"lin_v={linear_velocity:.2f}, ang_v={angular_velocity:.2f}"
        )
        
    def update_callback(self):
        if not self.goal_reached:
            return
        if self.odom_pose is None:
            return
            
        cmd_vel = Twist()
        current_pose = self.pose3d_to_2d(self.odom_pose)
        
        # self.get_logger().info(f"State: {self.state}")
        
        if self.state == "find_tower":
            # Rotate to find the tower using range_1 sensor
            if self.range_1 <= self.target_distance + 0.1:
                # Tower found within target distance, switch to positioning for left orbit
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.state = "position_for_orbit"
                self.get_logger().info(f"Tower found at distance: {self.range_1:.2f}m, positioning for left orbit")
            else:
                # Continue rotating to find tower
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.5  # Rotate counterclockwise to search
                self.get_logger().info(f"Searching for tower... range_1: {self.range_1:.2f}m")
        
        elif self.state == "position_for_orbit":
            # Position the robot to start orbiting from the left side
            # Turn left (counterclockwise) to position the tower on the right side of the robot
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn left
            
            # Check if the tower is now detected by the right sensor (range_0)
            if self.range_0 <= self.target_distance + 0.5:  # Allow some tolerance for positioning
                # Tower is now on the right side, start orbiting
                self.orbit_start_yaw = current_pose[2]
                self.state = "orbit_tower"
                self.get_logger().info(f"Positioned for orbit. Tower distance on right: {self.range_0:.2f}m")
            else:
                self.get_logger().info(f"Positioning... range_0: {self.range_0:.2f}m, range_1: {self.range_1:.2f}m")

        elif self.state == "orbit_tower":
            # Avoid other towers during the orbit
            if self.range_1 <= self.obstacle_target_distance:
                self.get_logger().warn("Obstacle detected in front - stopping orbit")
                self.state = "avoid_obstacle"

            # Use range_0 (right sensor) to maintain distance during left-side orbit
            measured_distance = self.range_0

            # Get tower position from camera for centering control
            tower_position = self.get_tower_position_from_camera()

            if tower_position is not None:
                self.orbit_around_tower(cmd_vel, tower_position, measured_distance)
            else:
                # Tower lost from camera view - prioritize finding it again
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.3  # Rotate to find tower
                self.get_logger().warn("Tower lost from camera view during orbit - searching")
                if self.range_1 <= self.target_distance + 0.1:
                    # Tower found again, switch to positioning for left orbit
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.state = "position_for_orbit"
                    self.get_logger().info(f"Tower found at distance: {self.range_1:.2f}m, repositioning for left orbit")

        elif self.state == "avoid_obstacle":
            # Obstacle avoidance state: orbit around the obstacle while checking for original tower
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -0.5  # Turn right

            measured_distance_avoiding = self.range_2
            distance_error_sum = 0.0
            last_distance_error = 0.0
            
            if self.range_2 <= 0.2:
                if self.range_0 < 10:
                    self.has_almost_avoided_obstacle = True
                elif self.range_0 >= 10 and self.has_almost_avoided_obstacle:
                    self.get_logger().info("Obstacle avoided, resuming orbit")
                    self.state = "orbit_tower"
                    self.has_almost_avoided_obstacle = False
                # Calculate distance error using range_3
                distance_error = measured_distance_avoiding - self.obstacle_target_distance
                
                # PID controller for distance regulation using range_3
                p_term = 0.1 * distance_error
                distance_error_sum += distance_error
                i_term = 0.0 * distance_error_sum
                d_term = 0.1 * (distance_error - last_distance_error)
                last_distance_error = distance_error

                # Calculate distance control output
                distance_control = p_term + i_term + d_term
                
                # Base velocities for left-side orbiting (counterclockwise around tower)
                base_linear_velocity = 0.15   # Forward motion
                base_angular_velocity = 0.4   # Positive angular velocity for counterclockwise rotation
                
                # Distance Control: Adjust angular velocity based on distance error from range_0
                # If too far (positive error), turn more sharply toward tower (reduce angular velocity)
                # If too close (negative error), turn away from tower (increase angular velocity)
                distance_angular_correction = distance_control * 0.3

                # Combine both controls
                angular_velocity = base_angular_velocity + distance_angular_correction
                
                # Linear velocity adjustment based on distance error
                linear_velocity = base_linear_velocity
                if distance_error > 0.3:  # Too far
                    linear_velocity = base_linear_velocity * 0.8  # Slow down to allow closer approach
                elif distance_error < 0:  # Too close
                    linear_velocity = base_linear_velocity * 1.2  # Speed up to move away
                
                # Apply velocity limits
                linear_velocity = max(0.05, min(0.3, linear_velocity))
                angular_velocity = max(0.1, angular_velocity)
                
                cmd_vel.linear.x = linear_velocity
                cmd_vel.angular.z = angular_velocity

                self.get_logger().info(
                    f"Avoiding obstacle: range_3={measured_distance_avoiding:.2f}m, " +
                    f"dist_error={distance_error:.2f}m, " +
                    f"lin_v={linear_velocity:.2f}, ang_v={angular_velocity:.2f}"
                )
        
        elif self.state == "align_tower":
            self.get_logger().info(f"range right: {self.range_1}, range left: {self.range_3}")
            if not self.seen_tower:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = -0.3
                if self.range_1 != self.range_3:
                    self.seen_tower = True
            else:
                if self.range_1 != self.range_3:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = -0.3
                else:
                    self.state = 'shoot_tower'
        
        elif self.state == "shoot_tower":
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            robot_x, robot_y, yaw = self.pose3d_to_2d(self.odom_pose)
            rot_matrix = np.array([[np.cos(yaw), -np.sin(yaw),0],
                       [np.sin(yaw), np.cos(yaw),0],
                       [0,0,1]])
            new_trans = rot_matrix@self.blaster_trans
            projectile_pos = (robot_x + new_trans[0], robot_y + new_trans[1], new_trans[2])
            self.get_logger().info(f"robot pos: {robot_x, robot_y}, projectile pos: {projectile_pos}")
            velocity = rot_matrix@np.array([3.0, 0.0, 3.0])  # shoot forward and upward
            self.get_logger().info(f"{velocity}")

            self.shoot_projectile(projectile_pos, velocity)
            self.goal_reached = False
            self.turn_ended.publish(Bool(data=True))
            self.get_logger().info("Task completed!")

        elif self.state == "align_to_shoot":
            # Stop and prepare for shooting
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            
            # Make gimbal rotate
            if not self.gimbal_rotation_started:
                self.gimbal_rotation_started = True
                self.alignment_start_time = self.get_clock().now()
                self.engage_gimbal(True)
                self.set_gimbal_angle(90.0, 30.0)
                self.get_logger().info("Gimbal rotation started")
            elif self.gimbal_rotation_started and not self.gimbal_rotation_complete:
                elapsed_time = (self.get_clock().now() - self.alignment_start_time).nanoseconds / 1e9
                if elapsed_time >= self.gimbal_rotation_duration:
                    self.gimbal_rotation_complete = True
                    self.set_gimbal_angle(0.0, 0.0)
                    self.engage_gimbal(False)
                    self.state = "shoot_tower"
                    self.get_logger().info("Gimbal rotation completed")

        
        self.vel_publisher.publish(cmd_vel)
    
    def calculate_angle_traveled(self, start_angle, current_angle):
        """Calculate the angle traveled considering wrap-around."""
        # Normalize angles
        start_angle = self.normalize_angle(start_angle)
        current_angle = self.normalize_angle(current_angle)
        
        # Calculate difference
        diff = current_angle - start_angle
        
        # Handle wrap-around for counterclockwise motion
        if diff < 0:
            diff += 2 * math.pi
            
        return diff

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