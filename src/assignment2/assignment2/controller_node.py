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
import sys
import itertools

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        # Create client object
        self.sim_client = RemoteAPIClient()
        self.sim = self.sim_client.getObject('sim')
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
        self.range_0 = 10.0  # back right
        self.range_1 = 10.0  # front right
        self.range_2 = 10.0  # back left
        self.range_3 = 10.0  # front left
        
        self.last_camera_image = None
        
        # Add camera subscriber
        self.camera_subscriber = self.create_subscription(Image, '/rm0/camera/image_color', self.camera_callback, 10)
        
        self.state = "approach_wall"  # Initialize robot state
        self.look_around_start_yaw = None
        
        # Target distance from the tower
        self.target_distance = 2.0
        
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
        
    def get_closest_sensor_to_tower(self):
        """Determine which sensor is closest to the tower and should be used for distance control."""
        # Get current orientation
        if self.odom_pose is None:
            return None, 10.0
            
        _, _, yaw = self.pose3d_to_2d(self.odom_pose)
        
        # Get all sensor readings
        sensors = {
            'range_0': self.range_0,  # back right
            'range_1': self.range_1,  # front right
            'range_2': self.range_2,  # back left
            'range_3': self.range_3   # front left
        }
        
        # Find the closest sensor
        closest_sensor = min(sensors, key=sensors.get)
        closest_distance = sensors[closest_sensor]
        
        # Check if the closest sensor is actually pointing towards the tower
        # This is a basic check and might need refinement based on your robot's geometry
        valid_sensors = []
        
        # If we have a tower in camera view, prioritize front sensors
        tower_visible = self.check_tower_visibility(0.01)
        
        if tower_visible:
            # If tower is visible, front sensors are more reliable
            if self.range_1 < 5.0:  # front right
                valid_sensors.append(('range_1', self.range_1))
            if self.range_3 < 5.0:  # front left
                valid_sensors.append(('range_3', self.range_3))
        
        # Add side sensors if they seem to detect something
        if self.range_0 < 5.0:  # back right
            valid_sensors.append(('range_0', self.range_0))
        if self.range_2 < 5.0:  # back left
            valid_sensors.append(('range_2', self.range_2))
            
        if not valid_sensors:
            return self.active_sensor, self.closest_distance
            
        # Use the closest valid sensor
        best_sensor, distance = min(valid_sensors, key=lambda x: x[1])
        
        # Apply some hysteresis to prevent rapid switching
        if self.active_sensor is None or distance < self.closest_distance - 0.3:
            self.active_sensor = best_sensor
            self.closest_distance = distance
            
        return self.active_sensor, self.closest_distance
    
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

            # Get row of color to check tower witdth
            # Step 1: Get middle row and compute tower width
            # middle_row = visualize_mask[visualize_mask.shape[0] // 2]

            # Count how many pixels are "active" (red or non-zero)
            # Assuming red mask, so non-zero means tower pixels
            max_row, width_color = self.get_maximum_width(visualize_mask)
            # for pixel in middle_row:
            #     if any(pixel):
            #         width_color += 1
            # width_color = np.count_nonzero(middle_row)
            # Proceed only if tower is in view
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
                            

            # # Analyze tower shape to detect if facing a complete side
            # # Find contours of the tower
            # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # if contours and self.check_tower_in_view(max_row):
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
            #         self.state = 'align_tower'
            # self.get_logger().info(f"Tower width: {self.min_tower_width}, {self.max_tower_width}, {self.is_growing_tower_width}")
            
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
        
    def update_callback(self):
        if self.odom_pose is None:
            return
            
        cmd_vel = Twist()
        current_pose = self.pose3d_to_2d(self.odom_pose)
        
        # self.get_logger().info(f"State: {self.state}")
        
        if self.state == "approach_wall":
            # Approach the tower until we're at approximately the target distance
            if min(self.range_1, self.range_3) <= self.target_distance + 0.1:
                cmd_vel.linear.x = 0.0
                self.orbit_start_yaw = current_pose[2]
                self.state = "orbit_tower"
                self.get_logger().info(f"Switching to orbit mode at distance: {min(self.range_1, self.range_3):.2f}m")
            else:
                # # Use camera to center the tower while approaching
                # tower_position = self.get_tower_position_from_camera()
                # if tower_position is not None:
                #     # Adjust orientation to keep tower centered while approaching
                #     angular_velocity = -0.5 * tower_position
                #     cmd_vel.angular.z = angular_velocity
                #     cmd_vel.linear.x = 0.2
                #     self.get_logger().info(f"Approaching tower: position={tower_position:.2f}, angular_vel={angular_velocity:.2f}")
                # else:
                    # Search for tower by rotating if not visible
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.3
                self.get_logger().info("Searching for tower")
        
        elif self.state == "orbit_tower":
            # Get the sensor closest to the tower
            active_sensor, measured_distance = self.get_closest_sensor_to_tower()
            
            # Get tower position from camera (for angular control)
            tower_position = self.get_tower_position_from_camera()
            
            if tower_position is not None:
                # Calculate distance error
                distance_error = measured_distance - self.target_distance # Positive error means the robot is too far away
                
                # ---- PID (Proportional-Integral-Derivative) controller for distance regulation ----
                # Calculates how much the robot should adjust its position (radially) to maintain the target 1-meter distance from the tower

                # Proportional Term -> adjustment proportional to the current error
                p_term = self.kp_distance * distance_error 
                # Accumulate error over time -> If the robot consistently stays slightly off target, this gradually increases correction
                self.distance_error_sum += distance_error 
                i_term = self.ki_distance * self.distance_error_sum
                # Derivative Term -> adjustment based on the rate of change of the error 
                d_term = self.kd_distance * (distance_error - self.last_distance_error)
                self.last_distance_error = distance_error
                
                radial_velocity = p_term + i_term + d_term
                
                # Limit radial velocity
                radial_velocity = max(-0.2, min(0.2, radial_velocity))
                
                # Always use forward motion for orbiting
                forward_velocity = 0.15  # Base forward velocity
                
                # Adjust forward velocity slightly based on distance (slow down if too close)
                if measured_distance < self.target_distance - 0.1:
                    forward_velocity = 0.1  # Slow down if too close
                elif measured_distance > self.target_distance + 0.1:
                    forward_velocity = 0.2  # Speed up if too far
                
                # Base angular velocity for orbiting
                angular_velocity = -0.4
                
                # Distance Control Through Steering
                distance_correction = 0.5 * distance_error  # Positive when too far, negative when too close
                
                # Adjust angular velocity based on tower position in camer
                # This helps keep the tower visible during orbiting
                # If the tower on the right side of the image (tower_position is positive), camera_correction is negative, reducing angular velocity and making the robot turn right
                camera_correction = -0.4 * tower_position

                # This makes the robot turn more sharply toward the tower when too far away
                # Conversely, when too close, the correction is negative, increasing angular velocity and making the robot turn more sharply away
                angular_velocity = angular_velocity - distance_correction + camera_correction
                
                # Convert to robot-centric velocities - always move forward
                cmd_vel.linear.x = forward_velocity
                cmd_vel.angular.z = angular_velocity
                
                # self.get_logger().info(
                #     f"Orbiting: sensor={active_sensor}, dist={measured_distance:.2f}m, " +
                #     f"error={distance_error:.2f}m, radial_v={radial_velocity:.2f}, " +
                #     f"angular_v={angular_velocity:.2f}, cam_pos={tower_position:.2f}"
                # )
                
                # Check if we've completed a full rotation
                current_yaw = current_pose[2]
                angle_traveled = self.calculate_angle_traveled(self.orbit_start_yaw, current_yaw)
                
                if angle_traveled >= self.full_rotation_angle and self.state != "done":
                    # self.get_logger().info("Full rotation completed!")
                    self.state = "done"
            else:
                # Tower lost from view - rotate to find it
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.3
                self.get_logger().warn("Tower lost from view - searching")
        elif self.state == 'align_tower':
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
                    
            # if abs(self.range_1 - self.range_3) > 0.5 or (self.range_1 == self.range_3 and):
            #     cmd_vel.linear.x = 0.0
            #     cmd_vel.angular.z = -0.3
            #     if self.range_1 != self.range_3:
            #         self.seen_tower = True
            # elif self.seen_tower == True and self.range_1 == self.range_3:
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
            self.get_logger().info("Task completed!")
        
        self.vel_publisher.publish(cmd_vel)
    
    def calculate_angle_traveled(self, start_angle, current_angle):
        """Calculate the angle traveled considering wrap-around."""
        # Normalize angles
        start_angle = self.normalize_angle(start_angle)
        current_angle = self.normalize_angle(current_angle)
        
        # Calculate difference
        diff = current_angle - start_angle
        
        # Handle wrap-around
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