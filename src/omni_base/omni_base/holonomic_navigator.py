#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

def euler_from_quaternion(q):
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class HolonomicNavigator(Node):
    def __init__(self):
        super().__init__('holonomic_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/astar_rigid_path', self.path_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.final_goal_yaw = 0.0
        self.latest_scan = None

        # --- Smooth Acceleration State ---
        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_vth = 0.0

        # --- PATH FOLLOWING TUNING ---
        self.max_speed = 0.25       
        self.max_yaw_rate = 0.40    
        self.yaw_kp = 2.0           
        
        self.lookahead_dist = 0.15  
        self.wp_tolerance = 0.15 
        
        # --- OBSTACLE AVOIDANCE TUNING ---
        self.danger_radius = 0.50   
        self.repulsion_force = 1.0  
        self.sliding_force = 0.80   # Increased to help 'wash' the robot around sharp corners
        self.emergency_stop_radius = 0.25 
        
        self.flip_lidar = True 

        self.accel_alpha = 0.50 

        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        self.get_logger().info("Highly Accurate Holonomic Navigator Started!")

    def path_cb(self, msg):
        self.waypoints = msg.poses
        if self.waypoints:
            q = self.waypoints[-1].pose.orientation
            _, _, self.final_goal_yaw = euler_from_quaternion(q)

    def scan_cb(self, msg):
        self.latest_scan = msg

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if not self.waypoints:
            # Smoothly decelerate to a stop if no path
            self.curr_vx *= (1.0 - self.accel_alpha)
            self.curr_vy *= (1.0 - self.accel_alpha)
            self.curr_vth *= (1.0 - self.accel_alpha)
            self.publish_velocities()
            return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            _, _, ryaw = euler_from_quaternion(trans.transform.rotation)
        except Exception:
            return

        # 1. Pre-compute closest obstacle to see if we are blocked
        min_r = float('inf')
        min_angle = 0.0
        if self.latest_scan is not None:
            angle = self.latest_scan.angle_min
            for r in self.latest_scan.ranges:
                if math.isfinite(r) and self.latest_scan.range_min < r < self.latest_scan.range_max:
                    if r < min_r:
                        min_r = r
                        min_angle = angle
                angle += self.latest_scan.angle_increment

        # 2. Waypoint Management (Drop Passed Waypoints)
        closest_idx = 0
        min_dist_to_path = float('inf')
        for i, wp in enumerate(self.waypoints):
            dist = math.hypot(wp.pose.position.x - rx, wp.pose.position.y - ry)
            if dist < min_dist_to_path:
                min_dist_to_path = dist
                closest_idx = i

        # Drop all waypoints behind us so the robot never turns back to an overshot point!
        self.waypoints = self.waypoints[closest_idx:]

        # --- NEW: DYNAMIC LOOKAHEAD EXPANSION ---
        # If fighting an obstacle, reach further down the path to "pull" the robot around it.
        # This completely solves the "corner trap" shown in your image!
        dynamic_wp_tolerance = self.wp_tolerance
        dynamic_lookahead = self.lookahead_dist
        
        if min_r < self.danger_radius:
            expansion = (self.danger_radius - min_r) * 2.0  # Scales from 0.0m to 0.5m extra reach
            dynamic_wp_tolerance = self.wp_tolerance + expansion
            dynamic_lookahead = self.lookahead_dist + expansion

        # Pop the target waypoint if we've arrived at it
        while self.waypoints:
            current_tolerance = self.wp_tolerance if len(self.waypoints) == 1 else dynamic_wp_tolerance
            dist = math.hypot(self.waypoints[0].pose.position.x - rx, self.waypoints[0].pose.position.y - ry)
            if dist < current_tolerance:
                self.waypoints.pop(0)
            else:
                break

        if not self.waypoints:
            self.get_logger().info("Destination Reached!")
            return

        # Find Lookahead Target using the dynamic distance
        target = self.waypoints[0]
        for wp in self.waypoints:
            if math.hypot(wp.pose.position.x - rx, wp.pose.position.y - ry) > dynamic_lookahead:
                target = wp
                break

        tx = target.pose.position.x
        ty = target.pose.position.y

        # 3. Calculate Global Intended Velocity
        dx = tx - rx
        dy = ty - ry
        path_dist = math.hypot(dx, dy)

        if path_dist > 0:
            gx = (dx / path_dist) * self.max_speed
            gy = (dy / path_dist) * self.max_speed
        else:
            gx, gy = 0.0, 0.0

        # Slow down smoothly as we approach the final destination waypoint
        dist_to_final = math.hypot(self.waypoints[-1].pose.position.x - rx, self.waypoints[-1].pose.position.y - ry)
        if dist_to_final < 0.5:
            damp = max(0.1, dist_to_final / 0.5)
            gx *= damp
            gy *= damp

        # 4. Calculate Obstacle Avoidance (in the GLOBAL Frame)
        rep_gx, rep_gy = 0.0, 0.0
        is_emergency = False
        
        if min_r < self.danger_radius:
            obs_global_angle = self.normalize_angle(ryaw + min_angle)
            
            # Direction pointing TOWARDS the obstacle
            obs_dir_x = math.cos(obs_global_angle)
            obs_dir_y = math.sin(obs_global_angle)

            if self.flip_lidar:
                obs_dir_x = -obs_dir_x
                obs_dir_y = -obs_dir_y
                
            # Repel AWAY from obstacle
            mag = self.repulsion_force * (self.danger_radius - min_r) / self.danger_radius
            rgx = -obs_dir_x * mag
            rgy = -obs_dir_y * mag

            # --- NEW: NORMALIZED TANGENTIAL SLIDE ---
            # Create vectors perpendicular to the obstacle (length 1.0)
            t1x, t1y = -obs_dir_y, obs_dir_x
            t2x, t2y = obs_dir_y, -obs_dir_x

            # Pick tangent closest to intended path
            if (t1x * gx + t1y * gy) > (t2x * gx + t2y * gy):
                slide_dir_x, slide_dir_y = t1x, t1y
            else:
                slide_dir_x, slide_dir_y = t2x, t2y
                
            # Scale the slide force based on how close the obstacle is
            slide_mag = self.sliding_force * (self.danger_radius - min_r) / self.danger_radius
            sgx = slide_dir_x * slide_mag
            sgy = slide_dir_y * slide_mag

            rep_gx = rgx + sgx
            rep_gy = rgy + sgy

            # --- NEW: LINEAR EMERGENCY BRAKING ---
            if min_r <= self.emergency_stop_radius:
                # EMERGENCY: Completely kill the A* forward momentum. Only push away.
                gx, gy = 0.0, 0.0
                is_emergency = True
                self.get_logger().warn(f"⚠️ EMERGENCY OVERRIDE! Obstacle at {min_r:.2f}m. Evading!")
            elif path_dist > 0:
                # SMART YIELDING: Smoothly ramp down speed to 0.0 as we approach the emergency radius
                norm_gx, norm_gy = gx / path_dist, gy / path_dist
                
                # If obstacle is generally in front of our intended path
                if (norm_gx * obs_dir_x + norm_gy * obs_dir_y) > 0.0:
                    # Math: 1.0 at danger_radius, smoothly transitions to 0.0 at emergency_stop_radius
                    damp = max(0.0, (min_r - self.emergency_stop_radius) / (self.danger_radius - self.emergency_stop_radius))
                    gx *= damp
                    gy *= damp

        # 5. Final Safe Global Vector
        safe_gx = gx + rep_gx
        safe_gy = gy + rep_gy

        # 6. Rotate Global Vector -> Local Robot Frame
        target_local_vx = safe_gx * math.cos(-ryaw) - safe_gy * math.sin(-ryaw)
        target_local_vy = safe_gx * math.sin(-ryaw) + safe_gy * math.cos(-ryaw)

        # 7. Target Yaw (Rotates to the goal orientation simultaneously)
        yaw_error = self.normalize_angle(self.final_goal_yaw - ryaw)
        target_vth = yaw_error * self.yaw_kp 

        # Clamp max limits
        target_local_vx = max(min(target_local_vx, self.max_speed), -self.max_speed)
        target_local_vy = max(min(target_local_vy, self.max_speed), -self.max_speed)
        target_vth = max(min(target_vth, self.max_yaw_rate), -self.max_yaw_rate)

        # 8. Smooth Acceleration (Low-Pass Filter)
        if is_emergency:
            # INSTANT BRAKE: Bypass the smoothing filter so we don't slide into the obstacle
            self.curr_vx = target_local_vx
            self.curr_vy = target_local_vy
            self.curr_vth = target_vth
        else:
            self.curr_vx = (1.0 - self.accel_alpha) * self.curr_vx + (self.accel_alpha * target_local_vx)
            self.curr_vy = (1.0 - self.accel_alpha) * self.curr_vy + (self.accel_alpha * target_local_vy)
            self.curr_vth = (1.0 - self.accel_alpha) * self.curr_vth + (self.accel_alpha * target_vth)

        self.publish_velocities()

    def publish_velocities(self):
        cmd = Twist()
        cmd.linear.x = float(self.curr_vx)
        cmd.linear.y = float(self.curr_vy)
        cmd.angular.z = float(self.curr_vth)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = HolonomicNavigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
