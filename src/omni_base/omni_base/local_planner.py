#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import math

def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')
        
        # --- LOCAL AVOIDANCE PARAMETERS ---
        self.danger_radius = 0.45  # Meters: Start pushing away if obstacles are closer than this
        self.repulsion_force = 1.0 # Max repulsive velocity
        self.sliding_force = 1.2   # Tangential force multiplier to slide around corners

        # FIX: this used to be a SECOND, lower speed cap (0.8) than the PID's own max_speed
        # (1.2 in holonomic_pid.py). Whichever node runs last/clamps tighter wins, and this
        # was silently capping your top speed below what the PID controller was allowed to
        # request. Raise it to match (or exceed slightly) the PID's max_output so this node
        # never becomes the bottleneck. Set this to whatever your robot's true physical
        # max translational speed is, in m/s.
        self.max_speed = 1.2
        
        # RPLidars mechanically have 0-degrees at their motor/wire side. 
        # If your robot is still driving INTO walls instead of away, change this to False.
        self.flip_lidar_orientation = True 
        
        # Smooths the repulsive force so it doesn't violently jerk the robot (Lower = smoother)
        self.smoothing_alpha = 0.15 

        self.num_danger_points = 8

        # --- IMPORTANT: this robot is HOLONOMIC and continuously yaws toward the FINAL
        # goal orientation while translating (see holonomic_pid.py: target_yaw is always
        # final_goal_yaw, applied from the first waypoint). That means base_link is rotating
        # in the world frame throughout normal driving, not just during avoidance.
        #
        # /cmd_vel_nav and /scan are both expressed in the ROBOT-LOCAL frame at the instant
        # they're produced. If we "commit" to a slide direction and hold it in robot-local
        # frame for ~2 seconds while the robot is yawing, that direction silently rotates
        # in the world frame too -- a slide that was correct at t=0 can point into a
        # different wall by t=2s. So: commit happens in WORLD frame, and gets re-projected
        # into the current robot-local frame every single cycle using TF.
        self.slide_commit_duration = 2.0
        self.slide_sign = 1.0
        self.slide_commit_until = 0.0
        # World-frame unit tangent direction we've committed to (set when a commit is made)
        self.committed_tangent_world = None  # (tx, ty) in map frame

        self.min_damp_factor = 0.35

        self.stall_speed_thresh = 0.05
        self.stall_time_thresh = 1.5
        self.stall_timer = 0.0
        self.escape_boost = 1.5
        self.last_cmd_time = None

        # TF so we can convert robot-local <-> world frame for the commit logic
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.nav_cmd_sub = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cmd_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.latest_scan = None
        self.danger_points = []  # list of (range, angle) inside danger_radius

        self.repulsive_vector_x = 0.0
        self.repulsive_vector_y = 0.0
        
        self.get_logger().info("Smoothed APF Local Planner (world-frame committed sliding) started!")

    def scan_callback(self, msg):
        self.latest_scan = msg
        
        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                if r < self.danger_radius:
                    pts.append((r, angle))
            angle += msg.angle_increment

        pts.sort(key=lambda p: p[0])
        self.danger_points = pts[: self.num_danger_points]

    def _compute_repulsion(self):
        """Average the repulsion contribution of all tracked danger points (robot-local frame)."""
        if not self.danger_points:
            return 0.0, 0.0, float('inf')

        sum_x = 0.0
        sum_y = 0.0
        min_r = float('inf')

        for r, ang in self.danger_points:
            magnitude = self.repulsion_force * (self.danger_radius - r) / self.danger_radius
            rx = -magnitude * math.cos(ang)
            ry = -magnitude * math.sin(ang)
            sum_x += rx
            sum_y += ry
            if r < min_r:
                min_r = r

        n = len(self.danger_points)
        rep_x = sum_x / n
        rep_y = sum_y / n

        if self.flip_lidar_orientation:
            rep_x = -rep_x
            rep_y = -rep_y

        return rep_x, rep_y, min_r

    def _get_current_yaw(self):
        """Robot's current yaw in the map frame, via TF. Returns None if unavailable."""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = trans.transform.rotation
            return euler_from_quaternion([q.x, q.y, q.z, q.w])
        except Exception:
            return None

    @staticmethod
    def _rotate(vx, vy, yaw):
        """Rotate a vector by yaw (robot-local -> world if yaw is robot's world heading)."""
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        wx = vx * cos_y - vy * sin_y
        wy = vx * sin_y + vy * cos_y
        return wx, wy

    @staticmethod
    def _unrotate(wx, wy, yaw):
        """Inverse of _rotate: world -> robot-local."""
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        vx = wx * cos_y + wy * sin_y
        vy = -wx * sin_y + wy * cos_y
        return vx, vy

    def nav_cmd_callback(self, msg):
        out_cmd = Twist()
        
        if self.latest_scan is None:
            self.cmd_pub.publish(msg)
            return

        nav_x = msg.linear.x
        nav_y = msg.linear.y

        now = self.get_clock().now().nanoseconds / 1e9
        current_yaw = self._get_current_yaw()

        rep_x, rep_y, closest_r = self._compute_repulsion()

        raw_force_x = 0.0
        raw_force_y = 0.0
        t1_x = t1_y = t2_x = t2_y = 0.0

        if closest_r < self.danger_radius:
            # Two perpendicular candidates to the (averaged) repulsion vector, in ROBOT-LOCAL frame
            t1_x, t1_y = -rep_y, rep_x
            t2_x, t2_y = rep_y, -rep_x

            nav_mag = math.hypot(msg.linear.x, msg.linear.y)

            if current_yaw is not None:
                # Convert current robot-local candidates to WORLD frame for stable comparison
                t1_world = self._rotate(t1_x, t1_y, current_yaw)
                t2_world = self._rotate(t2_x, t2_y, current_yaw)
                nav_world = self._rotate(msg.linear.x, msg.linear.y, current_yaw)

                need_new_commit = (now > self.slide_commit_until) or (self.committed_tangent_world is None)

                if need_new_commit and nav_mag > 0.05:
                    dot1 = t1_world[0] * nav_world[0] + t1_world[1] * nav_world[1]
                    dot2 = t2_world[0] * nav_world[0] + t2_world[1] * nav_world[1]
                    chosen_world = t1_world if dot1 >= dot2 else t2_world
                    self.committed_tangent_world = chosen_world
                    self.slide_commit_until = now + self.slide_commit_duration
                elif need_new_commit:
                    # nav is weak: keep prior committed world-frame direction if we have one,
                    # otherwise default to t1 (arbitrary, but consistent until next strong signal)
                    if self.committed_tangent_world is None:
                        self.committed_tangent_world = t1_world
                    self.slide_commit_until = now + self.slide_commit_duration

                # Re-project the WORLD-frame commitment into the CURRENT robot-local frame.
                # This is what stays correct even as the robot continuously yaws toward the
                # final goal orientation while translating.
                slide_local_x, slide_local_y = self._unrotate(
                    self.committed_tangent_world[0], self.committed_tangent_world[1], current_yaw
                )
            else:
                # No TF available this cycle: fall back to robot-local-only decision (old behavior)
                dot1 = (t1_x * msg.linear.x) + (t1_y * msg.linear.y)
                dot2 = (t2_x * msg.linear.x) + (t2_y * msg.linear.y)
                slide_local_x, slide_local_y = (t1_x, t1_y) if dot1 >= dot2 else (t2_x, t2_y)

            slide_x = slide_local_x * self.sliding_force
            slide_y = slide_local_y * self.sliding_force

            raw_force_x = rep_x + slide_x
            raw_force_y = rep_y + slide_y

            damp_factor = max(self.min_damp_factor, (closest_r / self.danger_radius))
            nav_x *= damp_factor
            nav_y *= damp_factor
        else:
            self.committed_tangent_world = None
            self.slide_commit_until = 0.0

        self.repulsive_vector_x = (1.0 - self.smoothing_alpha) * self.repulsive_vector_x + (self.smoothing_alpha * raw_force_x)
        self.repulsive_vector_y = (1.0 - self.smoothing_alpha) * self.repulsive_vector_y + (self.smoothing_alpha * raw_force_y)

        out_cmd.linear.x = nav_x + self.repulsive_vector_x
        out_cmd.linear.y = nav_y + self.repulsive_vector_y

        # --- Stall / deadlock escape (also re-projected from world frame) ---
        cmd_speed = math.hypot(out_cmd.linear.x, out_cmd.linear.y)
        if closest_r < self.danger_radius and cmd_speed < self.stall_speed_thresh:
            if self.last_cmd_time is not None:
                self.stall_timer += max(0.0, now - self.last_cmd_time)
            if self.stall_timer > self.stall_time_thresh and self.committed_tangent_world is not None and current_yaw is not None:
                escape_local_x, escape_local_y = self._unrotate(
                    self.committed_tangent_world[0], self.committed_tangent_world[1], current_yaw
                )
                out_cmd.linear.x += escape_local_x * self.escape_boost
                out_cmd.linear.y += escape_local_y * self.escape_boost
        else:
            self.stall_timer = 0.0

        self.last_cmd_time = now
        
        # Let the PID controller handle the rotation (yaw) un-interrupted
        out_cmd.angular.z = msg.angular.z
        
        # Clamp the final output to the physical limits of the robot
        out_cmd.linear.x = max(min(out_cmd.linear.x, self.max_speed), -self.max_speed)
        out_cmd.linear.y = max(min(out_cmd.linear.y, self.max_speed), -self.max_speed)
        
        self.cmd_pub.publish(out_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
