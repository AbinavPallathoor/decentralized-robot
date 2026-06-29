import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener

def euler_from_quaternion(quaternion):
    """ Converts quaternion (x, y, z, w) to euler roll, pitch, yaw. """
    x, y, z, w = quaternion
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class PIDController:
    """A simple PID controller class."""
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(output, self.max_output), -self.max_output)

class HolonomicWaypointFollower(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')

        # 1. Setup TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2. Setup Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to the A* Path instead of a single goal
        self.path_sub = self.create_subscription(Path, '/astar_rigid_path', self.path_callback, 10)

        # 3. PID Tuning Parameters
        max_speed = 0.8     
        max_rotation = 0.8   
        self.pid_x = PIDController(kp=0.8, ki=0.0, kd=0.2, max_output=max_speed)
        self.pid_y = PIDController(kp=0.8, ki=0.0, kd=0.2, max_output=max_speed)
        self.pid_yaw = PIDController(kp=1.0, ki=0.0, kd=0.1, max_output=max_rotation)

        # 4. Waypoint State Variables
        self.waypoints = []
        self.current_wp_index = 0
        self.target_yaw = 0.0
        
        # Smooth cornering: Target the next point when we get within 20cm of the current one
        self.wp_tolerance = 0.20   
        # Final stop: Stop exactly on the final destination dot
        self.final_tolerance = 0.05 

        # 5. Control Loop
        self.timer_period = 0.05 
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info("Holonomic Path Follower ready. Waiting for /astar_rigid_path...")

    def path_callback(self, msg):
        """Triggered when A* publishes a new path."""
        self.waypoints = msg.poses
        self.current_wp_index = 0
        
        if self.waypoints:
            self.get_logger().info(f"Received new path with {len(self.waypoints)} waypoints!")
            
        # Reset PID integrals for fresh movement
        self.pid_x.integral = 0.0
        self.pid_y.integral = 0.0
        self.pid_yaw.integral = 0.0

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        # 1. Do nothing if we have no path
        if not self.waypoints or self.current_wp_index >= len(self.waypoints):
            return 

        # 2. Get current robot position
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        except Exception:
            return 

        # 3. Get the current active waypoint
        target_pose = self.waypoints[self.current_wp_index]
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y

        # 4. Calculate Global Errors to the waypoint
        error_x_global = target_x - current_x
        error_y_global = target_y - current_y
        distance_to_wp = math.hypot(error_x_global, error_y_global)

        # 5. Waypoint Management (Check if we arrived)
        is_last_wp = (self.current_wp_index == len(self.waypoints) - 1)

        if is_last_wp:
            # If it's the final point, enforce a strict stop
            if distance_to_wp < self.final_tolerance:
                self.get_logger().info("Destination Reached!")
                self.stop_robot()
                self.waypoints = []
                return
        else:
            # If it's a middle point, cut the corner and target the next one
            if distance_to_wp < self.wp_tolerance:
                self.current_wp_index += 1
                return # Skip to the next loop iteration

        # 6. Yaw Logic: Rotate to face the direction of the path
        if distance_to_wp > 0.05: # Prevent erratic spinning when microscopic to a point
            self.target_yaw = math.atan2(error_y_global, error_x_global)
            
        error_yaw = self.normalize_angle(self.target_yaw - current_yaw)

        # 7. HOLONOMIC MATH: Rotate Global Error to Local (Robot) Frame
        error_x_local = (error_x_global * math.cos(current_yaw)) + (error_y_global * math.sin(current_yaw))
        error_y_local = (-error_x_global * math.sin(current_yaw)) + (error_y_global * math.cos(current_yaw))

        # 8. Compute PID Outputs
        cmd_vel = Twist()
        cmd_vel.linear.x = self.pid_x.compute(error_x_local, self.timer_period)
        cmd_vel.linear.y = self.pid_y.compute(error_y_local, self.timer_period)
        cmd_vel.angular.z = self.pid_yaw.compute(error_yaw, self.timer_period)

        self.cmd_pub.publish(cmd_vel)

    def stop_robot(self):
        cmd_vel = Twist()
        self.cmd_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = HolonomicWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
