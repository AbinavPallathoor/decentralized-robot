import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (x, y, z, w) to euler roll, pitch, yaw.
    This replaces the need for the external tf_transformations library.
    """
    x, y, z, w = quaternion

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp))) # Clamped to prevent math domain errors

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
        
        # Clamp output to the physical limits of the robot
        return max(min(output, self.max_output), -self.max_output)

class HolonomicWaypointFollower(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')

        # 1. Setup TF to get the robot's current position on the map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2. Setup Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # We listen to Foxglove's default 2D Nav Goal topic. 
        # Later, we will change this to listen to /astar_rigid_path
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)

        # 3. PID Tuning Parameters
        # Slowed down maximum speeds for stability
        max_speed = 0.2      # max m/s (reduced from 0.5)
        max_rotation = 0.5   # max rad/s (reduced from 1.0)
        
        # Softened Kp to prevent aggressive overshooting and added Kd to brake near the target
        self.pid_x = PIDController(kp=0.8, ki=0.0, kd=0.2, max_output=max_speed)
        self.pid_y = PIDController(kp=0.8, ki=0.0, kd=0.2, max_output=max_speed)
        self.pid_yaw = PIDController(kp=1.0, ki=0.0, kd=0.1, max_output=max_rotation)

        # 4. State Variables
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        # Widened the stopping tolerance to 10cm so it reliably registers the stop
        self.xy_tolerance = 0.1    
        self.yaw_tolerance = 0.1   

        # 5. Control Loop (Runs at 20Hz)
        self.timer_period = 0.05 
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info("Holonomic PID Controller is ready. Waiting for a Nav Goal on /move_base_simple/goal...")

    def goal_callback(self, msg):
        """Triggered when you click '2D Nav Goal' in Foxglove."""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        
        # Convert quaternion to Yaw angle
        q = msg.pose.orientation
        _, _, self.target_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.get_logger().info(f"New Target Received: X={self.target_x:.2f}, Y={self.target_y:.2f}, Yaw={self.target_yaw:.2f}")

        # Reset PID integrals for a fresh movement
        self.pid_x.integral = 0.0
        self.pid_y.integral = 0.0
        self.pid_yaw.integral = 0.0

    def normalize_angle(self, angle):
        """Ensures the robot rotates the shortest distance (e.g., prevents doing a 270 deg spin instead of a 90 deg spin)."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        """The core math engine that runs 20 times a second."""
        if self.target_x is None:
            return # Do nothing if we don't have a goal yet

        # 1. Get the robot's current position from TF (map -> base_link)
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        except Exception as e:
            # Normal to fail occasionally if TF hasn't initialized
            return 

        # 2. Calculate Global Errors
        error_x_global = self.target_x - current_x
        error_y_global = self.target_y - current_y
        error_yaw = self.normalize_angle(self.target_yaw - current_yaw)

        # 3. Check if we reached the goal
        distance_to_goal = math.hypot(error_x_global, error_y_global)
        if distance_to_goal < self.xy_tolerance and abs(error_yaw) < self.yaw_tolerance:
            self.get_logger().info("Goal Reached!")
            self.stop_robot()
            self.target_x = None # Clear target
            return

        # 4. HOLONOMIC MATH: Rotate Global Error to Local (Robot) Frame
        # If the goal is North, but the robot is facing East, we need to tell it to strafe Left!
        error_x_local = (error_x_global * math.cos(current_yaw)) + (error_y_global * math.sin(current_yaw))
        error_y_local = (-error_x_global * math.sin(current_yaw)) + (error_y_global * math.cos(current_yaw))

        # 5. Compute PID Outputs based on the local errors
        cmd_vel = Twist()
        cmd_vel.linear.x = self.pid_x.compute(error_x_local, self.timer_period)
        cmd_vel.linear.y = self.pid_y.compute(error_y_local, self.timer_period)
        cmd_vel.angular.z = self.pid_yaw.compute(error_yaw, self.timer_period)

        # 6. Publish the motor commands
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
