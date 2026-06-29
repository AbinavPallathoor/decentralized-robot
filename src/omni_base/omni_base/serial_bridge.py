import rclpy
from rclpy.node import Node
import serial
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.ser = serial.Serial('/dev/ttyS0', 115200, timeout=0.1)

        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.reset_srv = self.create_service(Empty, 'reset_imu', self.reset_imu_callback)

        self.timer = self.create_timer(0.01, self.read_serial)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.ROBOT_RADIUS = 0.092376
        self.WHEEL_RADIUS = 0.03
        self.VELOCITY_CONVERSION = 0.00153398078

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        # Invert the 'vy' component for all three wheels
        s1_ms = -vy + (omega * self.ROBOT_RADIUS)
        s2_ms = (-0.866025 * vx) + (0.5 * vy) + (omega * self.ROBOT_RADIUS)
        s3_ms = (0.866025 * vx) + (0.5 * vy) + (omega * self.ROBOT_RADIUS)

        factor = 1.0 / (self.VELOCITY_CONVERSION * self.WHEEL_RADIUS)
        s1_raw = int(s1_ms * factor)
        s2_raw = int(s2_ms * factor)
        s3_raw = int(s3_ms * factor)

        command = f"{s1_raw},{s2_raw},{s3_raw}\n"
        self.ser.write(command.encode('utf-8'))

    def read_serial(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                parts = line.split(',')
                
                if len(parts) == 13:
                    vx, vy, omega, qx, qy, qz, qw, gx, gy, gz, ax, ay, az = map(float, parts)

                    if abs(vx) < 0.005: vx = 0.0
                    if abs(vy) < 0.005: vy = 0.0
                    if abs(omega) < 0.01: omega = 0.0

                    if abs(gx) < 0.01: gx = 0.0
                    if abs(gy) < 0.01: gy = 0.0
                    if abs(gz) < 0.01: gz = 0.0

                    current_time = self.get_clock().now()
                    dt = (current_time - self.last_time).nanoseconds / 1e9
                    self.last_time = current_time

                    self.th += omega * dt
                    self.x += (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
                    self.y += (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt

                    self.publish_odom(current_time, vx, vy, omega)
                    self.publish_imu(current_time, qx, qy, qz, qw, gx, gy, gz, ax, ay, az)
            except Exception as e:
                self.get_logger().error(f"Failed parsing: {e} | Line: {line}")

    def publish_odom(self, time, vx, vy, omega):
        odom = Odometry()
        odom.header.stamp = time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        self.odom_pub.publish(odom)

    def publish_imu(self, time, qx, qy, qz, qw, gx, gy, gz, ax, ay, az):
        imu = Imu()
        imu.header.stamp = time.to_msg()
        imu.header.frame_id = 'base_link'
        
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.01
        self.imu_pub.publish(imu)

    def reset_imu_callback(self, request, response):
        self.ser.write(b"RESET\n")
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.get_logger().info("IMU and Odometry reset triggered!")
        return response

def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
