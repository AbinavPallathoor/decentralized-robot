#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import cv2
import numpy as np
import math

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.inflation_radius = 0.20
        self.blacklist = []
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_msg = None
        self.current_goal = None
        self.goal_time = 0.0
        
        self.timer = self.create_timer(2.0, self.exploration_loop)
        self.get_logger().info("Utility-Based Frontier Explorer started!")

    def map_cb(self, msg):
        self.map_msg = msg

    def exploration_loop(self):
        if self.map_msg is None: return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
        except Exception: return

        if self.current_goal is not None:
            dist = math.hypot(robot_x - self.current_goal[0], robot_y - self.current_goal[1])
            time_elapsed = self.get_clock().now().nanoseconds / 1e9 - self.goal_time
            if dist > 0.5:
                if time_elapsed < 12.0: return
                else:
                    self.get_logger().warn(f"Goal Timed Out! Blacklisting X:{self.current_goal[0]:.2f}, Y:{self.current_goal[1]:.2f}")
                    self.blacklist.append(self.current_goal)
                    self.current_goal = None

        best_frontier = self.find_best_frontier(robot_x, robot_y)

        if best_frontier is None:
            self.get_logger().info("🗺️ MAPPING COMPLETE! No frontiers left.")
            self.timer.cancel()
            return

        self.publish_goal(best_frontier[0], best_frontier[1], robot_x, robot_y)
        self.current_goal = best_frontier
        self.goal_time = self.get_clock().now().nanoseconds / 1e9

    def find_best_frontier(self, robot_x, robot_y):
        width, height = self.map_msg.info.width, self.map_msg.info.height
        grid = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))

        free_space = np.uint8((grid >= 0) & (grid <= 50)) * 255
        unknown_space = np.uint8(grid == -1) * 255
        obstacle_space = np.uint8(grid > 50) * 255

        # Noise filtering: Remove tiny speckles in the free space
        kernel_small = np.ones((3,3), np.uint8)
        free_space = cv2.morphologyEx(free_space, cv2.MORPH_OPEN, kernel_small)

        cell_radius = int(math.ceil(self.inflation_radius / self.map_msg.info.resolution))
        if cell_radius > 0:
            y, x = np.ogrid[-cell_radius:cell_radius+1, -cell_radius:cell_radius+1]
            circular_kernel = np.uint8(x**2 + y**2 <= cell_radius**2)
            inflated_obstacles = cv2.dilate(obstacle_space, circular_kernel, iterations=1)
        else: inflated_obstacles = obstacle_space

        safe_free_space = cv2.bitwise_and(free_space, cv2.bitwise_not(inflated_obstacles))
        
        dilate_radius = cell_radius + 3
        y_unk, x_unk = np.ogrid[-dilate_radius:dilate_radius+1, -dilate_radius:dilate_radius+1]
        unk_kernel = np.uint8(x_unk**2 + y_unk**2 <= dilate_radius**2)
        unknown_dilated = cv2.dilate(unknown_space, unk_kernel, iterations=1)

        frontier_mask = cv2.bitwise_and(safe_free_space, unknown_dilated)
        contours, _ = cv2.findContours(frontier_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        best_frontier = None
        max_utility_score = -float('inf')

        for contour in contours:
            if len(contour) < 10: continue

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            else:
                cx, cy = contour[0][0][0], contour[0][0][1]

            wx = (cx + 0.5) * self.map_msg.info.resolution + self.map_msg.info.origin.position.x
            wy = (cy + 0.5) * self.map_msg.info.resolution + self.map_msg.info.origin.position.y

            is_blacklisted = any(math.hypot(wx - bx, wy - by) < 1.0 for bx, by in self.blacklist)
            if is_blacklisted: continue

            dist = math.hypot(wx - robot_x, wy - robot_y)
            if dist < 0.4: continue
            
            # UTILITY SCORING: Prioritize massive unknown areas over slightly closer tiny areas
            utility_score = (len(contour) * 1.5) / (dist + 0.1)

            if utility_score > max_utility_score:
                max_utility_score = utility_score
                best_frontier = (wx, wy)

        return best_frontier

    def publish_goal(self, target_x, target_y, robot_x, robot_y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y = float(target_x), float(target_y)
        
        target_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
        q = quaternion_from_euler(0, 0, target_yaw)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q[0], q[1], q[2], q[3]

        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try: rclpy.spin(FrontierExplorer())
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
