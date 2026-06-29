#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import heapq
import math

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        # --- ROBOT PHYSICAL PARAMETERS ---
        # Keep the robot at least 25cm away from walls (Adjust this based on your robot's size!)
        self.inflation_radius = 0.20 
        
        # 1. Setup the special QoS profile required to receive saved maps
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 2. Subscribers and Publishers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/astar_rigid_path', 10)
        
        # 3. TF2 Setup for finding robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 4. Map variables
        self.map_data = None
        self.map_info = None
        
        self.get_logger().info("A* Planner Node Started. Waiting for /map and /move_base_simple/goal...")

    def map_callback(self, msg):
        """ Stores the map and inflates obstacles so the robot doesn't clip walls """
        self.get_logger().info("Map received! Inflating obstacles, please wait...")
        self.map_info = msg.info
        
        # Convert tuple to a list so we can modify it
        inflated_data = list(msg.data)
        
        # Calculate how many map grid cells equal our inflation radius
        cell_radius = int(math.ceil(self.inflation_radius / self.map_info.resolution))
        width = self.map_info.width
        height = self.map_info.height
        
        # Find all original obstacles on the map
        obstacle_indices = [i for i, val in enumerate(msg.data) if val > 50]
        
        # Add padding cells around every obstacle
        for idx in obstacle_indices:
            my = idx // width
            mx = idx % width
            
            for dy in range(-cell_radius, cell_radius + 1):
                for dx in range(-cell_radius, cell_radius + 1):
                    # Use hypotenuse to create circular padding, not a square
                    if math.hypot(dx, dy) <= cell_radius:
                        nx, ny = mx + dx, my + dy
                        
                        # Check bounds
                        if 0 <= nx < width and 0 <= ny < height:
                            infl_idx = ny * width + nx
                            inflated_data[infl_idx] = 100  # Mark as obstacle
                            
        self.map_data = inflated_data
        self.get_logger().info(f"Obstacles inflated by {self.inflation_radius}m. Ready to plan!", once=True)

    def world_to_grid(self, x, y):
        """ Converts real-world meters to map array indices """
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return mx, my

    def grid_to_world(self, mx, my):
        """ Converts map array indices back to real-world meters """
        wx = (mx + 0.5) * self.map_info.resolution + self.map_info.origin.position.x
        wy = (my + 0.5) * self.map_info.resolution + self.map_info.origin.position.y
        return wx, wy

    def is_valid(self, mx, my):
        """ Checks if a grid cell is within bounds and free of inflated obstacles """
        if mx < 0 or my < 0 or mx >= self.map_info.width or my >= self.map_info.height:
            return False
        
        index = my * self.map_info.width + mx
        
        # We now check against the artificially fattened map data
        val = self.map_data[index]
        if val > 50:
            return False
        return True

    def heuristic(self, a, b):
        """ Euclidean distance heuristic for A* """
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def astar(self, start, goal):
        """ The A* Pathfinding Algorithm """
        frontier = []
        heapq.heappush(frontier, (0, start))
        
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        # 8-way movement for holonomic robot
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), 
                     (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while frontier:
            _, current = heapq.heappop(frontier)
            
            if current == goal:
                break
                
            for dx, dy in neighbors:
                next_node = (current[0] + dx, current[1] + dy)
                
                if not self.is_valid(next_node[0], next_node[1]):
                    continue
                
                # Diagonal movement costs slightly more (sqrt(2))
                move_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                new_cost = cost_so_far[current] + move_cost
                
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current
                    
        # Reconstruct path
        if goal not in came_from:
            return [] # No path found
            
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        
        return path

    def goal_callback(self, msg):
        """ Triggered when a new goal is set in Foxglove """
        if self.map_data is None:
            self.get_logger().warn("Cannot plan path: No map received yet.")
            return
            
        try:
            # Look up robot's current position (map to base_link)
            transform = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time())
            
            start_x = transform.transform.translation.x
            start_y = transform.transform.translation.y
            
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            
            self.get_logger().info(f"Planning path from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})")
            
            # Convert to grid indices
            start_grid = self.world_to_grid(start_x, start_y)
            goal_grid = self.world_to_grid(goal_x, goal_y)
            
            # Run A*
            grid_path = self.astar(start_grid, goal_grid)
            
            if not grid_path:
                self.get_logger().error("A* could not find a valid path to the goal! (Goal might be inside an inflated wall)")
                return
                
            # Convert grid path back to world coordinates and publish
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            
            # Loop through the A* points to build the path message
            for i, (mx, my) in enumerate(grid_path):
                pose = PoseStamped()
                pose.header = path_msg.header
                wx, wy = self.grid_to_world(mx, my)
                pose.pose.position.x = float(wx)
                pose.pose.position.y = float(wy)
                pose.pose.position.z = 0.0
                
                # IMPORTANT: If this is the absolute final waypoint, give it the exact 
                # orientation requested from the Foxglove click!
                if i == len(grid_path) - 1:
                    pose.pose.orientation = msg.pose.orientation
                else:
                    pose.pose.orientation.w = 1.0 # Intermediate points stay flat
                    
                path_msg.poses.append(pose)
                
            self.path_pub.publish(path_msg)
            self.get_logger().info("Path published successfully!")
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Could not find robot position: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
