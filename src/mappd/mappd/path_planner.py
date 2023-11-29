import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np

class PathPlanner(Node):
    reservation_table = np.zeros((5, 5, 5), dtype=np.uint8)

    def __init__(self, drone_id):
        super().__init__('mapp_drone_'+str(drone_id))
        self.drone_id = drone_id
        self.map = None
        self.goals = {}
        self.obstacles = []
        self.resolution = 0.5
        self.grid_size = 5
        self.environment = np.zeros((self.grid_size, self.grid_size, self.grid_size), dtype=np.uint8)

        # ROS publishers and subscribers
        self.goal_publisher = self.create_publisher(PoseStamped, f"/drone{drone_id}/goal", 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.obstacle_subscriber = self.create_subscription(PoseStamped, "/obstacles", self.obstacle_callback, 10)
        self.reservation_subscriber = self.create_subscription(String, "/reservations", self.reservation_callback, 10)

    def map_callback(self, msg):
        self.map = msg
        print(msg)
        # Configure environment based on map data
        self.configure_environment()

    def configure_environment(self):
        # Extract map information
        map_width = self.map.info.width
        map_height = self.map.info.height
        map_data = self.map.data

        # Convert map data to environment grid
        for y in range(map_height):
            for x in range(map_width):
                map_index = y * map_width + x
                environment_index = (x, y, 0)
                if map_data[map_index] == 0:
                    self.environment[environment_index] = 0
                else:
                    self.environment[environment_index] = 1
        rclpy.spin_once(self)

    def obstacle_callback(self, msg):
        self.obstacles.append(msg)

    def reservation_callback(self, msg):
        self.reserved_paths.append(msg.data)

    def add_goal(self, goal_id, position):
        self.goals[goal_id] = position

    def compute_path(self, start, goal):
        start_voxel = self.position_to_voxel(start)
        goal_voxel = self.position_to_voxel(goal)
        print(start_voxel)
        print(goal_voxel)
        print(self.environment)
        # A* Diagonal Path Planning Algorithm
        open_set = [start_voxel]
        came_from = {}
        g_score = {start_voxel: 0}
        f_score = {start_voxel: self.heuristic(start_voxel, goal_voxel)}

        while open_set:
            current = min(f_score, key=f_score.get)

            if current == goal_voxel:
                path = self.reconstruct_path(came_from, current)
                return path

            open_set.remove(current)
            del f_score[current]

            neighbors = self.get_neighbors(current)
            for neighbor in neighbors:
                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal_voxel)

                    if neighbor not in open_set:
                        open_set.append(neighbor)

        return None

    def position_to_voxel(self, position):
        x = int(position[0] / self.resolution)
        y = int(position[1] / self.resolution)
        z = int(position[2] / self.resolution)
        return (x, y, z)

    def voxel_to_position(self, voxel):
        x = voxel[0] * self.resolution + self.resolution / 2.0
        y = voxel[1] * self.resolution + self.resolution / 2.0
        z = voxel[2] * self.resolution + self.resolution / 2.0
        return (x, y, z)

    def distance(self, voxel1, voxel2):
        return np.linalg.norm(np.array(voxel1) - np.array(voxel2))

    def heuristic(self, voxel1, voxel2):
        return np.linalg.norm(np.array(voxel1) - np.array(voxel2), ord=1)

    def get_neighbors(self, voxel):
        neighbors = []
        for dz in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    new_voxel = (voxel[0] + dx, voxel[1] + dy, voxel[2] + dz)
                    if self.is_valid_voxel(new_voxel):
                        neighbors.append(new_voxel)
        return neighbors
    def is_valid_voxel(self, voxel):
        if (
            0 <= voxel[0] < self.grid_size and
            0 <= voxel[1] < self.grid_size and
            0 <= voxel[2] < self.grid_size and
            self.environment[voxel] == 0 and
            self.reservation_table[voxel[0], voxel[1], voxel[2]] == 0  # Updated line
        ):
            return True
        return False

    def reconstruct_path(self, came_from, current):
        path = [self.voxel_to_position(current)]
        while current in came_from:
            current = came_from[current]
            path.append(self.voxel_to_position(current))
        path.reverse()
        return path

    def reserve_path(self, path):
        for voxel in path:
            self.reservation_table[voxel] = self.drone_id

    def unreserved(self, path):
        for voxel in path:
            self.reservation_table[voxel] = 0

    def wait_at_previous_pose(self, goal):
        goal_voxel = self.position_to_voxel(goal)
        while (
            goal_voxel[0] >= 0 and goal_voxel[0] < self.grid_size and
            goal_voxel[1] >= 0 and goal_voxel[1] < self.grid_size and
            goal_voxel[2] >= 0 and goal_voxel[2] < self.grid_size and
            self.reservation_table[goal_voxel[0], goal_voxel[1], goal_voxel[2]] != 0
        ):
            pass


    def plan_path(self):
        start = (0.5, 0.5, 0.5)  # Replace with actual start position
        goal = self.goals[self.drone_id]
        self.wait_at_previous_pose(goal)
        path = self.compute_path(start, goal)
        print(start)
        print(goal)
        print("Path:", path)
        if path:
            self.reserve_path(path)

            for position in path:
                goal_msg = PoseStamped()
                goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z = position
                self.goal_publisher.publish(goal_msg)

                rclpy.spin_once(self)

            self.unreserved(path)

    def print_environment(self):
        print(self.environment)

    def print_reservation(self):
        print(self.reservation_table)

def main(args=None):
    rclpy.init(args=args)

    # Create path planners for each drone
    drone1_planner = PathPlanner(1)
    drone2_planner = PathPlanner(2)

    # Define goals for each drone
    drone1_planner.add_goal(1, (2.5, 2.5, 2.5))
    drone2_planner.add_goal(2, (4.5, 4.5, 4.5))

    node = rclpy.create_node('path_planner_node')
    
    rate = node.create_rate(10)  # Run at 10 Hz
    while rclpy.ok():
        # Plan paths for each drone
        drone1_planner.plan_path()
        #drone2_planner.plan_path()

        #drone1_planner.print_environment()
        #print("------------------")
        #drone1_planner.print_reservation()
        #print("------------------")

        rclpy.spin_once(node)
        rate.sleep()

    drone1_planner.destroy_node()
    drone2_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()