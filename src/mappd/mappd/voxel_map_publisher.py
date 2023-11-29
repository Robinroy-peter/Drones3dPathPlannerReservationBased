import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class VoxelMapPublisher(Node):
    def __init__(self):
        super().__init__('voxel_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.map_ = OccupancyGrid()

    def publish_map(self):
        self.map_.header.frame_id = 'map'
        self.map_.info.width = 5
        self.map_.info.height = 5
        self.map_.info.resolution = 0.5
        self.map_.info.origin.position.x = -0.5
        self.map_.info.origin.position.y = -0.5
        self.map_.info.origin.position.z = -0.5
        self.map_.data = [0] * 125  # Assuming all voxels are unoccupied

        self.publisher_.publish(self.map_)

def main(args=None):
    rclpy.init(args=args)
    voxel_map_publisher = VoxelMapPublisher()
    rate = voxel_map_publisher.create_rate(10)  # Set the rate to 10Hz

    try:
        while rclpy.ok():
            voxel_map_publisher.publish_map()
            rclpy.spin_once(voxel_map_publisher)
            rate.sleep()
    except KeyboardInterrupt:
        # Handle keyboard interrupt (e.g., Ctrl+C)
        pass

    voxel_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
