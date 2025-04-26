import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
import numpy as np

class AetherNavViz(Node):
    def __init__(self):
        super().__init__('aethernav_viz')
        self.num_agents = 50  # Maximum number of agents to visualize
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray, '/aethernav/visualization', 10)
            
        # Subscribe to map updates
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
            
        # Subscribe to each agent's path and pose
        self.agent_paths = {}
        self.agent_poses = {}
        for i in range(self.num_agents):
            self.create_subscription(
                Path,
                f'/agent_{i}/path',
                lambda msg, aid=i: self.path_callback(msg, aid),
                10)
            self.create_subscription(
                PoseStamped,
                f'/agent_{i}/pose',
                lambda msg, aid=i: self.pose_callback(msg, aid),
                10)
                
        # Timer for periodic visualization updates
        self.create_timer(0.1, self.publish_visualization)  # 10Hz
        
        # Colors for different agents
        self.colors = self.generate_colors(self.num_agents)
        
    def generate_colors(self, num_colors):
        """Generate distinct colors for each agent"""
        colors = []
        for i in range(num_colors):
            hue = i / num_colors
            # Convert HSV to RGB (assuming S=V=1)
            h = hue * 6
            c = int(h)
            f = h - c
            q = 1 - f
            
            if c == 0:
                rgb = (1, f, 0)
            elif c == 1:
                rgb = (q, 1, 0)
            elif c == 2:
                rgb = (0, 1, f)
            elif c == 3:
                rgb = (0, q, 1)
            elif c == 4:
                rgb = (f, 0, 1)
            else:
                rgb = (1, 0, q)
                
            colors.append(ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0))
        return colors
        
    def map_callback(self, msg: OccupancyGrid):
        """Handle map updates"""
        self.current_map = msg
        
    def path_callback(self, msg: Path, agent_id: int):
        """Handle path updates from agents"""
        self.agent_paths[agent_id] = msg
        
    def pose_callback(self, msg: PoseStamped, agent_id: int):
        """Handle pose updates from agents"""
        self.agent_poses[agent_id] = msg
        
    def create_agent_marker(self, agent_id: int, pose: PoseStamped) -> Marker:
        """Create visualization marker for an agent"""
        marker = Marker()
        marker.header = pose.header
        marker.ns = f"agent_{agent_id}"
        marker.id = agent_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color = self.colors[agent_id % len(self.colors)]
        return marker
        
    def create_path_marker(self, agent_id: int, path: Path) -> Marker:
        """Create visualization marker for an agent's path"""
        marker = Marker()
        marker.header = path.header
        marker.ns = f"path_{agent_id}"
        marker.id = agent_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color = self.colors[agent_id % len(self.colors)]
        marker.points = [pose.pose.position for pose in path.poses]
        return marker
        
    def publish_visualization(self):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        # Add agent markers
        for agent_id, pose in self.agent_poses.items():
            marker_array.markers.append(self.create_agent_marker(agent_id, pose))
            
        # Add path markers
        for agent_id, path in self.agent_paths.items():
            marker_array.markers.append(self.create_path_marker(agent_id, path))
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    viz_node = AetherNavViz()
    rclpy.spin(viz_node)
    viz_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()