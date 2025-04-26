import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
import numpy as np
from typing import List, Tuple, Set
from .path_planning.hybrid_planner import HybridPlanner
import yaml
from rclpy.parameter import Parameter

class AetherNavAgent(Node):
    def __init__(self, agent_id: int = 0):
        super().__init__(f'aethernav_agent_{agent_id}')
        self.agent_id = agent_id
        
        # Declare parameters
        self.declare_parameter('scenario_file', '')
        
        self.grid_size = (100, 100)  # Default grid size
        self.planner = HybridPlanner(self.grid_size)
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path, f'/agent_{agent_id}/path', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, f'/agent_{agent_id}/pose', 10)
            
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, f'/agent_{agent_id}/goal', self.goal_callback, 10)
            
        # Subscribe to other agents' poses
        self.other_agents_poses = {}
        for i in range(50):  # Support up to 50 agents
            if i != agent_id:
                self.create_subscription(
                    PoseStamped, 
                    f'/agent_{i}/pose',
                    lambda msg, aid=i: self.other_agent_pose_callback(msg, aid),
                    10)
        
        self.current_pose = None
        self.current_goal = None
        self.obstacles: Set[Tuple[int, int]] = set()
        
        # Create timer for periodic updates
        self.create_timer(0.1, self.update_callback)  # 10Hz update rate
        
        # Load scenario if provided
        self.load_scenario()
        
    def load_scenario(self):
        """Load scenario from YAML file if specified"""
        scenario_file = self.get_parameter('scenario_file').value
        if not scenario_file:
            return
            
        try:
            with open(scenario_file, 'r') as f:
                scenario = yaml.safe_load(f)
                
            # Update grid size
            self.grid_size = tuple(scenario['config']['grid_size'])
            self.planner = HybridPlanner(self.grid_size)
            
            # Set obstacles
            self.obstacles = set(tuple(pos) for pos in scenario['obstacles'])
            
            # Find our agent configuration
            for agent in scenario['agents']:
                if agent['id'] == self.agent_id:
                    self.current_pose = tuple(agent['start'])
                    self.current_goal = tuple(agent['goal'])
                    break
                    
            if self.current_pose and self.current_goal:
                self.update_path()
                
        except Exception as e:
            self.get_logger().error(f'Failed to load scenario: {e}')

    def map_callback(self, msg: OccupancyGrid):
        """Update internal map and obstacles from occupancy grid"""
        self.grid_size = (msg.info.width, msg.info.height)
        self.planner = HybridPlanner(self.grid_size)
        
        # Update obstacles from occupancy grid
        new_obstacles = set()
        for i in range(msg.info.width):
            for j in range(msg.info.height):
                idx = i + j * msg.info.width
                if msg.data[idx] > 50:  # Occupied cell threshold
                    new_obstacles.add((i, j))
                    
        self.obstacles = new_obstacles
        if self.current_pose and self.current_goal:
            self.update_path()
            
    def goal_callback(self, msg: PoseStamped):
        """Handle new goal assignment"""
        self.current_goal = (
            int(msg.pose.position.x),
            int(msg.pose.position.y)
        )
        if self.current_pose:
            self.update_path()
            
    def other_agent_pose_callback(self, msg: PoseStamped, agent_id: int):
        """Track other agents' positions for collision avoidance"""
        self.other_agents_poses[agent_id] = (
            int(msg.pose.position.x),
            int(msg.pose.position.y)
        )
        
    def update_path(self):
        """Update path planning considering current obstacles and other agents"""
        # Add other agents' positions as temporary obstacles
        dynamic_obstacles = self.obstacles.union(set(self.other_agents_poses.values()))
        
        # Check if replanning is needed
        if self.planner.update_obstacles(dynamic_obstacles):
            self.planner.initialize(self.current_pose, self.current_goal)
            new_path = self.planner.replan()
            self.publish_path(new_path)
            
    def publish_path(self, path: List[Tuple[int, int]]):
        """Publish the planned path"""
        msg = Path()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        for x, y in path:
            pose = PoseStamped()
            pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
            msg.poses.append(pose)
            
        self.path_pub.publish(msg)
        
    def update_callback(self):
        """Periodic update function"""
        if not (self.current_pose and self.current_goal):
            return
            
        # Simulate simple motion along the path
        current_path = self.planner.get_current_path()
        if current_path and len(current_path) > 1:
            next_pose = current_path[1]  # Move to next point in path
            self.current_pose = next_pose
            
            # Publish updated pose
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position = Point(
                x=float(next_pose[0]),
                y=float(next_pose[1]),
                z=0.0
            )
            self.pose_pub.publish(pose_msg)
            
def main(args=None):
    rclpy.init(args=args)
    agent = AetherNavAgent(0)  # Start with agent ID 0
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()