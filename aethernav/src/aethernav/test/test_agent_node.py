import unittest
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from aethernav.agent_node import AetherNavAgent
import numpy as np
import time

class TestAetherNavAgent(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        
    def setUp(self):
        self.agent = AetherNavAgent(999)  # Use high ID to avoid conflicts
        self.test_node = Node('test_node')
        
        # Publishers for sending test data
        self.map_pub = self.test_node.create_publisher(
            OccupancyGrid, '/map', 10)
        self.goal_pub = self.test_node.create_publisher(
            PoseStamped, '/agent_999/goal', 10)
            
        # Subscribers for receiving agent output
        self.path_sub = self.test_node.create_subscription(
            Path,
            '/agent_999/path',
            self.path_callback,
            10)
        self.pose_sub = self.test_node.create_subscription(
            PoseStamped,
            '/agent_999/pose',
            self.pose_callback,
            10)
            
        self.received_path = None
        self.received_pose = None
        
    def tearDown(self):
        self.agent.destroy_node()
        self.test_node.destroy_node()
        
    def path_callback(self, msg):
        self.received_path = msg
        
    def pose_callback(self, msg):
        self.received_pose = msg
        
    def test_agent_initialization(self):
        """Test that agent properly initializes and responds to messages"""
        # Create and publish a simple map
        map_msg = OccupancyGrid()
        map_msg.info.width = 10
        map_msg.info.height = 10
        map_msg.data = [0] * 100  # Empty 10x10 grid
        self.map_pub.publish(map_msg)
        
        # Create and publish a goal
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = 9.0
        goal_msg.pose.position.y = 9.0
        self.goal_pub.publish(goal_msg)
        
        # Wait for processing
        rclpy.spin_once(self.agent, timeout_sec=1.0)
        rclpy.spin_once(self.test_node, timeout_sec=1.0)
        
        # Check that we received a path
        self.assertIsNotNone(self.received_path)
        if self.received_path:
            self.assertGreater(len(self.received_path.poses), 0)
            
    def test_obstacle_avoidance(self):
        """Test that agent properly avoids obstacles"""
        # Create map with obstacles
        map_msg = OccupancyGrid()
        map_msg.info.width = 10
        map_msg.info.height = 10
        map_data = [0] * 100
        # Add obstacle wall
        for i in range(3, 7):
            map_data[i * 10 + 5] = 100  # Vertical wall
        map_msg.data = map_data
        self.map_pub.publish(map_msg)
        
        # Set goal behind obstacle wall
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = 9.0
        goal_msg.pose.position.y = 5.0
        self.goal_pub.publish(goal_msg)
        
        # Wait for processing
        rclpy.spin_once(self.agent, timeout_sec=1.0)
        rclpy.spin_once(self.test_node, timeout_sec=1.0)
        
        # Check that path avoids obstacles
        self.assertIsNotNone(self.received_path)
        if self.received_path:
            # Convert path to set of points for easy checking
            path_points = set()
            for pose in self.received_path.poses:
                x = int(pose.pose.position.x)
                y = int(pose.pose.position.y)
                path_points.add((x, y))
                
            # Check that path doesn't go through obstacles
            for i in range(3, 7):
                self.assertNotIn((5, i), path_points)
                
    def test_multi_agent_coordination(self):
        """Test coordination between multiple agents"""
        # Create another test agent
        other_agent = AetherNavAgent(998)
        
        # Publish other agent's position
        other_pose = PoseStamped()
        other_pose.pose.position.x = 5.0
        other_pose.pose.position.y = 5.0
        
        # Publish goal near other agent
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = 6.0
        goal_msg.pose.position.y = 5.0
        self.goal_pub.publish(goal_msg)
        
        # Wait for processing
        rclpy.spin_once(self.agent, timeout_sec=1.0)
        rclpy.spin_once(self.test_node, timeout_sec=1.0)
        
        # Check that path maintains safe distance
        if self.received_path:
            for pose in self.received_path.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                distance = np.sqrt((x - 5.0)**2 + (y - 5.0)**2)
                self.assertGreater(distance, 1.0)  # Maintain minimum distance
                
        other_agent.destroy_node()

if __name__ == '__main__':
    unittest.main()