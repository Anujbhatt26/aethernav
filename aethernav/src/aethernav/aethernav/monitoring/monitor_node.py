import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Header
import numpy as np
import time
from typing import Dict, List
import psutil
import json

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        self.num_agents = 50
        self.metrics = {}
        
        # Performance metrics publishers
        self.metrics_pub = self.create_publisher(
            Float32MultiArray, '/aethernav/performance_metrics', 10)
            
        # Subscribe to all agent paths and poses
        self.path_times = {}  # Track when paths were last updated
        self.agent_positions = {}
        self.path_lengths = {}
        
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
                
        # Create timer for periodic metric collection
        self.create_timer(1.0, self.collect_metrics)  # 1Hz
        
        # Initialize metrics file
        self.metrics_file = 'performance_metrics.json'
        self.initialize_metrics_file()
        
    def initialize_metrics_file(self):
        """Initialize the metrics log file"""
        initial_metrics = {
            'timestamp': time.time(),
            'system_metrics': {},
            'path_metrics': {},
            'coordination_metrics': {}
        }
        with open(self.metrics_file, 'w') as f:
            json.dump(initial_metrics, f)
            
    def path_callback(self, msg: Path, agent_id: int):
        """Track path updates and calculate path metrics"""
        current_time = time.time()
        
        # Calculate path length
        path_length = 0
        for i in range(len(msg.poses) - 1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i + 1].pose.position
            path_length += np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            
        self.path_lengths[agent_id] = path_length
        
        # Track replanning frequency
        if agent_id in self.path_times:
            time_since_last = current_time - self.path_times[agent_id]
            if 'replanning_intervals' not in self.metrics:
                self.metrics['replanning_intervals'] = []
            self.metrics['replanning_intervals'].append(time_since_last)
            
        self.path_times[agent_id] = current_time
        
    def pose_callback(self, msg: PoseStamped, agent_id: int):
        """Track agent positions for coordination metrics"""
        self.agent_positions[agent_id] = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        
    def calculate_coordination_metrics(self):
        """Calculate metrics related to multi-agent coordination"""
        if len(self.agent_positions) < 2:
            return {}
            
        # Calculate minimum distances between agents
        min_distances = []
        for aid1 in self.agent_positions:
            for aid2 in self.agent_positions:
                if aid1 < aid2:
                    pos1 = self.agent_positions[aid1]
                    pos2 = self.agent_positions[aid2]
                    dist = np.sqrt(
                        (pos2[0] - pos1[0])**2 + 
                        (pos2[1] - pos1[1])**2
                    )
                    min_distances.append(dist)
                    
        return {
            'min_agent_distance': min(min_distances) if min_distances else float('inf'),
            'avg_agent_distance': np.mean(min_distances) if min_distances else float('inf'),
            'num_active_agents': len(self.agent_positions)
        }
        
    def collect_system_metrics(self):
        """Collect system resource usage metrics"""
        return {
            'cpu_percent': psutil.cpu_percent(),
            'memory_percent': psutil.virtual_memory().percent,
            'num_threads': psutil.Process().num_threads()
        }
        
    def collect_path_metrics(self):
        """Collect path planning performance metrics"""
        return {
            'avg_path_length': np.mean(list(self.path_lengths.values())) if self.path_lengths else 0,
            'max_path_length': max(self.path_lengths.values()) if self.path_lengths else 0,
            'avg_replan_interval': np.mean(self.metrics.get('replanning_intervals', [0])),
            'num_replannings': len(self.metrics.get('replanning_intervals', []))
        }
        
    def collect_metrics(self):
        """Periodic collection and publishing of all metrics"""
        current_metrics = {
            'timestamp': time.time(),
            'system_metrics': self.collect_system_metrics(),
            'path_metrics': self.collect_path_metrics(),
            'coordination_metrics': self.calculate_coordination_metrics()
        }
        
        # Log metrics to file
        with open(self.metrics_file, 'a') as f:
            json.dump(current_metrics, f)
            f.write('\n')
            
        # Publish metrics for real-time monitoring
        msg = Float32MultiArray()
        metrics_array = [
            current_metrics['system_metrics']['cpu_percent'],
            current_metrics['system_metrics']['memory_percent'],
            current_metrics['path_metrics']['avg_path_length'],
            current_metrics['coordination_metrics']['min_agent_distance']
        ]
        msg.data = metrics_array
        self.metrics_pub.publish(msg)
        
        # Clear temporary metrics
        self.metrics['replanning_intervals'] = []

def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()