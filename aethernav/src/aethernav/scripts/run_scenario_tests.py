#!/usr/bin/env python3

import rclpy
import os
import json
import yaml
import time
from datetime import datetime
from launch import LaunchService
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import matplotlib.pyplot as plt
import numpy as np

def run_scenario_test(scenario_file: str, duration: float) -> dict:
    """Run a single scenario test and collect metrics"""
    # Initialize ROS2
    rclpy.init()
    
    # Create launch description
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'scenario_file',
            default_value=scenario_file
        ),
        
        Node(
            package='aethernav',
            executable='viz_node',
            name='visualization'
        ),
        
        Node(
            package='aethernav',
            executable='monitor_node',
            name='performance_monitor'
        ),
        
        Node(
            package='aethernav',
            executable='agent_node',
            name='scenario_runner',
            parameters=[{
                'scenario_file': LaunchConfiguration('scenario_file')
            }]
        )
    ])
    
    # Run the scenario
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run(duration=duration)
    
    # Collect metrics
    metrics_file = 'metrics/performance_metrics.json'
    with open(metrics_file, 'r') as f:
        metrics = [json.loads(line) for line in f]
    
    # Process metrics
    result = {
        'scenario': os.path.basename(scenario_file),
        'duration': duration,
        'avg_cpu': np.mean([m['system_metrics']['cpu_percent'] for m in metrics]),
        'avg_memory': np.mean([m['system_metrics']['memory_percent'] for m in metrics]),
        'avg_path_length': np.mean([m['path_metrics']['avg_path_length'] for m in metrics]),
        'num_replannings': sum([m['path_metrics']['num_replannings'] for m in metrics]),
        'min_agent_distance': min([m['coordination_metrics'].get('min_agent_distance', float('inf')) 
                                 for m in metrics])
    }
    
    rclpy.shutdown()
    return result

def generate_report(results: list):
    """Generate performance report with graphs"""
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    report_dir = f'metrics/report_{timestamp}'
    os.makedirs(report_dir, exist_ok=True)
    
    # Save raw results
    with open(f'{report_dir}/results.json', 'w') as f:
        json.dump(results, f, indent=2)
    
    # Generate plots
    scenarios = [r['scenario'] for r in results]
    
    # CPU Usage
    plt.figure(figsize=(10, 6))
    plt.bar(scenarios, [r['avg_cpu'] for r in results])
    plt.title('Average CPU Usage by Scenario')
    plt.xticks(rotation=45)
    plt.ylabel('CPU %')
    plt.tight_layout()
    plt.savefig(f'{report_dir}/cpu_usage.png')
    
    # Memory Usage
    plt.figure(figsize=(10, 6))
    plt.bar(scenarios, [r['avg_memory'] for r in results])
    plt.title('Average Memory Usage by Scenario')
    plt.xticks(rotation=45)
    plt.ylabel('Memory %')
    plt.tight_layout()
    plt.savefig(f'{report_dir}/memory_usage.png')
    
    # Path Planning Performance
    plt.figure(figsize=(10, 6))
    plt.bar(scenarios, [r['avg_path_length'] for r in results])
    plt.title('Average Path Length by Scenario')
    plt.xticks(rotation=45)
    plt.ylabel('Path Length')
    plt.tight_layout()
    plt.savefig(f'{report_dir}/path_length.png')
    
    # Generate summary report
    with open(f'{report_dir}/summary.md', 'w') as f:
        f.write('# AetherNav Performance Test Results\n\n')
        f.write(f'Test Run: {timestamp}\n\n')
        
        for result in results:
            f.write(f'## {result["scenario"]}\n')
            f.write(f'- Duration: {result["duration"]} seconds\n')
            f.write(f'- Average CPU Usage: {result["avg_cpu"]:.2f}%\n')
            f.write(f'- Average Memory Usage: {result["avg_memory"]:.2f}%\n')
            f.write(f'- Average Path Length: {result["avg_path_length"]:.2f}\n')
            f.write(f'- Number of Replannings: {result["num_replannings"]}\n')
            f.write(f'- Minimum Agent Distance: {result["min_agent_distance"]:.2f}\n\n')

def main():
    # Get list of scenario files
    scenario_dir = 'scenarios'
    scenario_files = [os.path.join(scenario_dir, f) 
                     for f in os.listdir(scenario_dir) 
                     if f.endswith('.yaml')]
    
    results = []
    for scenario_file in scenario_files:
        print(f'Running scenario: {scenario_file}')
        
        # Load scenario configuration
        with open(scenario_file, 'r') as f:
            config = yaml.safe_load(f)
            duration = config['config']['scenario_duration']
        
        # Run test and collect results
        result = run_scenario_test(scenario_file, duration)
        results.append(result)
        print(f'Completed scenario: {scenario_file}')
        
    # Generate report
    generate_report(results)
    print('Testing complete. Report generated in metrics/report_*')

if __name__ == '__main__':
    main()