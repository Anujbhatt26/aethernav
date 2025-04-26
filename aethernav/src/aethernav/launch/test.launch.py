from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Run unit tests
        ExecuteProcess(
            cmd=['python', '-m', 'pytest', 
                 'test/test_astar.py',
                 'test/test_dstar_lite.py',
                 'test/test_hybrid_planner.py',
                 '-v'],
            name='unit_tests',
        ),
        
        # Run integration tests
        ExecuteProcess(
            cmd=['python', '-m', 'pytest', 
                 'test/test_agent_node.py',
                 '-v'],
            name='integration_tests',
        ),
    ])