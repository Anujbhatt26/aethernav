from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    scenario_file_arg = DeclareLaunchArgument(
        'scenario_file',
        default_value='scenarios/simple_scenario.yaml',
        description='Path to the scenario YAML file'
    )
    
    return LaunchDescription([
        scenario_file_arg,
        
        # Start visualization
        Node(
            package='aethernav',
            executable='viz_node',
            name='visualization'
        ),
        
        # Start performance monitoring
        Node(
            package='aethernav',
            executable='monitor_node',
            name='performance_monitor'
        ),
        
        # Load and run scenario
        Node(
            package='aethernav',
            executable='agent_node',
            name='scenario_runner',
            parameters=[{
                'scenario_file': LaunchConfiguration('scenario_file')
            }]
        )
    ])