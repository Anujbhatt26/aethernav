from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    num_agents_arg = DeclareLaunchArgument(
        'num_agents',
        default_value='10',
        description='Number of agents to spawn'
    )
    
    nodes = [
        # Visualization node
        Node(
            package='aethernav',
            executable='viz_node',
            name='aethernav_viz',
            output='screen'
        )
    ]
    
    # Create nodes for each agent
    num_agents = LaunchConfiguration('num_agents')
    for i in range(int(num_agents.perform(None))):
        agent_node = Node(
            package='aethernav',
            executable='agent_node',
            name=f'agent_{i}',
            parameters=[{'agent_id': i}],
            remappings=[
                ('path', f'/agent_{i}/path'),
                ('pose', f'/agent_{i}/pose'),
                ('goal', f'/agent_{i}/goal')
            ]
        )
        nodes.append(agent_node)
    
    return LaunchDescription([num_agents_arg] + nodes)