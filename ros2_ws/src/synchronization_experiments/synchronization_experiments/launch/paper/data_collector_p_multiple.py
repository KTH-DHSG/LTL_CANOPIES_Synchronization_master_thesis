from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    agents = LaunchConfiguration('agents')
    agents_types = LaunchConfiguration('agents_types')
    
    agents_arg = DeclareLaunchArgument(
        'agents',
        default_value='["/rosie0", "/rosie1", "/rosie2", "/turtlebot1", "/turtlebot2", "/turtlebot3", "/turtlebot4", "/turtlebot5"]'
    )
    agents_types_arg = DeclareLaunchArgument(
        'agents',
        default_value='["/rosie0", "/rosie1", "/rosie2", "/turtlebot1", "/turtlebot2", "/turtlebot3", "/turtlebot4", "/turtlebot5"]'
    )

    possible_agent_types = ['rosie', 'turtlebot']
    node= Node(
        package='synchronization_experiments',
        executable='data_collector',
        name='data_collecotr',
        namespace='data_collector',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'save_location': "/home/davideperon/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/results/paper/multiple_sim_1.pkl"},
            {'agents': agents},
            {'agents_types': agents_types},
            {'possible_agent_types': possible_agent_types},
            {'motion_action_dictionary_paths': [os.path.join(get_package_share_directory('synchronization_experiments'), 'config/paper', 'rosie0.yaml'),
                os.path.join(get_package_share_directory('synchronization_experiments'), 'config/paper', 'turtlebot6.yaml')            
            ]}            
        ]
    )  
    
    
    return LaunchDescription([agents_arg, agents_types_arg, node])
    