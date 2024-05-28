from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    agents = ['/rosie0','/rosie1' '/rosie2', '/turtlebot1', '/turtlebot2']
    agents_types = ['rosie', 'rosie', 'rosie','turtlebot', 'turtlebot']
    possible_agent_types = ['rosie', 'turtlebot']
    node= Node(
        package='synchronization_experiments',
        executable='data_collector',
        name='data_collecotr',
        namespace='data_collector',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'save_location': "/home/davide/git_thesis/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/results/data_sim_1.pkl"},
            {'agents': agents},
            {'agents_types': agents_types},
            {'possible_agent_types': possible_agent_types},
            {'motion_action_dictionary_paths': [os.path.join(get_package_share_directory('synchronization_experiments'), 'config', 'rosie0_lab.yaml'),
                os.path.join(get_package_share_directory('synchronization_experiments'), 'config', 'turtlebot1_lab.yaml')            
            ]}            
        ]
    )  
    
    
    return LaunchDescription([node])
    