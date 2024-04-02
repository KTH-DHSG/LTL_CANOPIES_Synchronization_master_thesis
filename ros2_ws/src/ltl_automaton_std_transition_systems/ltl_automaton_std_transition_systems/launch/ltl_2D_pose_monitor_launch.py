from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ltl_automaton_std_transition_systems',
            executable='ltl_2D_pose_monitor_node',
            name='ltl_2D_region_monitor',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'transition_system_path': os.path.join(get_package_share_directory('ltl_automaton_std_transition_systems'), 'config', 'ms4_nexus_ws.yaml')},   
            ]
        )        
    ])