from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ltl_automaton_synchronization',
            executable='synchro',
            name='ltl_test_planner',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'agent_name': 'turtlebot'},
                {'initial_beta': 1000},
                {'gamma': 10},
                {'hard_task': "([]<> (r2 && l123))"},
                {'soft_task': ""},
                {'initial_ts_state_from_agent': False},
                {'motion_dictionary_path': os.path.join(get_package_share_directory('ltl_automaton_synchronization'), 'config', 'region_dictionary.yaml')},
                {'action_dictionary_path': os.path.join(get_package_share_directory('ltl_automaton_synchronization'), 'config', 'action_dictionary.yaml')},
            ]
        )        
    ])
    