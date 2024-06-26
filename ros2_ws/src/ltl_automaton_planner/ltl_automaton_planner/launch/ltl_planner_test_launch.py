from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ltl_automaton_planner',
            executable='ltl_planner_node',
            name='ltl_test_planner',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'agent_name': 'turtlebot'},
                {'initial_beta': 1000},
                {'gamma': 10},
                {'hard_task': "([]<> unloaded) && ([]<> loaded) && ([]! r4)"},
                {'soft_task': ""},
                {'transition_system_path': os.path.join(get_package_share_directory('ltl_automaton_planner'), 'config', 'test_ts.yaml')},
                {'initial_ts_state_from_agent': False},
            ]
        )        
    ])
    