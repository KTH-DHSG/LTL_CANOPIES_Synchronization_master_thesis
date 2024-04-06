from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    planner_node= Node(
            package='ltl_automaton_planner',
            executable='ltl_planner_node',
            name='ltl_planner',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'agent_name': 'turtlebot'},
                {'initial_beta': 1000},
                {'gamma': 10},
                {'hard_task': "([]<> (r1 && loaded)) && ([]<> (r1 && unloaded))"},
                {'soft_task': "[]!r3"},
                {'transition_system_path': os.path.join(get_package_share_directory('ltl_automaton_planner'), 'config', 'example_ts.yaml')},
                {'initial_ts_state_from_agent': False},
                # Plugin commented out becasue the file is just a test
                #{'plugin_dictionary_path': os.path.join(get_package_share_directory('ltl_automaton_planner'), 'config', 'plugins.yaml')},    
            ]
        ) 
    return LaunchDescription([
               planner_node
    ])
    