from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    agents = ['/agent_1', '/turtlebot_1', '/turtlebot_2', '/rosie_1']
    
    action_node= Node(
            package='ltl_automaton_synchronization',
            executable='auto_actions',
            name='auto_actions',
            namespace='turtlebot_1',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'motion_action_dictionary_path': os.path.join(get_package_share_directory('ltl_automaton_synchronization'), 'config', 'turtlebot.yaml')},
                {'agents': agents},
            ]
        )   
    
    
    planner_node= Node(
            package='ltl_automaton_synchronization',
            executable='synchro',
            name='ltl_synchro_planner',
            namespace='turtlebot_1',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'initial_beta': 1000},
                {'gamma': 10},
                {'hard_task':'[]<> (r2)' },#"(r1 && (X r3) && (X X c13))"
                #X r3 && (<> r2) && ([]<> r1)
                #'[]<> (r2 && l123 && (X<> (r3 && c13)))'
                # "((r1 && (X r3) && (X X c13)) && ([]<> (l123 && r2))"
                # '(r1 && (X r3) && (X X c13)) && ([]<> (r2 && l123 && (X<> (r3 && c13))))'
                {'soft_task': ""},
                {'initial_ts_state_from_agent': False},
                {'motion_action_dictionary_path': os.path.join(get_package_share_directory('ltl_automaton_synchronization'), 'config', 'turtlebot.yaml')},
                {'agents': agents},
            ]
        )   
    
    
    
    return LaunchDescription([action_node, planner_node])
    