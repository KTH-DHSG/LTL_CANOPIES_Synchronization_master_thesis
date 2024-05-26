from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    agents = ['/turtlebot1', '/turtlebot2']
    dynamic_obstacles = ['/rosie0', '/turtlebot2']
    action_node= Node(
            package='ltl_automaton_synchronization',
            executable='auto_actions',
            name='auto_actions',
            namespace='turtlebot1',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'motion_action_dictionary_path': os.path.join(get_package_share_directory('synchronization_experiments'), 'config', 'turtlebot1_lab.yaml')},
                {'agents': agents},
                {'dynamic_obstacles': dynamic_obstacles},
                {'obstacles_dictionary_path': os.path.join(get_package_share_directory('synchronization_experiments'), 'config', 'obstacles.yaml')},                
                {'is_simulation': False},
            ]
        )   
    
    
    planner_node= Node(
            package='ltl_automaton_synchronization',
            executable='synchro',
            name='ltl_synchro_planner',
            namespace='turtlebot1',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'hard_task':'[]<> (patrol && p1 && <> (patrol && p2 ))' },#<>(check_connection && c1) && 
                {'soft_task': ""},
                {'initial_ts_state_from_agent': False},
                {'motion_action_dictionary_path': os.path.join(get_package_share_directory('synchronization_experiments'), 'config', 'turtlebot1_lab.yaml')},
                {'agents': agents},
                {'time_horizon': 10},
            ]
        )   
    
    
    
    return LaunchDescription([action_node, planner_node])
    