from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    agents = ['/rosie0','/rosie1', '/rosie2','/turtlebot1', '/turtlebot2', '/turtlebot3', '/turtlebot4', '/turtlebot5', '/turtlebot6']#, '/turtlebot7']
    dynamic_obstacles = ['/rosie0', '/rosie1', '/rosie2', '/turtlebot1', '/turtlebot2', '/turtlebot3', '/turtlebot4', '/turtlebot5', '/basket']#, '/turtlebot7']
    action_node= Node(
            package='ltl_automaton_synchronization',
            executable='auto_actions',
            name='auto_actions',
            namespace='turtlebot6',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'motion_action_dictionary_path': os.path.join(get_package_share_directory('synchronization_experiments'), 'config/paper', 'turtlebot6.yaml')},
                {'agents': agents},
                {'dynamic_obstacles': dynamic_obstacles},
                {'obstacles_dictionary_path': os.path.join(get_package_share_directory('synchronization_experiments'), 'config/paper', 'obstacles.yaml')},                
                {'is_simulation': False},
            ]
        )   
    
    
    planner_node= Node(
            package='ltl_automaton_synchronization',
            executable='synchro',
            name='ltl_synchro_planner',
            namespace='turtlebot6',
            emulate_tty=True,
            output='screen',
            parameters=[
                #{'hard_task':'[]<> (patrol && p3 && <> (patrol && p10 && <> group))' },
                {'hard_task':'X wait && <> (remove_object && <>(patrol && p5 && <>(check_connection && c2))) && []<> (patrol && p5 && <> (patrol && p7))' },
                {'soft_task': ""},
                {'initial_ts_state_from_agent': False},
                {'motion_action_dictionary_path': os.path.join(get_package_share_directory('synchronization_experiments'), 'config/paper', 'turtlebot6.yaml')},
                {'agents': agents},
                {'time_horizon': 10},
            ]
        )   
    
    
    
    return LaunchDescription([action_node, planner_node])
    