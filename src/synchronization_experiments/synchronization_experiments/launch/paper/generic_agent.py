from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
import json

def generate_launch_description():
    agents = LaunchConfiguration('agents')
    dictionary_file = LaunchConfiguration('dictionary_file')
    task = LaunchConfiguration('task')
    ns = LaunchConfiguration('ns')
    
    agents_arg = DeclareLaunchArgument(
        'agents',
        default_value='["/rosie0", "/rosie1", "/rosie2", "/turtlebot1", "/turtlebot2", "/turtlebot3", "/turtlebot4", "/turtlebot5"]'
    )
    
    dictionary_file_arg = DeclareLaunchArgument(
        'dictionary_file',
        default_value=os.path.join(get_package_share_directory('synchronization_experiments'), 'config/paper', 'rosie0.yaml')
    )
    task_arg = DeclareLaunchArgument(
        'task',
        default_value='X wait && []<> (harvest && h1 && <> (harvest && h3 && <> deliver))'
    )
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='rosie0'
    )
    
    dynamic_obstacles = ['/bin']
    action_node= Node(
            package='ltl_automaton_synchronization',
            executable='auto_actions',
            name='auto_actions',
            namespace=ns,
            emulate_tty=True,
            output='screen',
            parameters=[
                {'motion_action_dictionary_path': dictionary_file},
                {'agents': agents},
                {'dynamic_obstacles': dynamic_obstacles},
                {'obstacles_dictionary_path': os.path.join(get_package_share_directory('synchronization_experiments'), 'config/paper', 'obstacles.yaml')},                
                {'is_simulation': True},
            ]
        )   
    
    
    planner_node= Node(
            package='ltl_automaton_synchronization',
            executable='synchro',
            name='ltl_synchro_planner',
            namespace= ns,
            emulate_tty=True,
            output='screen',
            parameters=[
                {'hard_task': task },
                {'soft_task': ""},
                {'initial_ts_state_from_agent': False},
                {'motion_action_dictionary_path': dictionary_file},
                {'agents': agents},
                {'time_horizon': 10},
            ]
        )   
    
    
    
    return LaunchDescription([agents_arg, dictionary_file_arg, task_arg, ns_arg, action_node, planner_node])
    