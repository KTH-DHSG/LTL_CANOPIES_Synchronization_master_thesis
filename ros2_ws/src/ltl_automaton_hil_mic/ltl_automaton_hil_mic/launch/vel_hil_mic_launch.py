from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    transition_system_path = os.path.join(get_package_share_directory('ltl_automaton_hil_mic'), 'config', 'test_ts.yaml')
    
    planner_node =Node(
            package='ltl_automaton_planner',
            executable='ltl_planner_node',
            name='ltl_mic_planner',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'agent_name': 'nexus'},
                {'initial_beta': 1000},
                {'hard_task': "([]<> unloaded) && ([]<> loaded) && ([]! r4)"},
                {'soft_task': ""},
                {'transition_system_path': transition_system_path},
                {'initial_ts_state_from_agent_alt': False},
                {'plugin_dictionary_path': os.path.join(get_package_share_directory('ltl_automaton_hil_mic'), 'config', 'plugins.yaml')},
            ]
        ) 
       
    hil_node = Node(
            package='ltl_automaton_hil_mic',
            executable='vel_cmd',
            name='vel_mic_controller',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'state_dimension_name': "2d_pose_region"},                
            ]
        )
    
    return LaunchDescription([planner_node, hil_node])
    