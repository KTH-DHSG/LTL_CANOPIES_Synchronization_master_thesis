from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nodes = [] 
    num_agents = 350
    num_ass_actions = 8
    filter = False   
    for i in range(num_agents):
        node= Node(
            package='synchronization_experiments',
            executable='rrc',
            name='rrc'+str(i),
            namespace='agent'+str(i),
            emulate_tty=True,
            output='screen',
            parameters=[
                {'num_agents': num_agents},
                {'num_ass_actions': num_ass_actions},
                {'filter': filter},
                {'waiting': 350}
                
            ]
        )
        nodes.append(node)   
    
    
    return LaunchDescription(nodes)
    