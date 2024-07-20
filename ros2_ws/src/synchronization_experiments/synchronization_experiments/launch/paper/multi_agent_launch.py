import os
from ament_index_python.packages import get_package_share_directory
import json
import subprocess

def launch_agents(batches):
    num_agents = batches*9
    num_turtlebot = 0
    num_rosie = 0
    task=[  'X wait && []<> (harvest && h1 && <> (harvest && h3 && <> deliver))',
            'X wait &&  []<> (manipulate && <> (supervise && <> load))',
            'X wait && []<> (harvest && h2 && <> (harvest && h4 && <> deliver))',
            'X wait && []<> (patrol && p1 && <> (patrol && p11))',
            'X wait && []<> (patrol && p2 && <> (patrol && p10))',
            'X wait && []<> (patrol && p4 && <> (patrol && p6))',
            'X wait && []<> (patrol && p12 && <> (patrol && p9 && <> (check_connection && c1)))',
            'X wait && []<> (patrol && p3 && <> (patrol && p8 && <> group))',
            'X wait && <> (remove_object && <>(patrol && p5 && <>(check_connection && c2))) && []<> (patrol && p5 && <> (patrol && p7))' 
    ]
    agents=[]
    namespace=[]
    motion_action_dictionary_file=[]
    tasks=[]
    for i in range(batches):
        for j in range(9):
            if j < 3:
                agents.append('/rosie'+str(num_rosie))
                namespace.append('rosie'+str(num_rosie))
                motion_action_dictionary_file.append(os.path.join(get_package_share_directory('synchronization_experiments'), 'config/paper','rosie'+str(j)+'.yaml'))
                tasks.append(task[j])
                num_rosie+=1
            else:
                agents.append('/turtlebot'+str(num_turtlebot))
                namespace.append('turtlebot'+str(num_turtlebot))
                motion_action_dictionary_file.append(os.path.join(get_package_share_directory('synchronization_experiments'), 'config/paper', 'turtlebot'+str(j-2)+'.yaml'))
                tasks.append(task[j])
                num_turtlebot+=1
    #print(agents)
    #print(motion_action_dictionary_file)
    #print(tasks)
    agents_str= json.dumps(agents)
    agents_str="'"+agents_str+"'"
    
    
    
    curr_agent=0
    commands=[]
    for i in range(batches):        
        for j in range(9):
            commands.append('"ros2 launch synchronization_experiments generic_agent.py agents:='+agents_str+' dictionary_file:='+motion_action_dictionary_file[curr_agent]+" task:='"+tasks[curr_agent]+"' ns:="+namespace[curr_agent]+'"')
            curr_agent+=1

    layout_file = 'commands12.txt'   

    commands= '\n'.join(commands)
        # Write the layout to a temporary file
    with open(layout_file, 'w') as f:
        f.write(commands)

            # Run terminator with the specified layout
        #subprocess.run(f"terminator --layout default --config {layout_file}", shell=True)
        







if __name__ == '__main__':
    launch_agents(12)