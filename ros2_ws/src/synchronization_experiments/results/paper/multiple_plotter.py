import matplotlib.pyplot as plt
import pickle
import numpy as np
#Path to the pickle file
file_path = '/home/davide/git_thesis/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/results/paper/90_agents_mod.pkl'
# Open the pickle file and load the data
with open(file_path, 'rb') as file:
    data = pickle.load(file)
#print(data.values())


#added by hand because it is easier to do
#initial_regions = ['H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M','H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M' ,'H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M' ,'H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M','H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M' ,'H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M' ,'H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M' ,'H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M'  ]
# Assuming data is a dictionary with agent names as keys and AgentData as values
agents = list(data.keys())
activity_colors = {'assisting': 'red', 'local': 'blue', 'collaborative': 'green'}
activity_markers = {'assisting': 's', 'local': 'o', 'collaborative': 'd'}
collab_marker = '*'

fig, ax = plt.subplots(figsize=(16, 9))
plot_limit= 395
# Plotting the data
y_labels = ['/rosie0', '/rosie1', '/rosie2', '/turtlebot0','/turtlebot1', '/turtlebot2', '/turtlebot3', '/turtlebot4', '/turtlebot5', '....  ', '....  ', '....  ', '/rosie27', '/rosie28', '/rosie29', '/turtlebot54', '/turtlebot55', '/turtlebot56', '/turtlebot57', '/turtlebot58', '/turtlebot59']
initial_regions = ['H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M', ' ', ' ' ,'H1', 'M','H2', 'P1', 'P2', 'P4', 'P12', 'P3', 'M', 'M']
i=0
delay=200
for l, agent in enumerate(agents):
    if agent == '/turtlebot1' or agent == '/turtlebot2' or agent == '/turtlebot3' or agent == '/turtlebot4' or agent == '/turtlebot5' or agent == '/turtlebot0' or agent == '/rosie0' or agent == '/rosie1' or agent == '/rosie2' or agent == '/turtlebot54' or agent == '/turtlebot55' or agent == '/turtlebot56' or agent == '/turtlebot57' or agent == '/turtlebot58' or agent == '/turtlebot59' or agent == '/rosie27' or agent == '/rosie28' or agent == '/rosie29':
        if agent == '/turtlebot4':
            agent_data = data['/turtlebot40']
            actions = agent_data.action
            action_types = agent_data.action_type
            action_start= agent_data.action_start
            collab_start = agent_data.collab_start
            for j in range(len(action_start)):
                action_start[j] += -62.5
            for j in range(len(collab_start)):
                collab_start[j] += -62.5
        elif agent =='/rosie28':
            agent_data = data['/rosie25']
            actions = agent_data.action
            action_types = agent_data.action_type
            action_start = agent_data.action_start
            collab_start = agent_data.collab_start
            delay=292
            plot_limit = 395+92
        else:
            agent_data = data[agent]
            actions = agent_data.action
            action_types = agent_data.action_type
            action_start = agent_data.action_start
            collab_start = agent_data.collab_start
        last_region = initial_regions[i]
        if agent == '/turtlebot55' or agent == '/turtlebot56' or agent == '/turtlebot59'  or agent == '/rosie27' or agent == '/rosie29':
            delay=287
            plot_limit = 395+87
        elif agent == "/turtlebot54":
            delay=292
            plot_limit = 395+92
        elif agent == '/turtlebot58' or agent == '/turtlebot57':
            delay= 291
            plot_limit = 395+91
        elif agent == '/turtlebot1' or agent == '/turtlebot2' or agent == '/turtlebot3' or agent == '/turtlebot4' or agent == '/turtlebot5' or agent == '/turtlebot0' or agent == '/rosie0' or agent == '/rosie2':
            delay=200
            plot_limit= 395
        elif agent == "/rosie1":
            delay = 210
            plot_limit = 395+10
        for j in range(len(action_start) - 1):
            start_time = action_start[j]
            end_time = action_start[j + 1]
            action = action_types[j]
            color = activity_colors[action]
            marker = activity_markers[action]
            
            if not (actions[j].startswith('goto') or actions[j].startswith('none') or actions[j].startswith('wait')):#
                ax.hlines(i, start_time-delay, end_time-delay, colors=color, linewidth=4)
                ax.scatter([start_time-delay, end_time-delay], [i, i], color=color, marker=marker, s=75)
                # Add action name above the line
                if start_time <= plot_limit:       
                    if actions[j].startswith('manipulate'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'm$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32)
                    elif actions[j].startswith('h_check_connection'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'hcc$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32)
                    elif actions[j].startswith('check_connection'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'cc$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32)
                    elif actions[j].startswith('patrol'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'p$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32)
                    elif actions[j].startswith('harvest'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'h$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32)
                    elif actions[j].startswith('deliver'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'd$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32) 
                    elif actions[j].startswith('h_remove_object'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'hro$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32) 
                    elif actions[j].startswith('supervise'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f's$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32) 
                    elif actions[j].startswith('remove_object'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'ro$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32) 
                    elif actions[j].startswith('group'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'g$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32) 
                    elif actions[j].startswith('h_group'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'hg$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32) 
                    elif actions[j].startswith('load'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'l$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32) 
                    elif actions[j].startswith('h_load'):
                        ax.text((start_time + end_time-2*delay) / 2, i -0.8, f'hl$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32) 
                    else:
                        ax.text((start_time + end_time) / 2, i -0.8, f'{actions[j]}$_{{{last_region}}}$', ha='center', va='bottom', fontsize=32)
            elif actions[j].startswith('goto'):
                temp = actions[j].split('_')[-1]
                if len(temp) == 1:
                    last_region = temp[0].upper()
                else:
                    last_region = temp[0].upper() + temp[1:]
                    
        # Plot the last action that ends with the plot range end
        start_time = action_start[-1]
        if start_time <= plot_limit:
            end_time = 1000 # or any other endpoint
            action = action_types[-1]
            color = activity_colors[action]
            marker = activity_markers[action]
            if not (actions[j].startswith('goto') or actions[j].startswith('none') or actions[j].startswith('wait')):
                ax.hlines(i, start_time-delay, end_time-delay, colors=color, linewidth=4)
                ax.scatter([start_time-delay, end_time-delay], [i, i], color=color, marker=marker, s=75)
                # Add action name above the line
                #ax.text((start_time + -2*delay) / 2, i -0.5, f'{actions[-1]}$_{{{last_region}}}$', ha='center', va='bottom', fontsize=16)
                
        # Plot collaboration start times
        #print(collab_start)
        for j in range(len(collab_start)):
            collab_start[j] += -delay
        ax.scatter(collab_start, [i] * len(collab_start), color='black', marker=collab_marker, s=300,zorder=3)
        if i == 8:
            i=12
        else:
            i+=1
# add continue markers
values = [12.5, 37.5, 62.5, 87.5, 112.5, 137.5, 162.5, 187.5, 212.5, 237.5, 262.5, 287.5, 312.5, 337.5, 362.5, 387.5]
for i in range(9, 12):
    ax.scatter(values, [i] * len(values), color='black', marker='o', s=20,zorder=3)
        

# Create a custom legend
handles = []
labels = []

# Add a handle for each activity type
for action in activity_colors.keys():
    handles.append(plt.Line2D([0], [0], color=activity_colors[action], linewidth=4))
    labels.append(action)

# Add a handle for the collaboration start marker
handles.append(plt.Line2D([0], [0], color='black', marker=collab_marker, linestyle='None', markersize=26))
labels.append('collaboration start')



#ax.legend(handles, labels)
ax.legend(handles, labels, fancybox=True, shadow=True, ncol=4, prop={'size': 36})

plot_limit= 395
ax.grid(axis='x')
# Set labels and title
ax.set_xlabel('Time [s]', fontsize=32)
ax.set_yticks(range(len(y_labels)))


ax.set_yticklabels([f'{agent[1:-2]}$_{{{agent[-2]+agent[-1]}}}$' if (agent == '/turtlebot54' or agent == '/turtlebot55' or agent == '/turtlebot56' or agent == '/turtlebot57' or agent == '/turtlebot58' or agent == '/turtlebot59' or agent == '/rosie27' or agent == '/rosie29' or agent=='/rosie28') else f'{agent[1:-1]}$_{{{agent[-1]}}}$' for agent in y_labels], fontsize=32)

ax.set_xticks(np.arange(0, plot_limit+ 1, 25))
ax.tick_params(axis='x', labelsize=28)
plt.subplots_adjust(left=0.09, right=0.99, top=0.99, bottom=0.07)
# set limit of plot
ax.set_xlim(-2.5, plot_limit-195)
ax.set_ylim(-1, 22.5)

plt.show()