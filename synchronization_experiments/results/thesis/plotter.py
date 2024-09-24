import matplotlib.pyplot as plt
import pickle
import numpy as np
#Path to the pickle file
file_path = '/home/davide/git_thesis/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/results/thesis/data_sim_nominal.pkl'
# Open the pickle file and load the data
with open(file_path, 'rb') as file:
    data = pickle.load(file)
print(data)

#added by hand because it is easier to do
initial_regions = ['m1', 'm3','h', 'p1', 'p3' ]
# Assuming data is a dictionary with agent names as keys and AgentData as values
agents = list(data.keys())
activity_colors = {'assisting': 'red', 'local': 'blue', 'collaborative': 'green'}
activity_markers = {'assisting': 's', 'local': 'o', 'collaborative': 'd'}
collab_marker = '*'

fig, ax = plt.subplots(figsize=(16, 9))
plot_limit= 300
# Plotting the data
for i, agent in enumerate(agents):
    agent_data = data[agent]
    actions = agent_data.action
    action_types = agent_data.action_type
    action_start = agent_data.action_start
    collab_start = agent_data.collab_start
    last_region = initial_regions[i]
    for j in range(len(action_start) - 1):
        start_time = action_start[j]
        end_time = action_start[j + 1]
        action = action_types[j]
        color = activity_colors[action]
        marker = activity_markers[action]
        if start_time <= plot_limit:
            if not (actions[j].startswith('goto') or actions[j].startswith('none')):
                ax.hlines(i, start_time, end_time, colors=color, linewidth=2.5)
                ax.scatter([start_time, end_time], [i, i], color=color, marker=marker)
                # Add action name above the line
                if actions[j].startswith('manipulate'):
                    ax.text((start_time + end_time) / 2, i -0.15, f'man$_{{{last_region}}}$', ha='center', va='bottom', fontsize=16)
                elif actions[j].startswith('h_check_connection'):
                    ax.text((start_time + end_time) / 2, i -0.15, f'h_c_c$_{{{last_region}}}$', ha='center', va='bottom', fontsize=16)
                elif actions[j].startswith('check_connection'):
                    ax.text((start_time + end_time) / 2, i -0.15, f'c_c$_{{{last_region}}}$', ha='center', va='bottom', fontsize=16)
                elif actions[j].startswith('patrol'):
                    ax.text((start_time + end_time) / 2, i -0.15, f'pat$_{{{last_region}}}$', ha='center', va='bottom', fontsize=16)
                elif actions[j].startswith('harvest'):
                    ax.text((start_time + end_time) / 2, i -0.15, f'har$_{{{last_region}}}$', ha='center', va='bottom', fontsize=16)
                elif actions[j].startswith('deliver'):
                    ax.text((start_time + end_time) / 2, i -0.15, f'del$_{{{last_region}}}$', ha='center', va='bottom', fontsize=16) 
                else:
                    ax.text((start_time + end_time) / 2, i -0.15, f'{actions[j]}$_{{{last_region}}}$', ha='center', va='bottom', fontsize=16)
            elif actions[j].startswith('goto'):
                last_region = actions[j].split('_')[-1]
    # Plot the last action that ends with the plot range end
    start_time = action_start[-1]
    if start_time <= plot_limit:
        end_time = 1000 # or any other endpoint
        action = action_types[-1]
        color = activity_colors[action]
        marker = activity_markers[action]
        if not (actions[j].startswith('goto') or actions[j].startswith('none')):
            ax.hlines(i, start_time, end_time, colors=color, linewidth=2.5)
            ax.scatter([start_time, end_time], [i, i], color=color, marker=marker)
            # Add action name above the line
            ax.text((start_time + end_time) / 2, i -0.15, f'{actions[-1]}$_{{{last_region}}}$', ha='center', va='bottom', fontsize=16)
            
    # Plot collaboration start times
    ax.scatter(collab_start, [i] * len(collab_start), color='black', marker=collab_marker, s=200,zorder=3)

# Create a custom legend
handles = []
labels = []

# Add a handle for each activity type
for action in activity_colors.keys():
    handles.append(plt.Line2D([0], [0], color=activity_colors[action], linewidth=2.5))
    labels.append(action)

# Add a handle for the collaboration start marker
handles.append(plt.Line2D([0], [0], color='black', marker=collab_marker, linestyle='None', markersize=16))
labels.append('collaboration start')

#ax.legend(handles, labels)
ax.legend(handles, labels, fancybox=True, shadow=True, ncol=2, prop={'size': 20})


ax.grid(axis='x')
# Set labels and title
ax.set_xlabel('Time [s]', fontsize=20, fontweight='bold')
ax.set_ylabel('Agent', fontsize=20, fontweight='bold')
ax.set_yticks(range(len(agents)))
ax.set_yticklabels([f'{agent[1:-1]}$_{{{agent[-1]}}}$' for agent in agents], fontsize=18)
ax.set_xticks(np.arange(0, plot_limit+ 1, 25))
ax.tick_params(axis='x', labelsize=16)
ax.set_title('Activity Plot by Agent', fontsize=32, fontweight='bold', pad=15)
plt.subplots_adjust(left=0.06, right=0.99, top=0.95, bottom=0.05)
# set limit of plot
ax.set_xlim(-5, plot_limit+5)
ax.set_ylim(-0.5, 4.5)

plt.show()