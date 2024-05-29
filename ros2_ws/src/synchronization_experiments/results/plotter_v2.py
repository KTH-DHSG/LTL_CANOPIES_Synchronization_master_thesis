import matplotlib.pyplot as plt
import pickle

#Path to the pickle file
file_path = '/home/davide/git_thesis/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/results/data_sim_1.pkl'
# Open the pickle file and load the data
with open(file_path, 'rb') as file:
    data = pickle.load(file)
print(data)
del data['/turtlebot1']
del data['/turtlebot2']
#added by hand because it is easier to do
initial_regions = ['m1', 'h', 'p3', 'p1' ]
# Assuming data is a dictionary with agent names as keys and AgentData as values
agents = list(data.keys())
activity_colors = {'assisting': 'red', 'local': 'blue', 'collaborative': 'green'}
activity_markers = {'assisting': 's', 'local': 'o', 'collaborative': 'd'}
collab_marker = '*'

fig, ax = plt.subplots(figsize=(12, 8))

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
        if not (actions[j].startswith('goto') or actions[j].startswith('none')):
            ax.hlines(i, start_time, end_time, colors=color, linewidth=2.5, label=action if i == 0 and j == 0 else "")
            ax.scatter([start_time, end_time], [i, i], color=color, marker=marker)
            # Add action name above the line
            ax.text((start_time + end_time) / 2, i -0.15, f'{actions[j]}$_{{{last_region}}}$', ha='center', va='bottom', fontsize=13)
        elif actions[j].startswith('goto'):
            last_region = actions[j].split('_')[-1]
    # Plot the last action that ends with the plot range end
    start_time = action_start[-1]
    end_time = 1000 # or any other endpoint
    action = action_types[-1]
    color = activity_colors[action]
    marker = activity_markers[action]
    if not (actions[j].startswith('goto') or actions[j].startswith('none')):
        ax.hlines(i, start_time, end_time, colors=color, linewidth=2.5, label=action if i == 0 and j == 0 else "")
        ax.scatter([start_time, end_time], [i, i], color=color, marker=marker)
        # Add action name above the line
        ax.text((start_time + end_time) / 2, i -0.15, f'{actions[-1]}$_{{{last_region}}}$', ha='center', va='bottom', fontsize=13)
        
    # Plot collaboration start times
    ax.scatter(collab_start, [i] * len(collab_start), color='black', marker=collab_marker, s=100, label='Collab Start' if i == 0 else "")

# Add legend
handles, labels = ax.get_legend_handles_labels()
by_label = dict(zip(labels, handles))
ax.legend(by_label.values(), by_label.keys())

# Set labels and title
ax.set_xlabel('Time (s)')
ax.set_ylabel('Agent')
ax.set_yticks(range(len(agents)))
ax.set_yticklabels(agents)
ax.set_title('Activity Plot by Agent')
# set limit of plot
ax.set_xlim(-10, 500)
ax.set_ylim(-1, 4)

plt.show()