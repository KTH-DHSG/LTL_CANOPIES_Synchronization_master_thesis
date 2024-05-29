import matplotlib.pyplot as plt
import yaml

# Load the first YAML file
file_path1 = '/home/davide/git_thesis/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/synchronization_experiments/config/obstacles.yaml'
with open(file_path1, 'r') as file:
    data1 = yaml.safe_load(file)

# Load the second YAML file
file_path2 = '/home/davide/git_thesis/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/synchronization_experiments/config/rosie2_lab.yaml'
with open(file_path2, 'r') as file:
    data2 = yaml.safe_load(file)
# Extract the regions from the data
regions1 = {key: value for key, value in data1.items()}
print(regions1)
# Extract the regions from the second file considering the nested structure
regions2_nested = data2.get('motion', {}).get('regions', {})
regions2 = {key: [value['pose'][0], value['pose'][1], value['pose'][2]] for key, value in regions2_nested.items()}
print(regions2)
# Create a plot
plt.figure(figsize=(10, 10))
ax = plt.gca()

# Function to plot regions
def plot_regions(region_data, color, label_prefix):
    for key, (x, y, radius) in region_data.items():
        circle = plt.Circle((x, y), radius, color=color, alpha=0.5, label=f"{label_prefix}_{key}" if f"{label_prefix}_{key}" not in [text.get_text() for text in ax.get_legend().get_texts()] else "")
        ax.add_patch(circle)
        plt.text(x, y, key, fontsize=9, ha='right', color=color)

# Plot regions from the first file in blue
plot_regions(regions1, 'blue', 'obstacle')

# Plot regions from the second file in red
plot_regions(regions2, 'red', 'region')

# Set the limits and labels
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_aspect('equal', 'box')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Regions Plot with Obstacles')
plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
plt.grid(True)

# Save the plot as an image file
plot_file_path = '/mnt/data/regions_plot.png'
plt.savefig(plot_file_path)
plt.close()

plot_file_path
