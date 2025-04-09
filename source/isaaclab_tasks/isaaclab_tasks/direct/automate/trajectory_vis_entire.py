import json
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Specify the JSON file path
file_path = 'source/isaaclab_tasks/isaaclab_tasks/direct/automate/data/asset_00163_disassembly_traj.json'

# Extract the asset number from the path using the pattern "asset_number_"
match = re.search(r"asset_(\d+)_", file_path)
if match:
    asset_num = str(int(match.group(1)))  # Convert to integer to remove leading zeros and then back to string
else:
    asset_num = "unknown"

# Load JSON data from file
with open(file_path, 'r') as f:
    data = json.load(f)

# Create a 2x2 grid of 3D subplots (four panels)
fig, axes = plt.subplots(2, 2, subplot_kw={'projection': '3d'}, figsize=(12, 10))
axes = axes.flatten()  # Flatten the array of axes into a list

colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
num_subplots = len(axes)

# Assign each trajectory to one of the four subplots sequentially (using modulo operation)
for idx, traj in enumerate(data):
    # Select the subplot for the current trajectory
    ax = axes[idx % num_subplots]
    points = traj["fingertip_midpoint_pos"]
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    z = [p[2] for p in points]
    
    ax.plot(x, y, z, color=colors[idx % len(colors)], marker='o', 
            label=f"Trajectory {idx+1}", markersize=1)

# Add axis labels, titles, and legends to each subplot
for i, ax in enumerate(axes):
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # Set the title mapping with the asset number and trajectory number (e.g., "163_disassembly_trajectory_1")
    ax.set_title(f'{asset_num}_disassembly_trajectory_{i+1}')
    ax.legend(loc='upper left', fontsize='small', ncol=2)

plt.tight_layout()
plt.show()
