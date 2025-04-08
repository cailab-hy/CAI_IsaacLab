import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 3D plotting support

# Example: When the JSON data is saved in a file
with open('source/isaaclab_tasks/isaaclab_tasks/direct/automate/data/asset_00004_disassembly_traj.json', 'r') as f:
    data = json.load(f)

# Data extraction: Here we use the 'fingertip_midpoint_pos' list from the first dictionary
points = data[0]["fingertip_midpoint_pos"]
# You can choose a specific trajectory from 0 to 99.

# Separate the x, y, z coordinates
x = [p[0] for p in points]
y = [p[1] for p in points]
z = [p[2] for p in points]

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, c='b', marker='o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Fingertip Midpoint Positions')

plt.show()
