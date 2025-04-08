import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load trajectory data from the JSON file.
# Please update the JSON file path accordingly!
with open('source/isaaclab_tasks/isaaclab_tasks/direct/automate/data/asset_00004_disassembly_traj.json', 'r') as f:
    data = json.load(f)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot multiple trajectories with different colors.
colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
for idx, traj in enumerate(data):
    points = traj["fingertip_midpoint_pos"]
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    z = [p[2] for p in points]
    ax.plot(x, y, z, color=colors[idx % len(colors)], marker='o', label=f"Trajectory {idx+1}", markersize=1)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Multiple Trajectories')
ax.legend(
    loc='upper left',   # Legend location
    fontsize='small',   # Legend font size (e.g., 'small', 'medium', 'large', or a numeric value)
    ncol=2              # Display legend in 2 columns
)

plt.show()
