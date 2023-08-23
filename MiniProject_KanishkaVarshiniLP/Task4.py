import numpy as np
import matplotlib.pyplot as plt

# defining the constants
l1 = 2.0  # length of link 1
l2 = 1.0  # length of link 2

# range of angles as given
q1_range = np.radians(np.arange(35, 146,0.2))  # [35,36,...,145 deg] in radians
q2_range = np.radians(np.arange(35, 146,0.2))  # [35,36,...,145 deg] in radians

# initialize lists of possible end effector positions
workspace_x = []
workspace_y = []

# iterate through joint angle ranges to get possible end effector positions given the range of angles q1 and q2
for q1 in q1_range:
    for q2 in q2_range:
        x = l1 * np.cos(q1) + l2 * np.cos(q2)
        y = l1 * np.sin(q1) + l2 * np.sin(q2)
        workspace_x.append(x)
        workspace_y.append(y)


workspace_x = np.array(workspace_x)
workspace_y = np.array(workspace_y)

# plotting
plt.figure(figsize=(6, 6))
plt.scatter(workspace_x, workspace_y, s=0.1, c='c', marker='.')
plt.xlabel('x')
plt.ylabel('y')
plt.title('workspace of the 2R manipulator')
plt.grid()
plt.show()