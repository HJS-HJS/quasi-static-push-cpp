import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

import numpy as np

# Reload both original and new PyBullet files to compare trajectories before and after tuning
df_mine = pd.read_csv("target_positions_mine.csv")
df_mujoco = pd.read_csv("target_positions_mujoco.csv")
df_pybullet_new = pd.read_csv("target_positions_pybullet.csv")  # tuned


# Reshape to (steps, objects, 3)
positions_mine = df_mine.drop(columns=["step"]).values.reshape(len(df_mine), -1, 3)
positions_mujoco = df_mujoco.drop(columns=["step"]).values.reshape(len(df_mujoco), -1, 3)
positions_pybullet = df_pybullet_new.drop(columns=["step"]).values.reshape(len(df_pybullet_new), -1, 3)

# Determine common number of objects
common_objects = min(positions_mine.shape[1], positions_mujoco.shape[1], positions_pybullet.shape[1])

# Plot comparison before and after tuning
fig, axs = plt.subplots(2, 3, figsize=(18, 10))
axs = axs.flatten()

for i in range(common_objects):
    axs[i].plot(positions_mine[:, i, 0], positions_mine[:, i, 1], label="Mine", color="tab:blue", linestyle="-")
    axs[i].plot(positions_mujoco[:, i, 0], positions_mujoco[:, i, 1], label="MuJoCo", color="tab:orange", linestyle="--")
    axs[i].plot(positions_pybullet[:, i, 0], positions_pybullet[:, i, 1], label="PyBullet (Tuned)", color="tab:green", linestyle=":")

    axs[i].set_title(f"Object {i}")
    axs[i].set_xlabel("X [m]")
    axs[i].set_ylabel("Y [m]")
    axs[i].legend(fontsize="x-small")
    axs[i].grid(True)

fig.suptitle("Trajectory Comparison: PyBullet Before vs After Tuning", fontsize=16)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()

obstacle_radius = 0.12

plt.figure(figsize=(12, 8))  # 가로 12인치, 세로 8인치

for i in range(common_objects):
    plt.plot(positions_mine[:, i, 0], positions_mine[:, i, 1], label=f"Mine Obj {i}", color="tab:blue", linestyle="-")
    plt.plot(positions_mujoco[:, i, 0], positions_mujoco[:, i, 1], label=f"MuJoCo Obj {i}", color="tab:orange", linestyle="--")
    plt.plot(positions_pybullet[:, i, 0], positions_pybullet[:, i, 1], label=f"PyBullet Obj {i}", color="tab:green", linestyle=":")
    start_x, start_y = positions_mine[0, i, 0], positions_mine[0, i, 1]
    circle = plt.Circle((start_x, start_y), obstacle_radius, color='gray', fill=False, linestyle='--', linewidth=1)
    plt.gca().add_patch(circle)
    plt.text(start_x, start_y, f"{i}", fontsize=9, ha='center', va='center', color='black')

    start_x, start_y = positions_mine[-1, i, 0], positions_mine[-1, i, 1]
    circle = plt.Circle((start_x, start_y), obstacle_radius, color='tab:blue', fill=False, linestyle='--', linewidth=1)
    plt.gca().add_patch(circle)
    plt.text(start_x, start_y, f"{i}", fontsize=9, ha='center', va='center', color='black')

    start_x, start_y = positions_mujoco[-1, i, 0], positions_mujoco[-1, i, 1]
    circle = plt.Circle((start_x, start_y), obstacle_radius, color='tab:orange', fill=False, linestyle='--', linewidth=1)
    plt.gca().add_patch(circle)
    plt.text(start_x, start_y, f"{i}", fontsize=9, ha='center', va='center', color='black')

    start_x, start_y = positions_pybullet[-1, i, 0], positions_pybullet[-1, i, 1]
    circle = plt.Circle((start_x, start_y), obstacle_radius, color='tab:green', fill=False, linestyle='--', linewidth=1)
    plt.gca().add_patch(circle)
    plt.text(start_x, start_y, f"{i}", fontsize=9, ha='center', va='center', color='black')

   
plt.title("Trajectory Comparison After PyBullet Tuning")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.grid(True, which='both', linestyle=':', linewidth=0.5)
plt.axis("equal")
plt.legend(ncol=1, fontsize='small')
plt.tight_layout()
plt.show()


# Recalculate per-object error
from scipy.spatial.distance import cdist

def compute_min_distances(traj_ref, traj_target):
    errors = []
    for pos_ref in traj_ref:
        dists = cdist(traj_target, [pos_ref])
        errors.append(np.min(dists))
    return np.mean(errors)

# Compute errors
object_errors = []
for i in range(common_objects):
    pos_ref = positions_mine[:, i, :2]
    pos_mujoco = positions_mujoco[:, i, :2]
    pos_pybullet = positions_pybullet[:, i, :2]

    mujoco_error = compute_min_distances(pos_ref, pos_mujoco)
    pybullet_error = compute_min_distances(pos_ref, pos_pybullet)

    object_errors.append((i, mujoco_error * 1000, pybullet_error * 1000))  # in mm

# Prepare DataFrame
error_df = pd.DataFrame(object_errors, columns=["Object", "Error_vs_MuJoCo (mm)", "Error_vs_PyBullet (mm)"])
print(error_df)