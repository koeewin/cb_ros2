import numpy as np
from transform import body_to_world, Tz, Rz
deg2rad = np.deg2rad
rad2deg = np.rad2deg


def pretty(v, nd=6):
    return np.array2string(np.array(v), precision=nd, suppress_small=False)


currentPose = np.array([3.0, 1.5, deg2rad(30.0)])   # [3, 1.5, 30°]
lastPose    = np.array([1.0, 1.0, deg2rad(0.0)])    # [1, 1.0, 0°]

print("=== Inputs ===")
print("currentPose [x, y, yaw]:", pretty([currentPose[0], currentPose[1], rad2deg(currentPose[2])]), "(m, m, deg)")
print("lastPose    [x, y, yaw]:", pretty([lastPose[0], lastPose[1], rad2deg(lastPose[2])]), "(m, m, deg)")
print()


# Construction of the path storage
current_position = currentPose[:2] # current position (x,y) of the robot in the world frame
last_position = lastPose[:2]       # last position (x,y) of the robot in the world frame 
print("1) current_position (world):", pretty(current_position))
print("   last_position    (world):", pretty(last_position))
print()



current_position_last = - np.linalg.inv(Rz(currentPose[2])) @ np.append((current_position - last_position), 1) 
print("2) current_position_last (current frame):", pretty(current_position_last[:2]))

current_pose_last = np.append(current_position_last[:2], lastPose[2] - currentPose[2])

print("3) current_pose_last (current frame):", pretty(current_pose_last))

current_T_last = Tz(current_pose_last[2], current_pose_last)
print(pretty(current_T_last))




T_world_current = Tz(currentPose[2], current_position)
T_world_last    = Tz(lastPose[2], last_position)

print("T_world_current:\n", T_world_current)
print("\nT_world_last:\n", T_world_last)

T_current_last = np.linalg.inv(T_world_current) @ T_world_last

print("\nT_current_last:\n", T_current_last)


