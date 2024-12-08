import numpy as np
import pandas as pd
import math
import pybullet as p

def generate_gripper_position(gripper_length, object_position, n_samples):
    r = gripper_length  # Radius
    center = np.array(object_position)  # Center of the sphere
    phi = np.random.uniform(0, 2 * np.pi, n_samples)
    cos_theta = np.random.uniform(-1, 1, n_samples)
    theta = np.arccos(cos_theta)

    # Compute Cartesian coordinates
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * cos_theta

    x += center[0]
    y += center[1]
    z += center[2]

    points = np.vstack((x, y, z)).T
    return points

def compute_orientation_towards_target(gripper_pos, target_pos):
    # Compute the direction vector from gripper to target
    direction = [
        target_pos[0] - gripper_pos[0],
        target_pos[1] - gripper_pos[1],
        target_pos[2] - gripper_pos[2]
    ]
    
    # Normalize the direction vector
    length = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
    direction = [d / length for d in direction]
    
    # Compute yaw and pitch angles
    yaw = math.atan2(direction[1], direction[0])
    pitch = -math.asin(direction[2])  # Negative sign depends on coordinate system
    roll = 0  # Assume roll is zero
        
    # Convert to quaternion
    orientation_quaternion = p.getQuaternionFromEuler([roll, pitch, yaw])
    
    return orientation_quaternion

gripper_length = 0.3
item_position = [0, 0, 0.2]
noise = [0,0,0]

number_of_nosie = 10

positions = generate_gripper_position(gripper_length,item_position,1000)

data = []

for i in positions:
    Quaternion = compute_orientation_towards_target(i,item_position)
    data.append([i[0],i[1],i[2],Quaternion[0],Quaternion[1],Quaternion[2],Quaternion[3]])
    for j in range(number_of_nosie):
        noise = np.random.normal(0, 0.001, 3)
        new_position = i + noise
        data.append([new_position[0],new_position[1],new_position[2],Quaternion[0],Quaternion[1],Quaternion[2],Quaternion[3]])
    
df = pd.DataFrame(data,columns=['X Position','Y Position','Z Position','X Quaternion','Y Quaternion','Z Quaternion','W'])
df.to_csv("Gripper_position.csv",index=None)
    
    