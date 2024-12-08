import pybullet as p
import pybullet_data
from time import sleep
import math
import numpy as np
import pandas as pd
import time

class gripper:
    def __init__(self, gripper):
        self.gripper = gripper
        self.numjoints = p.getNumJoints(gripper)
        self.target_open = [0.548, 0, 0.548 , 0]
        self.target_close = [0, 0, 0, 0]
        self.gripper_states = "close"
    
    def open_gripper(self):
        for i in range(self.numjoints):
            p.setJointMotorControl2(bodyIndex=self.gripper,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=self.target_open[i],
                                    targetVelocity=0.01,
                                    force=500,
                                    positionGain=0.3,
                                    velocityGain=1)    
        self.gripper_states = "open"

    def close_gripper(self):
        for i in range(self.numjoints):
            p.setJointMotorControl2(bodyIndex=self.gripper,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=self.target_close[i],
                                    targetVelocity=0.01,
                                    force=500,
                                    positionGain=0.3,
                                    velocityGain=1)
        self.gripper_states = "close"

    def get_gripper_state(self):
        return self.gripper_states
    
    def get_joint_positions(self):
        joint_positions = []
        for i in range(self.numjoints):
            joint_positions.append(p.getJointState(self.gripper, i)[0])
        return joint_positions
    
    def check_if_gripper_is_open(self):
        joint_positions = self.get_joint_positions()
        #print(joint_positions)
        if math.isclose(joint_positions[0], self.target_open[0], abs_tol=1e-3) and math.isclose(joint_positions[2], self.target_open[2], abs_tol=1e-3):
            self.gripper_states = "open"
            return True
        else:
            return False
    
    def check_if_gripper_is_close(self):
        joint_positions = self.get_joint_positions()
        #print(joint_positions)
        if  math.isclose(joint_positions[0], self.target_close[0], abs_tol=1e-3) and math.isclose(joint_positions[2], self.target_close[2], abs_tol=1e-3):
            self.gripper_states = "close"
            return True
        else:
            return False




physicsClient = p.connect(p.DIRECT) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())

gravity_state = False
item_position = [0, 0, 0.2]
result = []
scuess_count = 0
fail_count = 0

df = pd.read_csv('Gripper_position.csv')
df = df.reset_index()  # make sure indexes pair with number of rows

for row in range(len(df)):
    p.setGravity(0, 0, 0)
    gripper_class = gripper(p.loadURDF("pr2_gripper.urdf", [df['X Position'][row],df['Y Position'][row],df['Z Position'][row]],baseOrientation=[df['X Quaternion'][row],df['Y Quaternion'][row],df['Z Quaternion'][row],df['W'][row]],useFixedBase=True)) 
    plane = p.loadURDF("plane.urdf", [0, 0, 0])

    while (1):
        p.stepSimulation()
        gripper_class.open_gripper()
        if gripper_class.check_if_gripper_is_open() == True:
            #print("Gripper is open")
            break

    item = p.loadURDF("cube_small.urdf", item_position,globalScaling=1)

    p.setGravity(0, 0, 0)

    for i in range(1000):
        p.stepSimulation()
        gripper_class.close_gripper()

    p.setGravity(0, 0, -9.8)

    gripped = True

    for i in range(1000):
        p.stepSimulation()
        contact_points = p.getContactPoints(bodyA=item, bodyB=plane)
        if len(contact_points) > 0:
            gripped = False
            break

    if gripped:
        print(str(row) + ": Item is gripped")
        scuess_count += 1
        result.append(1)
    else:
        print(str(row) + ": Item is not gripped")
        fail_count += 1
        result.append(0)

    p.resetSimulation()

print(scuess_count)
print(fail_count)
df['result'] = result
df.to_csv("Gripper_simulation_result.csv",index=None)