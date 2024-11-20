import pybullet as p
import pybullet_data
from time import sleep
import math

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0])
p.setGravity(0, 0, -10)
p.addUserDebugParameter("Open",1,0,1)
p.addUserDebugParameter("Close",1,0,1)
p.addUserDebugParameter("X",-2,2,0)
p.addUserDebugParameter("Y",-2,2,0)
p.addUserDebugParameter("Z",-2,2,0)

Open_param = p.readUserDebugParameter(0)
Close_param = p.readUserDebugParameter(1)

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


gripper_class = gripper(p.loadURDF("pr2_gripper.urdf", [0, 0, 0.05], useFixedBase=True)) 
item = p.loadURDF("cube_small.urdf", [1, 0, 0.2],globalScaling=1.5)


while (1):
    p.stepSimulation()
    #print(p.readUserDebugParameter(1))
    #print(Close_param)
    #print(gripper_class.get_gripper_state())
    #print(f'{p.readUserDebugParameter(0)}' + " " + f'{p.readUserDebugParameter(1)}')
    p.resetBasePositionAndOrientation(item, [p.readUserDebugParameter(2), p.readUserDebugParameter(3), p.readUserDebugParameter(4)], [0, 0, 0, 1])
    if p.readUserDebugParameter(1) > Close_param and gripper_class.get_gripper_state() == "open":
        gripper_class.close_gripper()
        if gripper_class.check_if_gripper_is_close() == True:
            #print("Gripper is close")
            Close_param = p.readUserDebugParameter(1)
    elif p.readUserDebugParameter(0) > Open_param and gripper_class.get_gripper_state() == "close":
        Close_param = p.readUserDebugParameter(1)
    
    if p.readUserDebugParameter(0) > Open_param and gripper_class.get_gripper_state() == "close":
        gripper_class.open_gripper()
        if gripper_class.check_if_gripper_is_open() == True:
            #print("Gripper is open")
            Open_param = p.readUserDebugParameter(0)
    elif p.readUserDebugParameter(0) > Open_param and gripper_class.get_gripper_state() == "open":
        Open_param = p.readUserDebugParameter(0)
    #print(gripper_class.get_joint_positions())
    #print(gripper_class.check_if_gripper_is_close())
    sleep(0.05)