import pybullet as p
import pybullet_data
from time import sleep
import math
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = p.loadURDF("plane.urdf", [0, 0, 0])

p.setGravity(0, 0, 0)
p.addUserDebugParameter(" Open",1,0,1)
p.addUserDebugParameter(" Close",1,0,1)
p.addUserDebugParameter(" Gravity",1,0,1)
p.addUserDebugParameter(" Next Position",1,0,1)
p.addUserDebugParameter(" Previous Position",1,0,1)
p.addUserDebugParameter(" Add Noise",1,0,1)
p.addUserDebugParameter(" Remove Noise",1,0,1)

Open_param = p.readUserDebugParameter(0)
Close_param = p.readUserDebugParameter(1)
gravity = p.readUserDebugParameter(2)
next_position_count = p.readUserDebugParameter(3)
previous_position_count = p.readUserDebugParameter(4)
noise_count = p.readUserDebugParameter(5)
remove_noise_count = p.readUserDebugParameter(6)

gravity_state = False
position_counter = 0

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

def get_gripper_position(gripper_length, object_position, n_samples):
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

gripper_length = 0.3
item_position = [0, 0, 0.2]

noise = [0,0,0]

gripper_positions = get_gripper_position(gripper_length, item_position, 1000)

gripper_class = gripper(p.loadURDF("pr2_gripper.urdf", gripper_positions[position_counter],baseOrientation=compute_orientation_towards_target(gripper_positions[position_counter], item_position),useFixedBase=True)) 

while (1):
    p.stepSimulation()
    gripper_class.open_gripper()
    if gripper_class.check_if_gripper_is_open() == True:
        #print("Gripper is open")
        break

item = p.loadURDF("cube_small.urdf", item_position,globalScaling=1)


while (1):
    p.stepSimulation()
    #print(p.readUserDebugParameter(1))
    #print(Close_param)
    #print(gripper_class.get_gripper_state())
    #print(f'{p.readUserDebugParameter(0)}' + " " + f'{p.readUserDebugParameter(1)}')
    if p.readUserDebugParameter(2) > gravity:
        if gravity_state == True:
            p.setGravity(0, 0, 0)
            gravity_state = False
            print("Gravity is off")
        else:
            p.setGravity(0, 0, -9.8)
            gravity_state = True
            print("Gravity is on")
        gravity = p.readUserDebugParameter(2)


    if p.readUserDebugParameter(1) > Close_param:
        gripper_class.close_gripper()
        #if gripper_class.check_if_gripper_is_close() == True:
            #print("Gripper is close")
        Close_param = p.readUserDebugParameter(1)
    
    if p.readUserDebugParameter(0) > Open_param:
        gripper_class.open_gripper()
        #if gripper_class.check_if_gripper_is_open() == True:
            #print("Gripper is open")
        Open_param = p.readUserDebugParameter(0)

    if p.readUserDebugParameter(3) > next_position_count:
        #print("Next Position")
        #print(next_position_count)
        #print(position_counter)

        position_counter += 1
        if position_counter >= gripper_positions.shape[0]:
            position_counter = gripper_positions.shape[0] - 1
        
        p.resetBasePositionAndOrientation(gripper_class.gripper, gripper_positions[position_counter] + noise, compute_orientation_towards_target(gripper_positions[position_counter], item_position))

        while (1):
            p.stepSimulation()
            gripper_class.open_gripper()
            if gripper_class.check_if_gripper_is_open() == True:
                #print("Gripper is open")
                break
        
        p.resetBaseVelocity(item,linearVelocity=[0, 0, 0],angularVelocity=[0, 0, 0])
        p.resetBasePositionAndOrientation(item, item_position, [0, 0, 0, 1])
        
        p.setGravity(0, 0, 0)
        gravity_state = False

        next_position_count = p.readUserDebugParameter(3)
    
    if p.readUserDebugParameter(4) > previous_position_count:
        #print("Previous Position")
        #print(previous_position_count)
        #print(position_counter)

        position_counter -= 1
        if position_counter < 0:
            position_counter = 0

        p.resetBasePositionAndOrientation(gripper_class.gripper, gripper_positions[position_counter] + noise, compute_orientation_towards_target(gripper_positions[position_counter], item_position))

        while (1):
            p.stepSimulation()
            gripper_class.open_gripper()
            if gripper_class.check_if_gripper_is_open() == True:
                #print("Gripper is open")
                break

        p.resetBaseVelocity(item,linearVelocity=[0, 0, 0],angularVelocity=[0, 0, 0])
        p.resetBasePositionAndOrientation(item, item_position, [0, 0, 0, 1])

        p.setGravity(0, 0, 0)
        gravity_state = False

        previous_position_count = p.readUserDebugParameter(4)

    if p.readUserDebugParameter(5) > noise_count:
        print("Add Noise")
        noise = np.random.normal(0, 0.001, 3)
        print(noise)
        noise_count = p.readUserDebugParameter(5)

    if p.readUserDebugParameter(6) > remove_noise_count:
        print("Remove Noise")
        noise = [0,0,0]
        print(noise)
        remove_noise_count = p.readUserDebugParameter(6)
    #print(gripper_class.get_joint_positions())
    #print(gripper_class.check_if_gripper_is_close())
    contact_points = p.getContactPoints(bodyA=item, bodyB=plane)
    if len(contact_points) > 0:
        print("Item is on the plane")
    sleep(0.05)