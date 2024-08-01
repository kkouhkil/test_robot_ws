import pybullet as p
import pybullet_data

import time as t
import numpy as np

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)

p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# load assets
robot_arm_1 = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase = True)
obj_of_focus = robot_arm_1

# number of joints
num_of_joints = p.getNumJoints(robot_arm_1)
print(f"num_of_joints = {num_of_joints}")

joint_lower_limit_vec = []
joint_upper_limit_vec = []

joint_index_vec = []
current_joint_value_vec = []
current_link_value_vec = []

current_end_eff_pos = []
current_end_eff_ori = []

# joint type and limit
for i in range (num_of_joints):
    joint_type = p.getJointInfo(robot_arm_1, i)
    if joint_type[2] == 0:

        joint_index_vec.append(i)
        current_joint_value_vec.append(0)
        current_link_value_vec.append(0)

        joint_lower_limit = joint_type[8]
        joint_lower_limit_vec.append(joint_lower_limit)

        joint_upper_limit = joint_type[9]
        joint_upper_limit_vec.append(joint_upper_limit)

    print(joint_type)
 
for i in range (len(joint_lower_limit_vec)):
    print(f"joint[{i}]_lower_limit = {joint_lower_limit_vec[i]}, joint[{i}]_upper_limit = {joint_upper_limit_vec[i]}")


# arm motion generation
desired_joints_value = [0] * len(joint_lower_limit_vec)

for step in range(250):

    focus_position, _ = p.getBasePositionAndOrientation(robot_arm_1)
    p.resetDebugVisualizerCamera(cameraDistance = 2, cameraYaw = 0, cameraPitch = -30, cameraTargetPosition = focus_position)
    p.stepSimulation()

    for i in range (len(joint_lower_limit_vec)):
        desired_joints_value[i] = np.random.uniform(joint_lower_limit_vec[i], joint_upper_limit_vec[i])    
    p.setJointMotorControlArray(robot_arm_1, joint_index_vec, p.POSITION_CONTROL, targetPositions = desired_joints_value)    

    current_joint_value_vec = p.getJointStates(robot_arm_1, joint_index_vec)
    current_link_value_vec = p.getLinkStates(robot_arm_1, joint_index_vec)

    current_end_eff_pos = current_link_value_vec[len(joint_lower_limit_vec) - 1][0]
    current_end_eff_ori = current_link_value_vec[len(joint_lower_limit_vec) - 1][1]

    # print(current_end_eff_pos, current_end_eff_ori)

    p.stepSimulation()
    t.sleep(0.25)
    














