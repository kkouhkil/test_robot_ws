import pybullet as p
import pybullet_data

import time as t
import numpy as np
import math

from collections import namedtuple

# setting up the env properties
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# load assets

# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240109_Alex_noHands.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)
# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240109_Alex_nubHands.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)

# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240109_Alex_PsyonicHands.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)

# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240109_Alex_SakeHands.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)

alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240516_Alex_TestStand_FixedHead.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)
# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240516_Alex_TestStand_FixedHead_PsyonicHands.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)

# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240614_Alex_TestStand_FixedHead_BarrettDumbbell.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)
# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240619_Alex_TestStand_FixedHead_LEFT_Barett_RIGHT_Psyonic.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)
# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240619_Alex_TestStand_FixedHead_LEFT_Psyonic_RIGHT_Barett.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)
# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240619_Alex_TestStand_FixedHead_NoHands.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)
# alex_robot = p.loadURDF("/home/keyhan/Documents/boardwalk_robotics/alex-robot-models/alex_description/urdf/20240619_Alex_TestStand_FixedHead_PsyonicHands.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)
obj_of_focus = alex_robot

# number of joints
num_of_joints = p.getNumJoints(alex_robot)
print(f"\nnum_of_joints = {num_of_joints}\n")

# arm motion generation modes
# select 1: random robot arm motion generation
# select 2: desired target robot arm motion generation 
mode_select = 1

# global variables definition
joint_type_global = (0) * num_of_joints

# robot arms - lower/upper limit
left_arm_joint_lower_limit_vec = []
left_arm_joint_upper_limit_vec = []

right_arm_joint_lower_limit_vec = []
right_arm_joint_upper_limit_vec = []

# robot arms - joint index
left_arm_joint_index_vec = []
right_arm_joint_index_vec = []

# robot arms - current joint value vectors
left_arm_current_joint_value_vec = []
right_arm_current_joint_value_vec = []

# robot arms - desired joint value vectors
left_arm_desired_joint_value_vec = []
right_arm_desired_joint_value_vec = []

# robot arms - current link vec
left_arm_current_link_value_vec = []
right_arm_current_link_value_vec = []

# robot arms - current end-effector position
left_arm_current_end_eff_pos = []
right_arm_current_end_eff_pos = []

# robot arms - current end-effector orientation
left_arm_current_end_eff_ori = []
right_arm_current_end_eff_ori = []

# robot arms - fisrt joint index
left_arm_joint_0_idx = 0
right_arm_joint_0_idx = 0

# robot arms - first joint index vector
robot_arm_joint_0_idx = []

gt0_global = t.time()
gt_global = 0

# environment camera visualaizer - function
def env_camera_visualizer():
    focus_position, _ = p.getBasePositionAndOrientation(alex_robot)
    p.resetDebugVisualizerCamera(cameraDistance = 2.25, cameraYaw = 90, cameraPitch = -30, cameraTargetPosition = focus_position)

def interactive_env_creation():

    # Define the size of the box
    box_size = [0.20, 0.20, 1]  # half extents in x, y, z

    # Create a visual shape for the box
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=box_size,
        rgbaColor=[0.75, 0.75, 0.75, 1],  # red color
    )

    # Create a collision shape for the box
    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=box_size,
    )

    # Define the position and orientation of the box
    start_pos = [0, 0, 0]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    # Create a multi-body with the visual and collision shape
    box_id = p.createMultiBody(
        baseMass=0,  # mass of the box
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=start_pos,
        baseOrientation=start_orientation,
    )

# robot joint index finder - function 
def robot_joint_idx_finder(num_of_joints):

    for i in range (num_of_joints):

        joint_type = p.getJointInfo(alex_robot, i)

        joint_type = list(joint_type)
        joint_type[1] = joint_type[1].decode("utf-8")

        joint_type = tuple(joint_type)

        if joint_type[1] == 'LeftShoulderPitch':
            left_arm_joint_0_idx = i
            robot_arm_joint_0_idx.append(left_arm_joint_0_idx)

        if joint_type[1] == 'RightShoulderPitch':
            right_arm_joint_0_idx = i
            robot_arm_joint_0_idx.append(right_arm_joint_0_idx)

# robot joint limit finder - function
def robot_joint_type_limit_finder(num_of_joints):

    # joint type and limit
    for i in range (num_of_joints):
    
        joint_type = p.getJointInfo(alex_robot, i)
    
        if i >= robot_arm_joint_0_idx[0] and i < robot_arm_joint_0_idx[0] + 7:
            left_arm_joint_index_vec.append(i)
            left_arm_current_joint_value_vec.append(0)
            left_arm_current_link_value_vec.append(0)
    
            left_arm_joint_lower_limit = joint_type[8]
            left_arm_joint_lower_limit_vec.append(left_arm_joint_lower_limit)
    
            left_arm_joint_upper_limit = joint_type[9]
            left_arm_joint_upper_limit_vec.append(left_arm_joint_upper_limit)
    
        if i >= robot_arm_joint_0_idx[1] and i < robot_arm_joint_0_idx[1] + 7:    
            right_arm_joint_index_vec.append(i)
            right_arm_current_joint_value_vec.append(0)
            right_arm_current_link_value_vec.append(0)
    
            right_arm_joint_lower_limit = joint_type[8]
            right_arm_joint_lower_limit_vec.append(right_arm_joint_lower_limit)
    
            right_arm_joint_upper_limit = joint_type[9]
            right_arm_joint_upper_limit_vec.append(right_arm_joint_upper_limit)
    
        print(joint_type)

def robot_motion_generation():

    #  arm motion generation
    left_arm_desired_joints_value = [0] * len(left_arm_joint_lower_limit_vec)
    right_arm_desired_joints_value = [0] * len(right_arm_joint_lower_limit_vec)

    left_initial_arm_position = [0.25, 0.35, 1.4]
    right_initial_arm_position = [0.25, -0.35, 1.4]

    left_des_arm_position = [0.25, 0.35, 1.4]
    right_des_arm_position = [0.25, -0.35, 1.4]

    left_arm_des_orientation = p.getQuaternionFromEuler([-math.pi/2, 0, 0])
    right_arm_des_orientation = p.getQuaternionFromEuler([math.pi/2, 0, 0])

    left_arm_des_pose = [left_des_arm_position, left_arm_des_orientation]
    right_arm_des_pose = [right_des_arm_position, right_arm_des_orientation]

    print(f"\nleft_arm_des_pose = {left_arm_des_pose}\nright_arm_des_pose = {right_arm_des_pose}\n")

    while(1):

        gt_global = t.time() - gt0_global

        mode_select = -1
        if gt_global >= 5:
            mode_select = 1
        if gt_global >= 15:
            mode_select = 2
        if gt_global >= 45:
            mode_select = 1

        if mode_select == 0:

            # random motion generation - arm test
            for i in range (len(left_arm_joint_lower_limit_vec)):
                left_arm_desired_joints_value[i] = np.random.uniform(left_arm_joint_lower_limit_vec[i], left_arm_joint_upper_limit_vec[i])    

            for i in range (len(right_arm_joint_lower_limit_vec)):
                right_arm_desired_joints_value[i] = np.random.uniform(right_arm_joint_lower_limit_vec[i], right_arm_joint_upper_limit_vec[i])   

            p.setJointMotorControlArray(alex_robot, left_arm_joint_index_vec, p.POSITION_CONTROL, targetPositions = left_arm_desired_joints_value[0:7])  
            p.setJointMotorControlArray(alex_robot, right_arm_joint_index_vec, p.POSITION_CONTROL, targetPositions = right_arm_desired_joints_value[0:7])  

        if mode_select == 1:

            # desired arm pose - inverse kinematics method
            left_arm_desired_joints_value = p.calculateInverseKinematics(alex_robot, left_arm_joint_index_vec[-1], left_initial_arm_position, left_arm_des_pose[1])
            right_arm_desired_joints_value = p.calculateInverseKinematics(alex_robot, right_arm_joint_index_vec[-1], right_initial_arm_position, right_arm_des_pose[1])

            p.setJointMotorControlArray(alex_robot, left_arm_joint_index_vec, p.POSITION_CONTROL, targetPositions = left_arm_desired_joints_value[0:7])  
            p.setJointMotorControlArray(alex_robot, right_arm_joint_index_vec, p.POSITION_CONTROL, targetPositions = right_arm_desired_joints_value[7:14])  

        if mode_select == 2:
            
            # left arm - trajectory
            left_des_arm_position[0] = left_des_arm_position[0] + 10 * 10**(-3) * math.sin(0.5 * gt_global)

            # right arm - trajectory
            right_des_arm_position[1] = right_des_arm_position[1] - 5 * 10**(-3) * math.sin(0.5 * gt_global)
            right_des_arm_position[2] = right_des_arm_position[2] - 5 * 10**(-3) * math.cos(0.5 * gt_global)

            # desired arm pose - inverse kinematics method
            left_arm_desired_joints_value = p.calculateInverseKinematics(alex_robot, left_arm_joint_index_vec[-1], left_arm_des_pose[0], left_arm_des_pose[1])
            right_arm_desired_joints_value = p.calculateInverseKinematics(alex_robot, right_arm_joint_index_vec[-1], right_arm_des_pose[0], right_arm_des_pose[1])

            p.setJointMotorControlArray(alex_robot, left_arm_joint_index_vec, p.POSITION_CONTROL, targetPositions = left_arm_desired_joints_value[0:7])  
            p.setJointMotorControlArray(alex_robot, right_arm_joint_index_vec, p.POSITION_CONTROL, targetPositions = right_arm_desired_joints_value[7:14])  

        left_arm_current_joint_value_vec = p.getJointStates(alex_robot, left_arm_joint_index_vec)
        left_arm_current_link_value_vec = p.getLinkStates(alex_robot, left_arm_joint_index_vec)

        right_arm_current_joint_value_vec = p.getJointStates(alex_robot, right_arm_joint_index_vec)
        right_arm_current_link_value_vec = p.getLinkStates(alex_robot, right_arm_joint_index_vec)

        left_arm_current_end_eff_pos = left_arm_current_link_value_vec[len(left_arm_joint_lower_limit_vec) - 1][0]
        left_arm_current_end_eff_ori = left_arm_current_link_value_vec[len(left_arm_joint_lower_limit_vec) - 1][1]

        right_arm_current_end_eff_pos = right_arm_current_link_value_vec[len(right_arm_joint_lower_limit_vec) - 1][0]
        right_arm_current_end_eff_ori = right_arm_current_link_value_vec[len(right_arm_joint_lower_limit_vec) - 1][1]

        p.stepSimulation()
        t.sleep(0.1)

        print(f"\ntime = {gt_global}\n")

        # print(left_arm_desired_joints_value[:])
        # print(right_arm_desired_joints_value[:])

        print(f"left_arm_cur_end_eff_pos: {left_arm_current_end_eff_pos}\nleft_arm_cur_end_eff_ori: {left_arm_current_end_eff_ori}\n")
        print(f"right_arm_cur_end_eff_pos: {right_arm_current_end_eff_pos}\nright_arm_cur_end_eff_ori: {right_arm_current_end_eff_ori}\n")

# printing function
def print_func():
    print(f"\nleft_arm_joint_0_idx = {robot_arm_joint_0_idx[0]}\nright_arm_joint_0_idx = {robot_arm_joint_0_idx[1]}")

    print(f"\nleft_arm_joint_idx = {left_arm_joint_index_vec}")
    print(f"right_arm_joint_idx = {right_arm_joint_index_vec}")

    print("\n")

    for i in range (len(left_arm_joint_lower_limit_vec)):
        print(f"left_arm_joint[{i}]_lower_limit = {left_arm_joint_lower_limit_vec[i]} \t left_arm_joint[{i}]_upper_limit = {left_arm_joint_upper_limit_vec[i]}")

    print("\n")

    for i in range (len(right_arm_joint_lower_limit_vec)):
        print(f"right_arm_joint[{i}]_lower_limit = {right_arm_joint_lower_limit_vec[i]} \t right_arm_joint[{i}]_upper_limit = {right_arm_joint_upper_limit_vec[i]}")

if __name__ == "__main__":

    env_camera_visualizer()
    interactive_env_creation()

    robot_joint_idx_finder(num_of_joints)
    robot_joint_type_limit_finder(num_of_joints)

    print_func()
    
    robot_motion_generation()
   
