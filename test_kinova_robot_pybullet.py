import pybullet as p
import pybullet_data

import time as t
import numpy as np
import math

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# load assets
robot_arm_1 = p.loadURDF("/home/keyhan/Documents/ros_kortex/kortex_description/arms/gen3/7dof/urdf/GEN3-7DOF-VISION_ARM_URDF_V12.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = True)
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

gt0_global = t.time()
gt_global = 0

# environment camera visualaizer - function
def env_camera_visualizer():
    focus_position, _ = p.getBasePositionAndOrientation(robot_arm_1)
    p.resetDebugVisualizerCamera(cameraDistance = 2.0, cameraYaw = 60, cameraPitch = -30, cameraTargetPosition = focus_position)

def interactive_env_creation():

    # Define the size of the box
    box_1_size = [0.20, 0.20, 1]  # half extents in x, y, z

    # Create a visual shape for the box - 1
    visual_shape_1_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=box_1_size,
        rgbaColor=[0.75, 0.75, 0.75, 1],  # gray color
    )

    # Create a collision shape for the box - 1
    collision_shape_1_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=box_1_size,
    )

    # Define the position and orientation of the box - 1
    start_1_pos = [0, 0, 0]
    start_1_orientation = p.getQuaternionFromEuler([0, 0, 0])

    # Create a multi-body with the visual and collision shape
    box_1_id = p.createMultiBody(
        baseMass=0,  # mass of the box
        baseCollisionShapeIndex=collision_shape_1_id,
        baseVisualShapeIndex=visual_shape_1_id,
        basePosition=start_1_pos,
        baseOrientation=start_1_orientation,
    )

    box_2_size = [0.25, 0.5, 1]  # half extents in x, y, z

    # Create a visual shape for the box - 2
    visual_shape_2_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=box_2_size,
        rgbaColor=[0.0, 0.0, 1.0, 1],  # blue color
    )

    # Create a collision shape for the box - 2
    collision_shape_2_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=box_2_size,
    )

    # Define the position and orientation of the box - 1
    start_2_pos = [0.5, 0, 0]
    start_2_orientation = p.getQuaternionFromEuler([0, 0, 0])

    # Create a multi-body with the visual and collision shape
    box_2_id = p.createMultiBody(
        baseMass=0,  # mass of the box
        baseCollisionShapeIndex=collision_shape_2_id,
        baseVisualShapeIndex=visual_shape_2_id,
        basePosition=start_2_pos,
        baseOrientation=start_2_orientation,
    )

    box_3_size = [0.05, 0.05, 0.05]  # half extents in x, y, z

    # Create a visual shape for the box - 2
    visual_shape_3_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=box_3_size,
        rgbaColor=[1.0, 0.0, 0.0, 1],  # blue color
    )

    # Create a collision shape for the box - 2
    collision_shape_3_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=box_3_size,
    )

    # Define the position and orientation of the box - 1
    start_3_pos = [0.5, 0.0, 1.05]
    start_3_orientation = p.getQuaternionFromEuler([0, 0, 0])

    # Create a multi-body with the visual and collision shape
    box_3_id = p.createMultiBody(
        baseMass=1.0,  # mass of the box
        baseCollisionShapeIndex=collision_shape_3_id,
        baseVisualShapeIndex=visual_shape_3_id,
        basePosition=start_3_pos,
        baseOrientation=start_3_orientation,
    )

def main():

    current_joint_value_vec = []
    current_link_value_vec = []

    initial_arm_position = [0.5, 0.0, 1.5]

    initial_arm_ori_roll = 0
    initial_arm_ori_pitch = 0
    initial_arm_ori_yaw = 0

    initial_arm_orientation = p.getQuaternionFromEuler([initial_arm_ori_roll, initial_arm_ori_pitch, initial_arm_ori_yaw])

    desired_arm_position = [0.5, 0.0, 1.05]

    desired_arm_ori_roll = 0
    desired_arm_ori_pitch = 0
    desired_arm_ori_yaw = 0

    desired_arm_orientation = p.getQuaternionFromEuler([desired_arm_ori_roll, desired_arm_ori_pitch, desired_arm_ori_yaw])

    initial_arm_pose = [initial_arm_position, initial_arm_orientation]
    desired_arm_pose = [desired_arm_position, desired_arm_orientation]

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
    
    print("\n")
    for i in range (len(joint_lower_limit_vec)):
        print(f"joint[{i}]_lower_limit = {joint_lower_limit_vec[i]}, joint[{i}]_upper_limit = {joint_upper_limit_vec[i]}")

    print("\n")

    # arm motion generation
    desired_joints_value = [0] * len(joint_lower_limit_vec)

    while(1):

        gt_global = t.time() - gt0_global

        demo_mode = 1
        mode_select = -1

        # static target tracking
        if demo_mode == 1:
            if gt_global >= 10:
                mode_select = 1
                print(f"mode --> {mode_select}")
            if gt_global >= 15:
                mode_select = 2

                desired_arm_position[0] = 0.5
                desired_arm_position[1] = 0.35
                desired_arm_position[2] = 1.05

                print(f"mode --> {mode_select}.1")

            if gt_global >= 20:
                mode_select = 2

                desired_arm_position[0] = 0.5
                desired_arm_position[1] = -0.35
                desired_arm_position[2] = 1.05

                print(f"mode --> {mode_select}.2")

            if gt_global >= 25:
                mode_select = 2

                desired_arm_position[0] = 0.25
                desired_arm_position[1] = -0.15
                desired_arm_position[2] = 1.45

                print(f"mode --> {mode_select}.3")

            if gt_global >= 30:
                mode_select = 2

                desired_arm_position[0] = 0.35
                desired_arm_position[1] = 0.15
                desired_arm_position[2] = 1.25           

                print(f"mode --> {mode_select}.4")

            if gt_global >= 35:
                mode_select = 2

                desired_arm_position[0] = 0.35
                desired_arm_position[1] = 0.35
                desired_arm_position[2] = 1.5

                print(f"mode --> {mode_select}.5")

            if gt_global >= 40:
                mode_select = 2

                desired_arm_position[0] = 0.35
                desired_arm_position[1] = -0.25
                desired_arm_position[2] = 1.35

                print(f"mode --> {mode_select}.6")

            if gt_global >= 45:
                mode_select = 2

                desired_arm_position[0] = 0.5
                desired_arm_position[1] = -0.25
                desired_arm_position[2] = 1.5

                print(f"mode --> {mode_select}.7")

            if gt_global >= 50:
                mode_select = 1
                print(f"mode --> {mode_select}")

        # trajectory tracking
        if demo_mode == 2:
            if gt_global >= 10:
                mode_select = 1

            if gt_global >= 15:
                mode_select = 2
                desired_arm_position[0] = desired_arm_position[0] - 10 * 10**(-3) * math.sin(0.5 * gt_global)
                desired_arm_position[1] = desired_arm_position[1] - 10 * 10**(-3) * math.cos(0.5 * gt_global)

            if gt_global >= 45:
                mode_select = 1

        if mode_select == 0:

            # random arm motion generation
            for i in range (len(joint_lower_limit_vec)):
                desired_joints_value[i] = np.random.uniform(joint_lower_limit_vec[i], joint_upper_limit_vec[i])    
            p.setJointMotorControlArray(robot_arm_1, joint_index_vec, p.POSITION_CONTROL, targetPositions = desired_joints_value)    

        if mode_select == 1:

            # initial arm pose - inverse kinematics method
            initial_arm_desired_joints_value = p.calculateInverseKinematics(robot_arm_1, joint_index_vec[-1], initial_arm_pose[0], initial_arm_pose[1])
            p.setJointMotorControlArray(robot_arm_1, joint_index_vec, p.POSITION_CONTROL, targetPositions = initial_arm_desired_joints_value[0:7])  

        if mode_select == 2:

            # desired arm pose - inverse kinematics method
            desired_arm_desired_joints_value = p.calculateInverseKinematics(robot_arm_1, joint_index_vec[-1], desired_arm_pose[0], desired_arm_pose[1])
            p.setJointMotorControlArray(robot_arm_1, joint_index_vec, p.POSITION_CONTROL, targetPositions = desired_arm_desired_joints_value[0:7])  

        current_joint_value_vec = p.getJointStates(robot_arm_1, joint_index_vec)
        current_link_value_vec = p.getLinkStates(robot_arm_1, joint_index_vec)

        current_end_eff_pos = current_link_value_vec[len(joint_lower_limit_vec) - 1][0]
        current_end_eff_ori = current_link_value_vec[len(joint_lower_limit_vec) - 1][1]

        # print(current_end_eff_pos, current_end_eff_ori)

        p.stepSimulation()
        t.sleep(0.1)

        print(f"time = {gt_global}")

if __name__ == "__main__":

    env_camera_visualizer()
    interactive_env_creation()
    main()

    














