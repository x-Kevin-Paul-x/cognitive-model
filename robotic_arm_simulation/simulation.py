import pybullet as p
import pybullet_data
import time
import math
from PIL import Image
import os
import numpy as np

def setup_simulation():
    """Sets up the PyBullet simulation environment."""
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    return physicsClient

def load_assets(base_path):
    """Loads the robot arm and the object to be picked."""
    arm_start_pos = [0, 0, 0]
    arm_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF(os.path.join(base_path, "arm.urdf"), arm_start_pos, arm_start_orientation, useFixedBase=1)

    object_start_pos = [0.4, 0.2, 0.025]
    object_id = p.loadURDF("cube.urdf", object_start_pos, globalScaling=0.05)

    return robot_id, object_id

def get_joint_info(robot_id):
    """Gets information about the robot's joints."""
    num_joints = p.getNumJoints(robot_id)
    joint_info = {}
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_info[info[1].decode('utf-8')] = {
            'index': info[0],
            'type': info[2],
            'lower_limit': info[8],
            'upper_limit': info[9],
        }
    return joint_info

def get_end_effector_link_index(robot_id):
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        if info[12].decode('utf-8') == 'gripper_base_link':
            return i
    return -1

def move_to(robot_id, end_effector_index, target_pos, target_orn=None):
    """Moves the end-effector to a target position and orientation."""
    if target_orn is None:
        target_orn = p.getQuaternionFromEuler([0, math.pi / 2, 0])

    all_joints = get_joint_info(robot_id)
    arm_joint_names = [name for name, j in all_joints.items() if j['type'] == p.JOINT_REVOLUTE]

    joint_poses = p.calculateInverseKinematics(
        robot_id, end_effector_index, target_pos, target_orn,
        maxNumIterations=200, residualThreshold=.01
    )

    for i, name in enumerate(arm_joint_names):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=all_joints[name]['index'],
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_poses[i],
            force=500,
        )

    for _ in range(240):
        p.stepSimulation()
        time.sleep(1./240.)

def set_gripper(robot_id, open_gripper):
    """Opens or closes the gripper."""
    all_joints = get_joint_info(robot_id)
    left_finger_joint_index = all_joints['gripper_base_to_left_finger']['index']
    right_finger_joint_index = all_joints['gripper_base_to_right_finger']['index']

    target_pos = 0.0 if open_gripper else 0.03

    p.setJointMotorControl2(robot_id, left_finger_joint_index, p.POSITION_CONTROL, targetPosition=target_pos, force=100)
    p.setJointMotorControl2(robot_id, right_finger_joint_index, p.POSITION_CONTROL, targetPosition=target_pos, force=100)

    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def capture_screenshot(filename, width=1280, height=720):
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.2, 0.2, 0.3],
        distance=1.0,
        yaw=45,
        pitch=-30,
        roll=0,
        upAxisIndex=2
    )
    proj_matrix = p.computeProjectionMatrixFOV(
        fov=60,
        aspect=float(width)/height,
        nearVal=0.1,
        farVal=100.0
    )
    (_, _, rgba_img, _, _) = p.getCameraImage(
        width, height, view_matrix, proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    np_img_arr = np.array(rgba_img, dtype=np.uint8).reshape((height, width, 4))
    img = Image.fromarray(np_img_arr)
    img.save(filename)
    print(f"Screenshot saved to {filename}")

def main():
    """Main function to run the pick-and-place simulation."""
    client = setup_simulation()

    # Clean up old screenshots before starting
    if not os.path.exists("screenshots"):
        os.makedirs("screenshots")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    robot_id, object_id = load_assets(script_dir)

    end_effector_index = get_end_effector_link_index(robot_id)
    if end_effector_index == -1:
        print("Could not find end-effector link 'gripper_base_link'.")
        p.disconnect()
        return

    try:
        set_gripper(robot_id, open_gripper=True)
        obj_pos, obj_orn = p.getBasePositionAndOrientation(object_id)
        above_obj_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.2]

        # 1. Approach
        move_to(robot_id, end_effector_index, above_obj_pos)
        capture_screenshot("screenshots/1_approach.png")

        # 2. Grasp
        move_to(robot_id, end_effector_index, [obj_pos[0], obj_pos[1], obj_pos[2] + 0.05])
        set_gripper(robot_id, open_gripper=False)
        constraint_id = p.createConstraint(
            parentBodyUniqueId=robot_id,
            parentLinkIndex=end_effector_index,
            childBodyUniqueId=object_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0.1],
            childFramePosition=[0, 0, 0]
        )
        capture_screenshot("screenshots/2_grasp.png")

        # 3. Lift
        move_to(robot_id, end_effector_index, above_obj_pos)
        capture_screenshot("screenshots/3_lift.png")

        # 4. Move to a new target location and release
        target_pos = [0.0, 0.5, 0.2]
        move_to(robot_id, end_effector_index, target_pos)
        p.removeConstraint(constraint_id)
        set_gripper(robot_id, open_gripper=True)
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./240.)

        # 5. Retract arm
        home_pos = [0.2, 0, 0.5]
        move_to(robot_id, end_effector_index, home_pos)

    except p.error as e:
        print(f"An error occurred during simulation: {e}")
    finally:
        print("Simulation finished. Disconnecting.")
        p.disconnect()

if __name__ == "__main__":
    main()
