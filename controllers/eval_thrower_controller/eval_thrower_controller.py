"""Evaluation controller - executes trajectories from data files during optimization"""

import sys
import os
import numpy as np
import json
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from controller import Supervisor
from utils.ik import getDesiredRobotCommand, fk

# -----------------------------------------------------------------------------
# Init
# -----------------------------------------------------------------------------
sup = Supervisor()
robot = sup.getFromDef("THROWER")
block = sup.getFromDef("BLOCK")
timestep = int(sup.getBasicTimeStep())

# -----------------------------------------------------------------------------
# UR5e arm joints (6-DoF)
# -----------------------------------------------------------------------------
arm_joint_names = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]
motors = [sup.getDevice(n) for n in arm_joint_names]
for m in motors:
    ps = m.getPositionSensor()
    ps.enable(timestep)

# -----------------------------------------------------------------------------
# Robotiq 2F-85 gripper
# -----------------------------------------------------------------------------
gripper_left = sup.getDevice("robotiq_2f85_thrower::left finger joint")
gripper_right = sup.getDevice("robotiq_2f85_thrower::right finger joint")

if gripper_left and gripper_right:
    for g in (gripper_left, gripper_right):
        g.setVelocity(1.5)
        g.setForce(100.0)
        # Enable position sensors on gripper
        ps = g.getPositionSensor()
        if ps:
            ps.enable(timestep)

def set_gripper(closed):
    """Set gripper state: True=closed, False=open"""
    if gripper_left and gripper_right:
        pos = 0.8 if closed else 0.0
        gripper_left.setPosition(pos)
        gripper_right.setPosition(pos)

def get_gripper_position():
    """Get current gripper finger positions"""
    if gripper_left and gripper_right:
        left_ps = gripper_left.getPositionSensor()
        right_ps = gripper_right.getPositionSensor()
        if left_ps and right_ps:
            return (left_ps.getValue() + right_ps.getValue()) / 2
    return 0.0

def get_joint_positions():
    """Get current arm joint positions"""
    return np.array([m.getPositionSensor().getValue() for m in motors])

# -----------------------------------------------------------------------------
# Load evaluation data
# -----------------------------------------------------------------------------

while True:
    print("[SIM] Starting simulation wait loop for eval data")

    while True:
        time.sleep(1)  # wait a bit for file to be written

        eval_data_path = os.path.join(os.path.dirname(__file__), 'eval_data.json')

        try:
            with open(eval_data_path, 'r') as f:
                # Read first/only line
                eval_data = json.loads(f.read().strip())
            
            if eval_data['run_id'] != last_run_id:
                # Already processed this one
                TRAJECTORY = np.array(eval_data['trajectory'])
                GRIPPER_CLOSE_TIME = eval_data['gripper_close_time']
                GRIPPER_OPEN_TIME = eval_data['gripper_open_time']
                
                print(f"[EVAL] Loaded trajectory: {len(TRAJECTORY)} timesteps")
                print(f"[EVAL] Gripper close: t={GRIPPER_CLOSE_TIME}, open: t={GRIPPER_OPEN_TIME}")

                break  # exit inner wait loop

            if eval_data['run_id'] == 'done training':
                sup.simulationQuit(0)
            
        except FileNotFoundError:
            print(f"[ERROR] No eval data found at {eval_data_path}")
            pass

    print(f"[SIM] Starting evaluation run_id={eval_data['run_id']} train_step={eval_data['train_step']}")

    # Same pickup sequence as learned_thrower_controller
    BLOCK_START_POS = np.concatenate((np.array(block.getPosition()) - np.array(robot.getPosition()), [0,0,0]))
    BASE_POSE = [1.2, -1.2, 1.5, -2.0, -1.57, 1.03]
    MOVE_TIME = 50
    CHECKPOINT_X = -0.67
    TABLE_Z = 0.715

    # Load start throw pose from trajectory
    START_THROW_POSE = TRAJECTORY[0]


    # Tracking variables
    checkpoint_reached = False
    block_dropped = False
    release_velocity = None
    final_distance = 0.0
    block_pos_at_release = None
    landed = False
    block_land_pos = None

    tt = 0
    last_pose = np.array(BASE_POSE)

    # Phase tracking
    start_pose = None
    end_pose = None

    pos_at_passing = None

    while sup.step(timestep) != -1 and not landed:
        # Get block position
        block_pos = np.array(block.getPosition())
        robot_pos = np.array(robot.getPosition())
        arm_pos = fk(last_pose)[:3]
        block_rel_pos = block_pos - robot_pos

        # Execute pickup sequence then trajectory
        if tt < 50:
            # Move to base pose
            joints = BASE_POSE
            gripper = 0
            
        elif tt < 100:
            # Move to block
            joints = getDesiredRobotCommand(0, BLOCK_START_POS, last_pose)
            gripper = 0
            
        elif tt < GRIPPER_CLOSE_TIME:
            # Position over block
            joints = getDesiredRobotCommand(0, BLOCK_START_POS, last_pose)
            gripper = 1
            
        elif tt == GRIPPER_CLOSE_TIME:
            # Close gripper
            joints = getDesiredRobotCommand(0, BLOCK_START_POS, last_pose)
            gripper = 1
            
        elif tt < 150:
            # Lift block
            desired_pose = BLOCK_START_POS.copy()
            desired_pose[2] += 0.1  # lift 10cm
            joints = getDesiredRobotCommand(0, desired_pose, last_pose)
            gripper = 1
            
        elif tt < 150 + MOVE_TIME:
            # Move to throw start position
            if start_pose is None:
                start_pose = last_pose.copy()
                end_pose = START_THROW_POSE
            
            progress = (tt - 150) / MOVE_TIME
            joints = (1 - progress) * start_pose + progress * end_pose
            gripper = 1
            
        elif tt < 150 + MOVE_TIME + len(TRAJECTORY):
            # Execute trajectory
            traj_step = tt - (150 + MOVE_TIME)
            joints = TRAJECTORY[traj_step]
            
            # Handle gripper release
            if tt == GRIPPER_OPEN_TIME and gripper:
                gripper = 0
                
        else:
            # Continue simulation until block lands (passes through the table plane)
            joints = TRAJECTORY[-1]
            gripper = 0
            
            if not checkpoint_reached and block_pos[0] < CHECKPOINT_X:
                checkpoint_reached = True
                pos_at_passing = block_pos.copy()
            
            if not landed and block_pos[2] < TABLE_Z:
                landed = True
                block_land_pos = block_rel_pos.copy()

        # Apply commands
        for j, motor in enumerate(motors):
            motor.setPosition(joints[j])
        set_gripper(gripper == 1)

        # Check if block dropped during pickup/throw
        # Block has dropped if:
        # 1. Gripper should be closed (gripper_closed=True)
        # 2. Block is too far from gripper (>0.15m)
        # 3. Gripper fingers are not actually closed (<0.5, indicating nothing is gripping)
        if tt > 150 + MOVE_TIME and not block_dropped:
            joints_pos = get_joint_positions()
            block_gripper_dist = np.linalg.norm(block_rel_pos - fk(joints_pos)[:3])
            if block_gripper_dist > 0.08:
                block_dropped = True
                release_velocity = block.getVelocity()
                block_pos_at_release = block_rel_pos.copy()
                print(f"[EVAL] Block released at t={tt}")
        last_pose = np.array(joints)
        tt += 1

    # Write results


    eval_results_dir = os.path.join(os.path.dirname(__file__), 'eval_results')
    os.makedirs(eval_results_dir, exist_ok=True)
    result_path = os.path.join(os.path.dirname(__file__), "eval_result.json")
    result_archive_path = os.path.join(eval_results_dir, f'eval_results_{eval_data["run_name"]}.jsonl')

    release_vel_hor = (
            -float(np.linalg.norm(release_velocity[:2])) if release_velocity is not None and release_velocity[0] > 0
            else float(np.linalg.norm(release_velocity[:2])) if release_velocity is not None
            else 0.0
        )

    final_distance = (
        -float(np.linalg.norm(block_land_pos[:2] - block_pos_at_release[:2])) if block_pos_at_release is not None and block_land_pos[0] < block_pos_at_release[0]
        else float(np.linalg.norm(block_land_pos[:2] - block_pos_at_release[:2])) if block_land_pos is not None
        else 0.0
    )

    result = {
        'run_id': eval_data['run_id'],
        'train_step': eval_data['train_step'],
        'success': checkpoint_reached,
        'release_velocity_z': release_velocity[2] if release_velocity is not None else 0.0,
        # Signed horizontal speed: negative when release velocity x-component is positive
        'release_velocity_hor': release_vel_hor,
        'pos_at_cp': pos_at_passing.tolist() if pos_at_passing is not None else [0, 0, 0],
        'final_distance': float(np.linalg.norm(block_land_pos[:2] - block_pos_at_release[:2])) if block_pos_at_release is not None else 0.0,
    }

    with open(result_path, "w") as f:
        f.write(json.dumps(result) + '\n')

    result = {
        'run_id': eval_data['run_id'],
        'train step': eval_data['train_step'],
        "block released": block_dropped,
        "block released time": tt if block_dropped else 'N/A',
        'release velocity z': release_velocity[2] if release_velocity is not None else 0.0,
        # Make horizontal release value negative when vx > 0 per user request
        'release velocity hor': release_vel_hor,
        "success": checkpoint_reached,
        "final distance (from release pos XY to landing pos XY)": (-1 + 2 * (block_land_pos[0] > block_pos_at_release[0])) * float(np.linalg.norm(block_land_pos[:2] - block_pos_at_release[:2])) if block_dropped else 0.0,
        "position at checkpoint (global)": pos_at_passing.tolist() if pos_at_passing is not None else 'Did not reach',
        "position at release (rel. arm base)": block_pos_at_release.tolist() if block_pos_at_release is not None else [0, 0, 0],
        "final arm pos (rel. arm base)": arm_pos.tolist(),
        "final block pos (rel. arm base)": block_rel_pos.tolist(),
    }

    with open(result_archive_path, 'a') as f:
        f.write(json.dumps(result) + '\n')

    print(f"[EVAL] Results: success={result['success']}, velocity={release_velocity[:3]}, displacement={block_land_pos}m")

    print("[SIM] Resetting simulation for next evaluation run")
<<<<<<< HEAD
    # Release large in-memory objects as soon as possible
    # For memory-mapped arrays, we need to explicitly delete the reference
    # and force garbage collection to free the file handle
    try:
        if 'TRAJECTORY' in locals():
            del TRAJECTORY
    except Exception:
        pass
    
    # Force garbage collection to release file handles from memory-mapped arrays
    gc.collect()
=======
>>>>>>> parent of 8f1b4bd (fix mem issue and clean up old stuff)

    sup.simulationReset()
    sup.step(timestep)  # needed after reset
