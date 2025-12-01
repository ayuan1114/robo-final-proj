"""Evaluation controller - executes trajectories from data files during optimization"""

import sys
import os
import numpy as np
import json

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

# -----------------------------------------------------------------------------
# Load evaluation data
# -----------------------------------------------------------------------------
eval_data_path = os.path.join(os.path.dirname(__file__), 'eval_data.json')

try:
    with open(eval_data_path, 'r') as f:
        # Read first/only line
        eval_data = json.loads(f.read().strip())
    
    TRAJECTORY = np.array(eval_data['trajectory'])
    GRIPPER_CLOSE_TIME = eval_data['gripper_close_time']
    GRIPPER_OPEN_TIME = eval_data['gripper_open_time']
    
    print(f"[EVAL] Loaded trajectory: {len(TRAJECTORY)} timesteps")
    print(f"[EVAL] Gripper close: t={GRIPPER_CLOSE_TIME}, open: t={GRIPPER_OPEN_TIME}")
    
except FileNotFoundError:
    print(f"[ERROR] No eval data found at {eval_data_path}")
    sup.simulationQuit(1)
    sys.exit(1)

# Same pickup sequence as learned_thrower_controller
BLOCK_START_POS = np.concatenate((np.array(block.getPosition()) - np.array(robot.getPosition()), [0,0,0]))
BASE_POSE = [1.2, -1.2, 1.5, -2.0, -1.57, 1.03]
MOVE_TIME = 50
CHECKPOINT_X = -0.67

# Load start throw pose from trajectory
START_THROW_POSE = TRAJECTORY[0]

# Tracking variables
checkpoint_reached = False
block_dropped = False
release_velocity = None
final_distance = 0.0
block_pos_at_release = None
gripper_closed = False

tt = 0
last_pose = np.array(BASE_POSE)

# Phase tracking
start_pose = None
end_pose = None

pos_at_passing = None

while sup.step(timestep) != -1:
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
        gripper_closed = True
        
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
        if tt == GRIPPER_OPEN_TIME and gripper_closed:
            set_gripper(False)
            gripper_closed = False
            gripper = 0
        else:
            gripper = 1 if gripper_closed else 0
            
    else:
        # Hold final pose for 100 steps to observe result
        joints = TRAJECTORY[-1]
        gripper = 0
        
        if not checkpoint_reached and block_pos[0] < CHECKPOINT_X:
            checkpoint_reached = True
            pos_at_passing = block_pos.copy()

        # Check if we should end
        if tt > 150 + MOVE_TIME + len(TRAJECTORY) + 100:
            break
    
    # Apply commands
    for j, motor in enumerate(motors):
        motor.setPosition(joints[j])
    set_gripper(gripper == 1)

    # Get current gripper position for drop detection
    gripper_pos = get_gripper_position()
    #block_gripper_dist = np.linalg.norm(block_rel_pos - arm_pos)

    # Check if block dropped during pickup/throw
    # Block has dropped if:
    # 1. Gripper should be closed (gripper_closed=True)
    # 2. Block is too far from gripper (>0.15m)
    # 3. Gripper fingers are not actually closed (<0.5, indicating nothing is gripping)
    if tt > GRIPPER_CLOSE_TIME and gripper_closed:
        if gripper_pos < 0.5: # or block_gripper_dist > 0.15:
            block_dropped = True
            release_velocity = block.getVelocity()
            block_pos_at_release = block_pos.copy()
            print(f"[EVAL] Block dropped at t={tt}")
            break
    
    # Record final distance after trajectory completes
    if tt == 150 + MOVE_TIME + len(TRAJECTORY) + 99:
        if block_pos_at_release is not None:
            final_block_position = block_pos[:2]

    last_pose = np.array(joints)
    tt += 1

# Write results


eval_results_dir = os.path.join(os.path.dirname(__file__), 'eval_results')
os.makedirs(eval_results_dir, exist_ok=True)
result_path = os.path.join(os.path.dirname(__file__), "eval_result.json")
result_archive_path = os.path.join(eval_results_dir, f'eval_results_{eval_data["run_name"]}.jsonl')

result = {
    'train_step': eval_data['train_step'],
    'success': checkpoint_reached,
    'release_velocity': np.linalg.norm(release_velocity) if release_velocity is not None else 0.0,
    'pos_at_cp': pos_at_passing.tolist() if pos_at_passing is not None else [0, 0, 0],
    'final_distance': float(np.linalg.norm(block_rel_pos - block_pos_at_release)) if block_pos_at_release is not None else 0.0,
}

with open(result_path, "w") as f:
    f.write(json.dumps(result) + '\n')

result = {
    'train step': eval_data['train_step'],
    "block released": block_dropped,
    "block released time": tt if block_dropped else 'N/A',
    "release velocity": np.linalg.norm(release_velocity) if release_velocity is not None else 0.0,
    "success": checkpoint_reached,
    "final distance (from release pos)": float(np.linalg.norm(block_rel_pos - block_pos_at_release)) if block_pos_at_release is not None else 0.0,
    "position at checkpoint (global)": pos_at_passing.tolist() if pos_at_passing is not None else 'Did not reach',
    "position at release (rel. arm base)": block_pos_at_release.tolist() if block_pos_at_release is not None else [0, 0, 0],
    "final arm pos (rel. arm base)": arm_pos.tolist(),
    "final block pos (rel. arm base)": block_rel_pos.tolist(),
    
}

with open(result_archive_path, 'a') as f:
    f.write(json.dumps(result) + '\n')

print(f"[EVAL] Results: success={result['success']}, velocity={release_velocity}, distance={final_distance:.3f}m")

sup.simulationQuit(0)
