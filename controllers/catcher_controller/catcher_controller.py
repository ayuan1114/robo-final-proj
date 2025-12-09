"""ur5e_controller controller."""

import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from controller import Robot, Supervisor
from utils.ik import getDesiredRobotCommand, fk

# initialize the supervisor, robot, and timestep
sup = Supervisor()
robot = sup.getFromDef("CATCHER")
block = sup.getFromDef("BLOCK")
timestep = int(sup.getBasicTimeStep())

# initialize URe5 arm moters (6 DOF)
arm_joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
motors = [sup.getDevice(n) for n in arm_joint_names]
for m in motors:
    ps = m.getPositionSensor()
    ps.enable(timestep)

# -----------------------------------------------------------------------------
# Robotiq 2F-85 gripper (two finger motors under toolSlot)
# Names come from your world: Robotiq2f85Gripper { name "robotiq_2f85" }
# So the device names are:
#   "robotiq_2f85::left finger joint", "robotiq_2f85::right finger joint"
# -----------------------------------------------------------------------------
GRIP_MIN = 0.0
GRIP_MAX = 0.80  # typical range on actuated finger joints; adjust if needed

def _safe_get_device(name):
    try:
        return sup.getDevice(name)
    except Exception:
        return None

gripper_left  = _safe_get_device("robotiq_2f85_catcher::left finger joint")
gripper_right = _safe_get_device("robotiq_2f85_catcher::right finger joint")

if gripper_left and gripper_right:
    for g in (gripper_left, gripper_right):
        try:
            g.setVelocity(1.5)
        except Exception:
            pass
        try:
            g.setForce(50.0)
        except Exception:
            pass
    print("[gripper] Robotiq 2F-85 finger motors found.")
else:
    print("[gripper] Could not find Robotiq 2F-85 motors. "
          "Expected: 'robotiq_2f85::left finger joint' and "
          "'robotiq_2f85::right finger joint'.")

def set_gripper_normalized(g):
    """g in [0,1]: 0=open, 1=closed."""
    if not (gripper_left and gripper_right):
        return
    g = float(np.clip(g, 0.0, 1.0))
    q = GRIP_MIN + g * (GRIP_MAX - GRIP_MIN)
    gripper_left.setPosition(q)
    gripper_right.setPosition(q)

def get_joint_positions():
    return np.array([m.getPositionSensor().getValue() for m in motors])

# Constants
CATCH_X = -0.67
GRAVITY = np.array([0.0, 0.0, -9.81])
MAX_INTERCEPT_TIME = 3.0
MIN_INTERCEPT_TIME = 0.1
CATCH_MARGIN = 0.15

# Thresholds for detecting when block is in flight
MIN_THROW_VX = -0.3  # block moving toward catcher (negative x)
MIN_SPEED = 0.5      # minimum total speed to consider "in flight"
MIN_HEIGHT = 0.8     # block must be above table to be considered thrown

# State variables
catching = False
intercept_joints = None
last_valid_prediction = None
PASSED = False
tt = 0

# Default ready pose
READY_POSE = [-0.343, -1.2, 1.5, -2.0, -1.57, 1.03]

def is_block_in_flight(pos, vel):
    """Check if block is actually thrown and in flight"""
    speed = np.linalg.norm(vel[:3])
    return (vel[0] < MIN_THROW_VX and  # moving toward catcher
            speed > MIN_SPEED and        # moving fast enough
            pos[2] > MIN_HEIGHT)         # above table

def predict_intercept(block_pos, block_vel, target_x):
    """Predict where block will be when it crosses target_x plane"""
    vx = block_vel[0]
    
    # Need negative velocity (moving toward catcher)
    if vx >= -1e-6:
        return None, None
    
    # Time to reach target x
    dt = (target_x - block_pos[0]) / vx
    
    # Check if time is reasonable
    if dt < MIN_INTERCEPT_TIME or dt > MAX_INTERCEPT_TIME:
        return None, None
    
    # Predict position with ballistic trajectory
    pred_pos = block_pos + block_vel[:3] * dt + 0.5 * GRAVITY * dt * dt
    
    # Sanity checks on predicted position
    if pred_pos[2] < 0.5 or pred_pos[2] > 2.0:  # reasonable height range
        return None, None
    
    return dt, pred_pos

def compute_catch_joints(target_pos, robot_pos, current_joints):
    """Compute joint angles to reach target position"""
    try:
        # Convert to relative position
        rel_pos = target_pos - robot_pos
        rel_pose = np.concatenate((rel_pos, [0, 0, 0]))
        
        # Get IK solution
        joints = getDesiredRobotCommand(0, rel_pose, current_joints)
        
        # Validate solution
        if len(joints) == 6:
            # Check if joints are in reasonable range
            if np.all(np.abs(joints) < 2 * np.pi):
                return joints
        
        return None
    except Exception as e:
        print(f"[catch] IK failed: {e}")
        return None

# Velocity history for better prediction
vel_history = []
VEL_HISTORY_SIZE = 5

while sup.step(timestep) != -1:
    current_joints = get_joint_positions()
    block_position = np.array(block.getPosition())
    block_velocity = np.array(block.getVelocity()[:3])
    robot_pos = np.array(robot.getPosition())
    
    # Smooth velocity with moving average
    vel_history.append(block_velocity)
    if len(vel_history) > VEL_HISTORY_SIZE:
        vel_history.pop(0)
    smoothed_vel = np.mean(vel_history, axis=0)
    
    # Check if block is in flight
    in_flight = is_block_in_flight(block_position, smoothed_vel)
    
    # Update catch plan continuously while block is in flight
    if in_flight and not PASSED:
        dt, pred_pos = predict_intercept(block_position, smoothed_vel, CATCH_X)
        
        if dt is not None and pred_pos is not None:
            # Try to compute joints for this intercept
            joints = compute_catch_joints(pred_pos, robot_pos, current_joints)
            
            if joints is not None:
                if not catching:
                    print(f"[catch] Block detected! Planning intercept in {dt:.3f}s")
                    print(f"[catch] Predicted position: {pred_pos}")
                
                catching = True
                intercept_joints = joints
                last_valid_prediction = pred_pos
                
                # Move to intercept position
                for j, motor in enumerate(motors):
                    motor.setPosition(float(intercept_joints[j]))
                
                # Open gripper wide
                set_gripper_normalized(0.0)
    
    elif not catching:
        # Default ready position
        for j, motor in enumerate(motors):
            motor.setPosition(READY_POSE[j])
        set_gripper_normalized(0.0)  # Keep open
    
    # Check if block passed catch plane
    if not PASSED and block_position[0] < CATCH_X:
        PASSED = True
        print(f"[{tt}] Block crossed catch plane at: {block_position}")
        
        if last_valid_prediction is not None:
            # Calculate catch accuracy
            actual_pos_rel = block_position - robot_pos
            predicted_pos_rel = last_valid_prediction - robot_pos
            error = np.linalg.norm(actual_pos_rel - predicted_pos_rel)
            
            print(f"[catch] Prediction error: {error:.3f}m")
            
            # Check if we actually caught it
            end_effector_pos = fk(current_joints)[:3]
            catch_distance = np.linalg.norm(actual_pos_rel - end_effector_pos)
            
            print(f"[catch] Distance to end effector: {catch_distance:.3f}m")
            
            if catch_distance < CATCH_MARGIN:
                print("[catch] ✓ CAUGHT!")
            else:
                print("[catch] ✗ Missed")
        else:
            print("[catch] No prediction was made")
    
    tt += 1