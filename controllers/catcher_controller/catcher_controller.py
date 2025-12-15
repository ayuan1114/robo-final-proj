"""ur5e_controller controller - adaptive catching for both naive and learned throws."""

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

# Robotiq 2F-85 gripper setup
GRIP_MIN = 0.0
GRIP_MAX = 0.80

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
            g.setVelocity(10.0)
        except Exception:
            pass
        try:
            g.setForce(200.0)
        except Exception:
            pass
    print("[gripper] Robotiq 2F-85 finger motors found.")
else:
    print("[gripper] Could not find Robotiq 2F-85 motors.")

def set_gripper_normalized(g):
    """g in [0,1]: 0=open, 1=closed."""
    if not (gripper_left and gripper_right):
        return
    g = float(np.clip(g, 0.0, 1.0))
    q = GRIP_MIN + g * (GRIP_MAX - GRIP_MIN)
    gripper_left.setPosition(q)
    gripper_right.setPosition(q)

def get_gripper_positions():
    """Get actual gripper positions for debugging"""
    if gripper_left and gripper_right:
        left_sensor = gripper_left.getPositionSensor()
        right_sensor = gripper_right.getPositionSensor()
        if left_sensor and right_sensor:
            left_sensor.enable(timestep)
            right_sensor.enable(timestep)
            return left_sensor.getValue(), right_sensor.getValue()
    return None, None

def get_joint_positions():
    return np.array([m.getPositionSensor().getValue() for m in motors])

# ADAPTIVE PARAMETERS

# Dynamically determine catch plane based on block trajectory
ADAPTIVE_CATCH_PLANE = True
FALLBACK_CATCH_X = -0.67  # Use if adaptive fails

# Flight detection - more permissive for learned throws
MIN_THROW_VX = -0.2  
MIN_SPEED = 0.3      
MIN_HEIGHT = 0.6    

# Gripper timing
GRIPPER_TRIGGER_DISTANCE = 0.25  
GRIPPER_CATCH_DISTANCE = 0.12

# Constants
GRAVITY = np.array([0.0, 0.0, -9.81])
MAX_INTERCEPT_TIME = 5.0  
MIN_INTERCEPT_TIME = 0.05
CATCH_MARGIN = 0.15

# Velocity history for better prediction
VEL_HISTORY_SIZE = 5
vel_history = []

# State variables
catching = False
intercept_joints = None
last_valid_prediction = None
gripper_closing = False
gripper_closed = False
catch_success = False
PASSED = False
tt = 0
flight_start_time = None
flight_start_pos = None

# adaptive catch plane
determined_catch_x = None

# default ready pose
READY_POSE = [-0.343, -1.2, 1.5, -2.0, -1.57, 1.03]

# Check if block is actually thrown and in flight - more permissive
# Block is in flight if:
    # - Moving towards catcher (negative x velocity)
    # - Has sufficient speed
    # - Is above minimum height
def is_block_in_flight(pos, vel):
    
    speed = np.linalg.norm(vel[:3])
    return (vel[0] < MIN_THROW_VX and
            speed > MIN_SPEED and
            pos[2] > MIN_HEIGHT)

# Estimate optimal catch plane based on trajectory.
#   Returns x-coordinate where we should intercept.
# Find a reasonable intercept point:
    # - In front of robot but reachable
    # - Not too close (need time to react)
    # - Not too far (accuracy degrades)
def estimate_catch_plane(block_pos, block_vel, robot_pos):
    
    robot_x = robot_pos[0]
    
    # Try multiple candidate catch planes
    candidates = []
    for offset in [-0.5, -0.6, -0.7, -0.8, -0.9, -1.0]:
        catch_x = robot_x + offset
        
        # only consider if block is moving towards this plane
        if block_pos[0] > catch_x:
            vx = block_vel[0]
            if vx < -0.1:  # moving towards catcher
                dt = (catch_x - block_pos[0]) / vx
                
                if 0.2 < dt < 4.0:  
                    # predict position at this catch plane
                    pred_pos = block_pos + block_vel[:3] * dt + 0.5 * GRAVITY * dt * dt
                    
                    # check if prediction is reasonable
                    if 0.4 < pred_pos[2] < 2.2:  # Reachable height
                        # calculate how far from robot center
                        dist_from_robot = np.linalg.norm(pred_pos - robot_pos)
                        
                        if dist_from_robot < 1.2:  # Within reach
                            candidates.append((catch_x, dt, pred_pos, dist_from_robot))
    
    if candidates:
        # choose the candidate with moderate time (not too early, not too late)
        candidates.sort(key=lambda x: abs(x[1] - 1.0))  # Prefer ~1 second intercept
        return candidates[0][0]
    
    return None

# predict where block will be when it crosses target_x plane
def predict_intercept(block_pos, block_vel, target_x):

    vx = block_vel[0]
    
    if vx >= -1e-6:
        return None, None
    
    dt = (target_x - block_pos[0]) / vx
    
    if dt < MIN_INTERCEPT_TIME or dt > MAX_INTERCEPT_TIME:
        return None, None
    
    pred_pos = block_pos + block_vel[:3] * dt + 0.5 * GRAVITY * dt * dt
    
    # more permissive height check
    if pred_pos[2] < 0.3 or pred_pos[2] > 2.5:
        return None, None
    
    return dt, pred_pos

# compute joint angles to reach target position
def compute_catch_joints(target_pos, robot_pos, current_joints):
    try:
        rel_pos = target_pos - robot_pos
        rel_pose = np.concatenate((rel_pos, [0, 0, 0]))
        joints = getDesiredRobotCommand(0, rel_pose, current_joints)
        
        if len(joints) == 6:
            if np.all(np.abs(joints) < 2 * np.pi):
                return joints
        return None
    except Exception as e:
        print(f"[catch] IK failed: {e}")
        return None

while sup.step(timestep) != -1:
    current_joints = get_joint_positions()
    block_position = np.array(block.getPosition())
    block_velocity = np.array(block.getVelocity()[:3])
    robot_pos = np.array(robot.getPosition())
    
    # calculate end effector position
    end_effector_pos = fk(current_joints)[:3]
    end_effector_world = end_effector_pos + robot_pos
    
    # distance from block to gripper
    block_to_gripper_dist = np.linalg.norm(block_position - end_effector_world)
    
    # smooth velocity with moving average
    vel_history.append(block_velocity)
    if len(vel_history) > VEL_HISTORY_SIZE:
        vel_history.pop(0)
    smoothed_vel = np.mean(vel_history, axis=0)
    
    # check if block is in flight
    in_flight = is_block_in_flight(block_position, smoothed_vel)
    
    # detect start of flight and determine catch plane
    if in_flight and flight_start_time is None:
        flight_start_time = tt
        flight_start_pos = block_position.copy()
        print(f"[catch] Block in flight detected at t={tt}")
        print(f"[catch] Position: {block_position}, Velocity: {smoothed_vel}")
        
        # determine catch plane adaptively
        if ADAPTIVE_CATCH_PLANE:
            determined_catch_x = estimate_catch_plane(block_position, smoothed_vel, robot_pos)
            if determined_catch_x is not None:
                print(f"[catch] Adaptive catch plane: x={determined_catch_x:.3f}")
            else:
                determined_catch_x = FALLBACK_CATCH_X
                print(f"[catch] Using fallback catch plane: x={FALLBACK_CATCH_X:.3f}")
        else:
            determined_catch_x = FALLBACK_CATCH_X
    
    # use determined catch plane (or fallback)
    catch_x = determined_catch_x if determined_catch_x is not None else FALLBACK_CATCH_X
    
    # update catch plan continuously while block is in flight (but stop if we've caught it)
    if in_flight and not PASSED and not catch_success:
        dt, pred_pos = predict_intercept(block_position, smoothed_vel, catch_x)
        
        if dt is not None and pred_pos is not None:
            joints = compute_catch_joints(pred_pos, robot_pos, current_joints)
            
            if joints is not None:
                if not catching:
                    print(f"[catch] Planning intercept in {dt:.3f}s at x={catch_x:.3f}")
                    print(f"[catch] Predicted position: {pred_pos}")
                
                catching = True
                intercept_joints = joints
                last_valid_prediction = pred_pos
                
                # Move to intercept position
                for j, motor in enumerate(motors):
                    motor.setPosition(float(intercept_joints[j]))
                
                # gripper timing logic
                if not gripper_closing and not gripper_closed:
                    # adaptive trigger based on distance and time
                    should_close = False
                    reason = ""
                    
                    # calculate dynamic trigger distance based on block speed
                    block_speed = np.linalg.norm(smoothed_vel)
                    # time for gripper to close (assume ~0.2 seconds)
                    gripper_close_time = 0.2
                    # distance block travels during closure
                    dynamic_trigger_dist = block_speed * gripper_close_time * 1.5  # 1.5x safety factor
                    
                    # use larger of static or dynamic trigger
                    trigger_dist = max(GRIPPER_TRIGGER_DISTANCE, dynamic_trigger_dist)
                    
                    if block_to_gripper_dist < trigger_dist:
                        should_close = True
                        reason = f"Distance (dist={block_to_gripper_dist:.3f}m, trigger={trigger_dist:.3f}m)"
                    # Also trigger on time
                    elif dt < 0.25:  
                        should_close = True
                        reason = f"Time (t={dt:.3f}s)"
                    
                    if should_close:
                        print(f"[gripper] *** CLOSING *** - {reason}")
                        gripper_closing = True
                    else:
                        set_gripper_normalized(0.0)
                        if tt % 10 == 0:
                            print(f"[gripper] OPEN - dist={block_to_gripper_dist:.3f}m, time={dt:.3f}s, trigger={trigger_dist:.3f}m")
                
                if gripper_closing and not gripper_closed:
                    set_gripper_normalized(1.0)
                    
                    if tt % 3 == 0:
                        left_pos, right_pos = get_gripper_positions()
                        avg_pos = (left_pos + right_pos) / 2 if left_pos is not None else 0.0
                        print(f"[gripper] CLOSING... dist={block_to_gripper_dist:.3f}m, gripper_pos={avg_pos:.3f}")
                    
                    # check if we've caught it
                    if block_to_gripper_dist < GRIPPER_CATCH_DISTANCE:
                        gripper_closed = True
                        catch_success = True
                        print(f"[gripper] --- CAUGHT --- Distance: {block_to_gripper_dist:.3f}m")
                        print(f"[gripper] Holding position - no longer tracking block")
    
    elif not catching and not catch_success:
        # default ready position (only if we haven't caught yet)
        for j, motor in enumerate(motors):
            motor.setPosition(READY_POSE[j])
        set_gripper_normalized(0.0)
    
    elif catch_success:
        # hold the catch position - don't move!
        if intercept_joints is not None:
            for j, motor in enumerate(motors):
                motor.setPosition(float(intercept_joints[j]))
    
    # keep gripper closed if we caught it
    if gripper_closed:
        set_gripper_normalized(1.0)
    
    # check if block passed catch plane
    if not PASSED and block_position[0] < catch_x:
        PASSED = True
        print(f"\n{'='*60}")
        print(f"[{tt}] Block crossed catch plane (x={catch_x:.3f})")
        print(f"[{tt}] Gripper: closing={gripper_closing}, closed={gripper_closed}")
        
        if last_valid_prediction is not None:
            actual_pos_rel = block_position - robot_pos
            predicted_pos_rel = last_valid_prediction - robot_pos
            error = np.linalg.norm(actual_pos_rel - predicted_pos_rel)
            
            print(f"[catch] Prediction error: {error:.3f}m")
            print(f"[catch] Final distance to gripper: {block_to_gripper_dist:.3f}m")
            
            if catch_success:
                print("[catch] SUCCESSFULLY CAUGHT!")
            elif block_to_gripper_dist < CATCH_MARGIN:
                print("[catch] Close but timing was off")
            else:
                print("[catch] Missed")
        else:
            print("[catch] No prediction was made")
        print(f"{'='*60}\n")
    
    tt += 1