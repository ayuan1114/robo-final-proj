"""ur5e_controller controller."""

import sys
import os
import numpy as np
import json

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from controller import Robot, Supervisor
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

gripper_left  = _safe_get_device("robotiq_2f85_thrower::left finger joint")
gripper_right = _safe_get_device("robotiq_2f85_thrower::right finger joint")

if gripper_left and gripper_right:
    for g in (gripper_left, gripper_right):
        try:
            g.setVelocity(1.5)
        except Exception:
            pass
        try:
            g.setForce(100.0)
        except Exception:
            pass
    print("[gripper] Robotiq 2F-85 finger motors found.")
else:
    print("[gripper] Could not find Robotiq 2F-85 motors. "
          "Expected: 'robotiq_2f85::left finger joint' and "
          "'robotiq_2f85::right finger joint'.")

def set_gripper_q(q):
    """Set finger joint target (radians) to both fingers."""
    if not (gripper_left and gripper_right):
        return
    q = float(np.clip(q, GRIP_MIN, GRIP_MAX))
    gripper_left.setPosition(q)
    gripper_right.setPosition(q)

def set_gripper_normalized(g):
    """g in [0,1]: 0=open, 1=closed."""
    g = float(np.clip(g, 0.0, 1.0))
    q = GRIP_MIN + g * (GRIP_MAX - GRIP_MIN)
    set_gripper_q(q)

def set_gripper_width_mm(width_mm, max_open_mm=85.0):
    """Approximate map from opening width (mm) to joint value; tune as needed."""
    width_mm = float(np.clip(width_mm, 0.0, max_open_mm))
    set_gripper_normalized(1.0 - width_mm / max_open_mm)

# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------

BLOCK_START_POS = np.concatenate((np.array(block.getPosition()) - np.array(robot.getPosition()), [0,0,0]))

BASE_POSE = [1.2, -1.2, 1.5, -2.0, -1.57, 1.03]
# START_THROW_POSE = np.array([2.7, -1.2, 1.5, -1, -1.57, 1.57])
# END_THROW_POSE = np.array([2.7, -1.2, 1.5, -3, -1.57, 1.57])
START_THROW_POSE = np.array([2.7, -1.22, 1.75, -2.1, -1.57, 3.14])
END_THROW_POSE = np.array([2.7, -1.22, 1, -3, -1.57, 3.14])

THROW_TIME = 15  # timesteps over which to execute the throw
MOVE_TIME = 50
PAUSE_TIME = 50
CHECKPOINT_X = -0.67
TABLE_Z = 0.715

start = None

class Arm:
    def __init__(self):
        self.start_pose = None
        self.end_pose = None
        self.throw_velocity = 0.0  # Velocity for interpolation progress
        self.throw_progress = 0.0  # Interpolation progress from 0 to 1
        self.throwing = False

    def get_pose(self, t: int, last_pose):
        # return END_THROW_POSE + [0]  # Placeholder for no movement
        if t < 50:
            return BASE_POSE + [0]
        elif t < 100:
            return np.concatenate((getDesiredRobotCommand(0, BLOCK_START_POS, last_pose), [0]))
        elif t < 130:
            return np.concatenate((getDesiredRobotCommand(0, BLOCK_START_POS, last_pose), [1]))
        elif t < 150:
            desired_pose = BLOCK_START_POS.copy()
            desired_pose[2] += 0.1  # lift 10cm above block
            return np.concatenate((getDesiredRobotCommand(0, desired_pose, last_pose), [1]))
        elif t < 150 + MOVE_TIME:
            if self.start_pose is None:
                self.start_pose = last_pose.copy()
                self.end_pose = START_THROW_POSE
            
            progress = (t - 150) / MOVE_TIME
            joints = (1 - progress) * self.start_pose + progress * self.end_pose
            
            if progress >= 1.0:
                self.start_pose = None
                self.end_pose = None
            return np.concatenate((joints, [1]))
        else:
            return np.concatenate((END_THROW_POSE, [1]))

tt = 0
last_pose = BASE_POSE

thrower = Arm()

# Tracking variables for statistics
checkpoint_reached = False
block_dropped = False
release_velocity = None
final_distance = 0.0
block_pos_at_release = None
landed = False
block_land_pos = None
pos_at_passing = None

while sup.step(timestep) != -1:

    t = sup.getTime()    

    cur_pose = thrower.get_pose(tt, last_pose[:-1])
    for j, motor in enumerate(motors):
        motor.setPosition(cur_pose[j])
    
    set_gripper_normalized(cur_pose[-1])

    # Get block position
    block_pos = np.array(block.getPosition())
    robot_pos = np.array(robot.getPosition())
    block_rel_pos = block_pos - robot_pos
    arm_pos = fk(last_pose[:6])[:3]

    # Check if block released (similar to eval_thrower_controller)
    if tt > 200 and not block_dropped:
        block_gripper_dist = np.linalg.norm(block_rel_pos - arm_pos)
        if block_gripper_dist > 0.08:
            block_dropped = True
            release_velocity = block.getVelocity()
            block_pos_at_release = block_rel_pos.copy()
            print(f"[NAIVE] Block released at t={tt}")

    if tt > 150:
        if not checkpoint_reached and block_pos[0] < CHECKPOINT_X:
            checkpoint_reached = True
            pos_at_passing = block_pos.copy()

        # Check if block landed
        if not landed and block_pos[2] < TABLE_Z:
            landed = True
            block_land_pos = block_rel_pos.copy()
            print(f"[NAIVE] Block landed at t={tt}")

    if tt == 200:
        print('=' * 50 + 'THROWN' + '=' * 50)

    if tt % 50 == 0:
        print(f"[{tt}] Thrower Config: ", cur_pose)
        print(f"[{tt}] Thrower End: ", fk(cur_pose[:-1])[:3])
        print(f"[{tt}] Thrower Block Pos: ", block_rel_pos)

    last_pose = cur_pose
    tt += 1

    # Exit when block has landed
    if landed:
        break

# Save statistics
result_path = os.path.join(os.path.dirname(__file__), "naive_result.json")

release_vel_hor = (
    -float(np.linalg.norm(release_velocity[:2])) if release_velocity is not None and release_velocity[0] > 0
    else float(np.linalg.norm(release_velocity[:2])) if release_velocity is not None
    else 0.0
)

final_distance = (
    -float(np.linalg.norm(block_land_pos[:2] - block_pos_at_release[:2])) if block_pos_at_release is not None and block_land_pos[0] < block_pos_at_release[0]
    else float(np.linalg.norm(block_land_pos[:2] - block_pos_at_release[:2])) if block_land_pos is not None and block_pos_at_release is not None
    else 0.0
)

result = {
    'controller': 'naive_thrower',
    'total_timesteps': tt,
    'block_released': block_dropped,
    'block_released_time': tt if block_dropped else None,
    'release_velocity_z': float(release_velocity[2]) if release_velocity is not None else 0.0,
    'release_velocity_hor': release_vel_hor,
    'success': checkpoint_reached,
    'final_distance': final_distance,
    'position_at_checkpoint': pos_at_passing.tolist() if pos_at_passing is not None else None,
    'position_at_release': block_pos_at_release.tolist() if block_pos_at_release is not None else None,
    'final_arm_pos': arm_pos.tolist(),
    'final_block_pos': block_rel_pos.tolist(),
}

with open(result_path, 'w') as f:
    json.dump(result, f, indent=2)

print(f"\n[NAIVE] Results saved to {result_path}")
print(f"[NAIVE] Success: {result['success']}")
print(f"[NAIVE] Release velocity (hor, z): ({release_vel_hor:.3f}, {result['release_velocity_z']:.3f}) m/s")
print(f"[NAIVE] Final distance: {final_distance:.3f} m")
