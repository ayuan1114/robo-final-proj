import numpy as np
from scipy.spatial.transform import Rotation as R # you might find this useful for calculating the error
from kinematic_helpers import *
    
accept_threshold = 0.0005
match_angle = True

def getDesiredRobotCommand(tt, desired_pose, current_q):
    # tt: current simulation timestep
    # desired_pose: a list of six for the desired position (3) and desired exponential coordinates (3)
    # current q is the list of six joint angles: initially the current six joint angles, and then the last commanded joint angles after that
    # a timestep is 0.016s (62.5 Hz)

    # write your code here...
    # you should return a six-element list of the joint angles in radians

    # your goal is to program the inverse kinematics so the 
    # robot tracks the red circle to trace a W using the Newton-Raphson method
    
    # you are provided with the desired_pose
    # you do not need to use tt in this assignment
    
    # you have been provided with kinematic helper functions
    # for each one of the robot transforms. You can use these to form the
    # Jacobian. Note, the first z-axis in the jacobian should be for the identity
    # matrix (the first joint rotates about the z-axis). If you start with T01, you
    # will miss the first Jacobian component!
    
    # you are welcome to break it down with as many helper functions as necessary
    # you do not need to analytically solve for each entry of the Jacobian matrix.

    # replace this with your IK solution! :)\

    # ensure inputs are numpy arrays to allow elementwise ops
    current_q = np.asarray(current_q, dtype=float)
    desired_pose = np.asarray(desired_pose, dtype=float)

    # helper: normalize angles to [-pi, pi]
    def normalize_angles(q):
        q = np.asarray(q, dtype=float)
        return np.arctan2(np.sin(q), np.cos(q))

    # normalize incoming joint angles
    current_q = normalize_angles(current_q)

    cur_pos = fk(current_q, include_angle=match_angle)
    if not match_angle:
        desired_pose = desired_pose[:3]  # ignore orientation for now

    # compute error taking into account that trailing elements are angles
    def pose_error(desired, current):
        desired = np.asarray(desired, dtype=float)
        current = np.asarray(current, dtype=float)
        # positional part (first up to 3 elements)
        n_pos = min(3, desired.size, current.size)
        pos_err = desired[:n_pos] - current[:n_pos]
        # angular part: remaining elements (if any) should be treated as angles
        ang_err = np.array([])
        if desired.size > n_pos and current.size > n_pos:
            ang_des = desired[n_pos:]
            ang_cur = current[n_pos:]
            # normalize angular difference to [-pi, pi]
            ang_diff = ang_des - ang_cur
            ang_err = np.arctan2(np.sin(ang_diff), np.cos(ang_diff))
        # combine
        if ang_err.size:
            combined = np.concatenate((pos_err, ang_err))
        else:
            combined = pos_err
        return np.linalg.norm(combined)

    err = pose_error(desired_pose, cur_pos)
    iters = 0

    # guard against infinite loops
    max_iters = 200
    while err > accept_threshold and iters < max_iters:
        # Compute the Jacobian
        J = compute_jacobian(current_q, match_angle=match_angle)

        # Update the joint angles using the pseudo-inverse of the Jacobian
        dq = np.linalg.pinv(J) @ (desired_pose - cur_pos)
        # elementwise update
        current_q = current_q + dq
        # normalize angles to keep them within [-pi, pi]
        current_q = normalize_angles(current_q)

        # Recompute the current position
        cur_pos = fk(current_q, include_angle=match_angle)
        err = pose_error(desired_pose, cur_pos)

        print(f"Iteration {iters}: target = {desired_pose}, current = {cur_pos}, error = {err}")
        iters += 1

    # final normalization and return as numpy array
    q = normalize_angles(current_q)
    return np.array(q)

def fk(q, include_angle=True):
    t05 = np.eye(4) @ T01(q[0]) @ T12(q[1]) @ T23(q[2]) @ T34(q[3]) @ T45(q[4]) @ T56(q[5])
    
    to_ret = t05[:3, 3]
    if include_angle:
        to_ret = np.concatenate((to_ret, R.from_matrix(t05[:3, :3]).as_rotvec()))
    return to_ret

def compute_jacobian(q, match_angle=False, step_size=1e-6):
    pose_dim = 6 if match_angle else 3
    
    to_ret = np.zeros((pose_dim, len(q)))
    
    for i in range(len(q)):
        q_plus = q.copy()
        q_plus[i] += step_size
        pos_plus = fk(q_plus, include_angle=match_angle)

        q_minus = q.copy()
        q_minus[i] -= step_size
        pos_minus = fk(q_minus, include_angle=match_angle)

        to_ret[:, i] = (pos_plus - pos_minus) / (2 * step_size)

    return to_ret