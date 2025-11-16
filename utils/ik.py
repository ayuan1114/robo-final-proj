import numpy as np
from scipy.spatial.transform import Rotation as R
from .kinematic_helpers import *

accept_threshold = 0.0005
match_angle = True

def getDesiredRobotCommand(tt, desired_pose, current_q):
    current_q = np.asarray(current_q, dtype=float)
    desired_pose = np.asarray(desired_pose, dtype=float)

    def normalize_angles(q):
        q = np.asarray(q, dtype=float)
        return np.arctan2(np.sin(q), np.cos(q))

    current_q = normalize_angles(current_q)
    cur_pos = fk(current_q, include_angle=match_angle)
    if not match_angle:
        desired_pose = desired_pose[:3]

    def pose_error(desired, current):
        desired = np.asarray(desired, dtype=float)
        current = np.asarray(current, dtype=float)
        n_pos = min(3, desired.size, current.size)
        pos_err = desired[:n_pos] - current[:n_pos]
        ang_err = np.array([])
        if desired.size > n_pos and current.size > n_pos:
            ang_des = desired[n_pos:]
            ang_cur = current[n_pos:]
            ang_diff = ang_des - ang_cur
            ang_err = np.arctan2(np.sin(ang_diff), np.cos(ang_diff))
        if ang_err.size:
            combined = np.concatenate((pos_err, ang_err))
        else:
            combined = pos_err
        return np.linalg.norm(combined)

    err = pose_error(desired_pose, cur_pos)
    iters = 0
    max_iters = 200
    while err > accept_threshold and iters < max_iters:
        J = compute_jacobian(current_q, match_angle=match_angle)
        dq = np.linalg.pinv(J) @ (desired_pose - cur_pos)
        current_q = current_q + dq
        current_q = normalize_angles(current_q)
        cur_pos = fk(current_q, include_angle=match_angle)
        err = pose_error(desired_pose, cur_pos)
        iters += 1
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
