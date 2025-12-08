"""Gym environment wrapping Webots evaluation as an episodic task.

Each action is a low-dimensional parameter vector (spline knots for active joints).
The environment expands knots into a full joint trajectory, writes `eval_data.json`
for `eval_thrower_controller`, runs Webots via `WebotsSimulator`, and returns a scalar reward.

This env treats each episode as a single trial (one action -> one Webots run).
"""

import os
import gym
import numpy as np
from gym import spaces
from scipy.interpolate import CubicSpline

from webots_simulator import WebotsSimulator

class WebotsThrowEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self,
                 world_file: str,
                 start_pose: np.ndarray,
                 end_pose: np.ndarray,
                 n_knots: int = 5,
                 run_name: str = 'rl_run',
                 use_simulation: bool = True,
                 show_gui: bool = False):
        super().__init__()
        self.world_file = world_file
        self.start_pose = np.array(start_pose)
        self.end_pose = np.array(end_pose)
        self.n_joints = 6
        self.n_knots = int(n_knots)
        self.active_joints = np.where(np.abs(self.start_pose - self.end_pose) > 0.01)[0]
        self.n_active = len(self.active_joints)
        self.run_name = run_name
        # Per-environment episode counter. incremented at each reset()
        self.episode_counter = 0
<<<<<<< HEAD
=======
        self.best_reward = -np.inf
        self.best_traj = None
>>>>>>> parent of 8f1b4bd (fix mem issue and clean up old stuff)

        # Action: knot values for active joints, flattened (n_active * n_knots)
        self.action_dim = self.n_active * self.n_knots
        # Reasonable bounds: use joint limits from simulator/dynamics assumption
        joint_min = -2 * np.pi
        joint_max = 2 * np.pi
        self.action_space = spaces.Box(low=joint_min, high=joint_max, shape=(self.action_dim,), dtype=np.float32)

        # Observation: simple vector with start and end poses (can be expanded later)
        self.observation_space = spaces.Box(low=-10.0, high=10.0, shape=(12,), dtype=np.float32)

        # Webots simulator instance
        self.sim = WebotsSimulator(self.world_file, headless=not show_gui, run_name=run_name)

    def reset(self):
        # New episode begins — increment episode counter and return initial obs
        self.episode_counter += 1
        obs = np.concatenate([self.start_pose, self.end_pose]).astype(np.float32)
        return obs

    def _knots_to_traj(self, action: np.ndarray, n_timesteps: int = 30):
        # Reconstruct full trajectory (n_timesteps x 6)
        # For inactive joints, keep constant at start_pose
        q_traj = np.tile(self.start_pose, (n_timesteps, 1)).astype(float)

        # Reshape action to (n_active, n_knots)
        knots = action.reshape(self.n_active, self.n_knots)

        # Parameterize time for knots excluding start/end
        t_knots = np.linspace(0, 1, self.n_knots)
        t_full = np.linspace(0, 1, n_timesteps)

        for idx_i, joint_idx in enumerate(self.active_joints):
            # Build cubic spline through [start, knots..., end]
            values = np.concatenate([[self.start_pose[joint_idx]], knots[idx_i], [self.end_pose[joint_idx]]])
            t_pts = np.linspace(0, 1, len(values))
            cs = CubicSpline(t_pts, values, bc_type='clamped')
            q_traj[:, joint_idx] = cs(t_full)

        return q_traj

    def step(self, action):
        # Action -> trajectory
        action = np.clip(action, self.action_space.low, self.action_space.high)
        q_traj = self._knots_to_traj(action, n_timesteps=30)

        # Run evaluation in Webots
        sim_result = self.sim.evaluate_trajectory(
            q_trajectory=q_traj,
            gripper_close_time=130,
            gripper_open_time=150 + 50 + len(q_traj),
            # pass the episode counter so the simulator and controller know which trial this is
            train_step=int(self.episode_counter)
        )

        # Build reward: encourage passing checkpoint and horizontal velocity
        vz = np.array(sim_result.get('release_velocity_z'))
        hor_speed = np.array(sim_result.get('release_velocity_hor'))
        final_distance = sim_result.get('final_distance', 0.0)
        success = bool(sim_result.get('success', False))

        reward = 0.0
        if success:
            reward += 1000.0
        else:
            reward -= 100
        reward += 50.0 * hor_speed
        reward += 5.0 * vz
        reward += 20.0 * final_distance

        # Expose last simulator result on the env instance so callbacks can access it
        try:
            # Include the computed reward and episode index in the sim result
            # Note: avoid using the reserved key 'episode' because SB3 expects
            # info['episode'] to be an episode-info dict. Use 'sim_episode'
            # instead to prevent SB3 from misinterpreting this field.
            sim_result['reward'] = float(reward)
            sim_result['sim_episode'] = int(self.episode_counter)
            self.last_sim_result = sim_result
        except Exception:
            self.last_sim_result = None

        obs = np.concatenate([self.start_pose, self.end_pose]).astype(np.float32)
        done = True  # one-shot episode
        info = sim_result

<<<<<<< HEAD
=======
        if success and reward > self.best_reward:
            self.best_reward = reward
            self.best_traj = {
                'run_name': self.run_name,
                'trajectory': q_traj.tolist(),
                'gripper_close_time': int(130),
                'gripper_open_time': 150 + 50 + int(release_param * len(q_traj))
            }
            eval_data_path = os.path.join(OUTPUT_DIR, 'best_traj.json')
            os.makedirs(OUTPUT_DIR, exist_ok=True)
            with open(eval_data_path, 'w') as f:
                f.write(json.dumps(self.best_traj) + '\n')

>>>>>>> parent of 8f1b4bd (fix mem issue and clean up old stuff)
        print(f"[Env] Episode {self.episode_counter} reward: {reward}")
        print(f"      Details: success={success}, hor_speed={hor_speed:.2f}, vertical_speed={vz:.2f}, final_distance={final_distance:.2f}")
        return obs, float(reward), done, info

    def render(self, mode='human'):
        # In this environment each episode is one full trajectory. For interactive
        # debugging we want to open Webots GUI and block until the user closes it.
        if mode == 'human':
            # Launch Webots GUI and wait until the user closes the window.
            # Note: this blocks the current thread — do not call during large-scale
            # training. Intended for debugging / visualization only.
            try:
                self.sim.launch_gui_and_wait()
            except Exception as e:
                print(f"render() failed to launch GUI: {e}")
        else:
            # other render modes not implemented
            return None

    def close(self):
        pass
