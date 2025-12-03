"""Train a policy with PPO on the WebotsThrowEnv.

This script requires `stable-baselines3` and `gym`.
It trains a policy that outputs spline knots (flattened) for active joints.

Usage:
    python rl_train.py --run-name rl1 --timesteps 2048 --n_knots 5 --popsize 8

Notes:
- Each PPO ``update`` will require many environment episodes; this can be slow because
  each episode runs a full Webots simulation. Start small to verify integration.
"""

import argparse
import os
import sys
import numpy as np

# Check for gym and stable-baselines3 and give actionable diagnostics if missing.
try:
    import gym  # noqa: F401
except Exception as e:
    print(f"ERROR: failed to import 'gym': {e}")
    print(f"Python executable: {sys.executable}")
    print("Install into the active environment with:")
    print("  pip install gym stable-baselines3")
    print("Or, if using the project's venv:")
    print("  .\\.venv\\Scripts\\Activate.ps1 ; pip install gym stable-baselines3")
    raise

# stable-baselines3 imports
try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv
except Exception as e:
    print(f"ERROR: failed to import 'stable_baselines3': {e}")
    print(f"Python executable: {sys.executable}")
    print("Install into the active environment with:")
    print("  pip install stable-baselines3 gym")
    print("Or, if using the project's venv:")
    print("  .\\.venv\\Scripts\\Activate.ps1 ; pip install stable-baselines3 gym")
    raise

# Import the Gym wrapper after verifying gym is available
from webots_gym_env import WebotsThrowEnv


def make_env(args):
    world = args.world
    env = WebotsThrowEnv(world_file=world,
                         start_pose=np.array([2.7, -1.22, 1.75, -2.1, -1.57, 3.14]),
                         end_pose=np.array([2.7, -1.22, 1.0, -3.0, -1.57, 3.14]),
                         n_knots=args.n_knots,
                         run_name=args.run_name,
                         show_gui=args.gui)
    return env


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-name', type=str, default='rl_run')
    parser.add_argument('--timesteps', type=int, default=4096)
    parser.add_argument('--n-knots', dest='n_knots', type=int, default=5)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--world', type=str, default=os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'worlds', 'train.wbt')))
    args = parser.parse_args()

    n_envs = 2

    env = DummyVecEnv([lambda: make_env(args)] * n_envs)

    # Use sensible defaults: PPO requires n_steps * n_envs >= batch_size.
    # Webots episodes are slow, so keep batch sizes modest but n_steps > 1.
    model = PPO('MlpPolicy', env, verbose=1, n_steps=1, batch_size=2)

    print("Starting training — this will be slow because each episode runs Webots.")
    model.learn(total_timesteps=args.timesteps)

    save_path = os.path.join(os.path.dirname(__file__), f'ppo_{args.run_name}.zip')
    model.save(save_path)
    print(f"Saved model to {save_path}")


if __name__ == '__main__':
    main()
