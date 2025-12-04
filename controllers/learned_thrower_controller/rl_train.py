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
    from stable_baselines3.common.callbacks import BaseCallback, CallbackList
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

    n_envs = 1

    env = DummyVecEnv([lambda: make_env(args)] * n_envs)

    # Use sensible defaults: PPO requires n_steps * n_envs >= batch_size.
    # Webots episodes are slow, so keep batch sizes modest but n_steps > 1.
    model = PPO('MlpPolicy', env, verbose=1, n_steps=2, batch_size=2)

    print("Starting training — this will be slow because each episode runs Webots.")

    # Prepare simulation-result logging callback (will attempt to log to wandb if available)
    class SimResultLoggingCallback(BaseCallback):
        """Logs numeric fields from the env's `last_sim_result` to wandb (if available).

        This callback runs on each rollout end and will inspect all wrapped envs
        (works with DummyVecEnv). It only logs numeric values and prefixes keys
        with `sim/` to keep them separate from training metrics.
        """
        def __init__(self, verbose=0):
            super().__init__(verbose)

        def _on_step(self) -> bool:
            # Required abstract method for BaseCallback. We don't need per-step
            # behavior; return True to continue training.
            return True

        def _on_rollout_end(self) -> None:
            try:
                envs = getattr(self.training_env, 'envs', [])
                for env in envs:
                    res = getattr(env, 'last_sim_result', None)
                    if not res:
                        continue
                    # collect numeric-only values
                    log_dict = {}
                    for k, v in res.items():
                        if isinstance(v, (int, float)):
                            log_dict[f'sim/{k}'] = v
                    if not log_dict:
                        continue
                    try:
                        import wandb
                        wandb.log(log_dict, step=self.num_timesteps)
                    except Exception:
                        if self.verbose:
                            print("SimResult (no wandb):", log_dict)
            except Exception as e:
                if self.verbose:
                    print("SimResultLoggingCallback error:", e)

    sim_cb = SimResultLoggingCallback(verbose=1)

    # Optional Weights & Biases integration. If wandb and the SB3 integration
    # are installed, initialize a run and pass the WandbCallback to log metrics.
    try:
        import wandb
        from wandb.integration.sb3 import WandbCallback
        print("wandb available — initializing run")
        import time
        timestamp = int(time.time())
        run_name = f"{args.run_name}_{timestamp}"
        wandb.init(project='robo-final-proj', name=run_name, config=vars(args))
        cb_list = CallbackList([WandbCallback(), sim_cb])
        model.learn(total_timesteps=args.timesteps, callback=cb_list)
        wandb.finish()
    except Exception as e:
        print(f"wandb integration unavailable, running without wandb: {e}")
        # still use sim result callback (will print if wandb not installed)
        model.learn(total_timesteps=args.timesteps, callback=sim_cb)

    save_path = os.path.join(os.path.dirname(__file__), f'ppo_{args.run_name}.zip')
    model.save(save_path)
    print(f"Saved model to {save_path}")

    

    print('Done traingin, terminating sim')
    
    eval_data_path = os.path.join('..', 'eval_thrower_controller', 'eval_data.json')
    payload = {
        'run_id': 'done training'
    }
    
    with open(eval_data_path, 'w') as f:
        import json
        json.dump(payload, f)

if __name__ == '__main__':
    main()
