"""
Webots simulator wrapper for headless policy evaluation.
"""

import numpy as np
import subprocess
import tempfile
import os
import time
import shutil
from typing import Tuple, Optional
import json
import uuid
import dotenv

dotenv.load_dotenv()

# Optional psutil import for monitoring Webots process memory
try:
    import psutil
except Exception:
    psutil = None

class WebotsSimulator:
    """Headless Webots simulator for evaluating throwing trajectories"""
    
    def __init__(self, world_file: str, timestep: float = 0.016, headless: bool = True, run_name: str = 'run0', restart_every: int = 100, restart_memory_mb: Optional[int] = None):
        """
        Args:
            world_file: Path to the Webots world file
            timestep: Simulation timestep in seconds
            headless: If True, run without graphics; if False, show GUI
        """
        print("[SIM] Initializing WebotsSimulator")

        self.world_file = world_file
        self.timestep = timestep
        self.headless = headless
        self.webots_process = None
        self.project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
        self.controllers_dir = os.path.join(self.project_root, 'controllers')
        self.run_name = run_name
        self.process = None
        # Restart policy: restart Webots every `restart_every` evaluations or
        # when its RSS exceeds `restart_memory_mb` (MB). Set to None to disable.
        self.run_count = 0
        # Allow environment variables to override configuration so you can
        # experiment without editing code.
        env_restart_every = os.getenv('WEBOTS_RESTART_EVERY')
        env_restart_mem = os.getenv('WEBOTS_RESTART_MEMORY_MB')

        # Set aggressive defaults to prevent memory leaks: restart every 20 runs or at 2GB
        self.restart_every = int(env_restart_every) if env_restart_every is not None else (int(restart_every) if restart_every is not None else 20)
        self.restart_memory_mb = int(env_restart_mem) if env_restart_mem is not None else (int(restart_memory_mb) if restart_memory_mb is not None else 2048)

    def evaluate_trajectory(self, 
                          q_trajectory: np.ndarray,
                          gripper_close_time: int = 130,
                          gripper_open_time: int = 200,
                          train_step: int = 0) -> dict:
        """
        Evaluate a trajectory in headless Webots simulation.
        
        Args:
            q_trajectory: Joint trajectory (n_timesteps, 6)
            initial_block_pos: Initial block position relative to robot
            gripper_close_time: Timestep when gripper closes
            gripper_open_time: Timestep when gripper opens
            
        Returns:
            dict with 'success', 'block_velocity', 'block_dropped', 'final_distance'
        """
        # Save trajectory data to eval_thrower_controller directory
        controller_dir = os.path.join(self.controllers_dir, 'eval_thrower_controller')
        result_path = os.path.join(os.path.dirname(__file__), "..", "eval_thrower_controller", "eval_result.json")
        os.makedirs(controller_dir, exist_ok=True)

        # Save trajectory and gripper timing to JSON data file
        # Generate an easy-to-read unique run id for traceability
        run_id = str(uuid.uuid4())
        print(f"[SIM] Starting run_id={run_id} train_step={train_step}")

        # Save trajectory as a compact binary `.npy` with float32 to avoid
        # embedding large JSON payloads in eval_data. The controller can
        # memory-map this file to avoid loading the entire trajectory into RAM.
        policy_weights_dir = os.path.join(os.path.dirname(__file__), 'trajectory_archive')
        os.makedirs(policy_weights_dir, exist_ok=True)

        traj_fname = f'eval_traj_{self.run_name}_{run_id}.npy'
        traj_path = os.path.join(policy_weights_dir, traj_fname)
        # Save as float32 to reduce size
        np.save(traj_path, np.asarray(q_trajectory, dtype=np.float32))

        eval_data = {
            'run_id': run_id,
            'run_name': self.run_name,
            'train_step': train_step,
            # Provide path to binary trajectory file instead of embedding the list
            'trajectory_path': traj_path,
            'render': not self.headless,
            'gripper_close_time': int(gripper_close_time),
            'gripper_open_time': int(gripper_open_time)
        }

        eval_data_path = os.path.join(controller_dir, 'eval_data.json')
        with open(eval_data_path, 'w') as f:
            f.write(json.dumps(eval_data) + '\n')

        # Archive trajectory metadata (path + run info) for later inspection
        archive_path = os.path.join(policy_weights_dir, f'eval_traj_{self.run_name}.jsonl')
        archive_entry = {
            'run_id': run_id,
            'run_name': self.run_name,
            'train_step': train_step,
            'trajectory_path': traj_path,
            'gripper_close_time': int(gripper_close_time),
            'gripper_open_time': int(gripper_open_time)
        }
        with open(archive_path, 'a') as f:
            f.write(json.dumps(archive_entry) + '\n')
        
        # Run simulation
        # Increment run counter and possibly restart Webots to avoid memory growth
        self.run_count += 1
        self._maybe_restart_process()
        result = self._run_simulation()
        
        # Wait for the controller to write a matching result. The controller
        # may create the file and write it in multiple steps; ignore empty or
        # invalid JSON until we can parse a complete result with the same
        # run_id.
        while True:
            time.sleep(0.1)
            try:
                with open(result_path, 'r') as f:
                    text = f.read().strip()
                    if not text:
                        # File exists but is empty yet
                        continue
                    try:
                        result = json.loads(text)
                    except json.JSONDecodeError:
                        # Partial write — wait and retry
                        continue

                if result.get('run_id', '') == run_id:
                    break

            except FileNotFoundError:
                # Controller hasn't created the file yet
                continue

        # Log Webots process RSS for this run (helpful to choose restart thresholds)
        try:
            mem_log_path = os.path.join(self.controllers_dir, 'learned_thrower_controller', 'webots_memory_log.csv')
            header = not os.path.exists(mem_log_path)
            rss_mb = ''
            if psutil is not None and self.process is not None:
                try:
                    p = psutil.Process(self.process.pid)
                    rss_mb = f"{p.memory_info().rss / (1024**2):.2f}"
                except Exception:
                    rss_mb = ''
            with open(mem_log_path, 'a') as mf:
                if header:
                    mf.write('ts,run_id,run_name,run_count,rss_mb\n')
                mf.write(f"{time.time()},{run_id},{self.run_name},{self.run_count},{rss_mb}\n")
        except Exception:
            pass

        # Clean up old trajectory files to prevent memory/disk accumulation
        # Delete the trajectory file after successful evaluation since it's no longer needed
        try:
            if os.path.exists(traj_path):
                os.remove(traj_path)
        except Exception as e:
            print(f"[SIM] Warning: failed to delete trajectory file {traj_path}: {e}")

        return result
    
    def _run_simulation(self) -> dict:
        """Ensure Webots is running and execute the evaluation."""

        if self.process is not None:
            if self.process.poll() is None:
                # Process already running; nothing to start
                return

        # Build Webots command
        if self.headless:
            webots_cmd = [
                "webots",
                "--mode=fast",
                "--no-rendering",
                "--minimize",
                "--batch",
                self.world_file
            ]
        else:
            # Run with graphics for visualization/debugging
            webots_cmd = [
                "webots",
                #"--mode=fast",
                self.world_file
            ]
        
        self.process = subprocess.Popen(webots_cmd)
        # Give Webots a short moment to initialize native resources after
        # startup. This small sleep reduces races between process start and
        # inspectors (psutil) or file-based signaling.
        try:
            time.sleep(0.5)
        except Exception:
            pass

    def _maybe_restart_process(self) -> None:
        """Restart the Webots process based on run count or RSS memory.

        This helps mitigate Webots memory leaks in long headless fast runs by
        periodically restarting the process.
        """
        if self.process is None:
            return

        # If process exited, clear reference so _run_simulation can start a new one
        if self.process.poll() is not None:
            self.process = None
            return

        # Restart based on run count
        try:
            if self.restart_every is not None and self.run_count % self.restart_every == 0:
                print(f"[SIM] Restarting Webots after {self.run_count} runs to limit memory growth")
                self._terminate_process()
                self.process = None
                return
        except Exception:
            pass

        # Restart based on memory usage
        try:
            if psutil is not None and self.restart_memory_mb is not None:
                try:
                    p = psutil.Process(self.process.pid)
                    rss_mb = p.memory_info().rss / (1024 ** 2)
                    if rss_mb > self.restart_memory_mb:
                        print(f"[SIM] Webots RSS={rss_mb:.0f}MB > {self.restart_memory_mb}MB — restarting")
                        self._terminate_process()
                        self.process = None
                        return
                except Exception:
                    # If psutil can't inspect the process, ignore
                    pass
        except Exception:
            pass

    def _terminate_process(self, timeout: float = 5.0) -> None:
        """Terminate the Webots process gracefully, then kill if it doesn't exit."""
        try:
            if self.process is None:
                return
            print("[SIM] Terminating Webots process")
            self.process.terminate()
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                print("[SIM] Webots did not exit in time; killing")
                self.process.kill()
            finally:
                self.process = None
        except Exception as e:
            print(f"[SIM] Failed to terminate Webots process: {e}")

    def launch_gui_and_wait(self) -> None:
        """Launch Webots with GUI and block until the user closes the application.

        This is a convenience helper used by environments' `render()` methods for
        interactive debugging. It does not read/write eval data — it simply opens
        the world file with Webots and waits until the program exits.
        """
        webots_cmd = [
            "webots",
            self.world_file
        ]
        # Use subprocess.run to block until the user closes the GUI
        subprocess.run(webots_cmd, check=False)