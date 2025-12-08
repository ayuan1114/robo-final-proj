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

class WebotsSimulator:
    """Headless Webots simulator for evaluating throwing trajectories"""
    
    def __init__(self, world_file: str, timestep: float = 0.016, headless: bool = True, run_name: str = 'run0'):
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
        result = self._run_simulation()
        
        while True:
            time.sleep(0.1)
            try:
                with open(result_path, 'r') as f:
                    result = json.loads(f.read().strip())
                
                if result.get('run_id', '') == run_id:
                    break


            except FileNotFoundError:
                pass

        return result
    
    def _run_simulation(self) -> dict:
        """Ensure Webots is running and execute the evaluation."""

        if self.process is not None:
            if self.process.poll() is None:
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