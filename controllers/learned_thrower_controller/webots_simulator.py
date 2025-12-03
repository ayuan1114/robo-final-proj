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

class WebotsSimulator:
    """Headless Webots simulator for evaluating throwing trajectories"""
    
    def __init__(self, world_file: str, timestep: float = 0.016, headless: bool = True, run_name: str = 'run0'):
        """
        Args:
            world_file: Path to the Webots world file
            timestep: Simulation timestep in seconds
            headless: If True, run without graphics; if False, show GUI
        """
        self.world_file = world_file
        self.timestep = timestep
        self.headless = headless
        self.webots_process = None
        self.project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
        self.controllers_dir = os.path.join(self.project_root, 'controllers')
        self.run_name = run_name
        
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
        os.makedirs(controller_dir, exist_ok=True)

        # Save trajectory and gripper timing to JSON data file
        eval_data = {
            'run_name': self.run_name,
            'train_step': train_step,
            'trajectory': q_trajectory.tolist(),
            'render': not self.headless,
            'gripper_close_time': int(gripper_close_time),
            'gripper_open_time': int(gripper_open_time)
        }

        eval_data_path = os.path.join(controller_dir, 'eval_data.json')
        with open(eval_data_path, 'w') as f:
            f.write(json.dumps(eval_data) + '\n')
        
        # Archive trajectory to policy_weights for later testing
        policy_weights_dir = os.path.join(os.path.dirname(__file__), 'trajectory_archive')
        os.makedirs(policy_weights_dir, exist_ok=True)
        
        archive_path = os.path.join(policy_weights_dir, f'eval_traj_{self.run_name}.jsonl')
        with open(archive_path, 'a') as f:
            f.write(json.dumps(eval_data) + '\n')
        
        # Run simulation
        result = self._run_simulation()
        
        return result
    
    def _run_simulation(self) -> dict:
        """Run Webots simulation and get results"""
        result_path = os.path.join(os.path.dirname(__file__), "..", "eval_thrower_controller", "eval_result.json")
        
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
        
        try:
            # Run simulation - don't capture output so prints pass through to console
            process = subprocess.run(webots_cmd, timeout=30, capture_output=False)
            
            import json
            with open(result_path, 'r') as f:
                # Read the last line (most recent result)
                result = json.loads(f.read().strip())
            os.remove(result_path)
            return result
                
        except subprocess.TimeoutExpired:
            return {
                "success": False,
                "block_dropped": True,
                "release_velocity": [0, 0, 0],
                "final_distance": 0.0
            }

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