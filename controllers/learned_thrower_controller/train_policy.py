"""
Headless training script for learned throwing policy.
Optimizes trajectory using Webots simulation in the loop.
"""

import numpy as np
import pickle
import os
import argparse
from trajectory_optimizer import TrajectoryOptimizer

# Same poses as naive_thrower_controller
START_THROW_POSE = np.array([2.7, -1.22, 1.75, -2.1, -1.57, 3.14])
END_THROW_POSE = np.array([2.7, -1.22, 1, -3, -1.57, 3.14])

# Training parameters
N_TIMESTEPS = 30  # Duration of optimized throw (30 timesteps = ~0.48s at 16ms/step)
DT = 0.016  # Webots timestep (16ms)
MAX_ITER = 50  # Reduced iterations since simulation is slower

# Find world file - use train.wbt for evaluation
WORLD_FILE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'worlds', 'train.wbt'))

def train_throwing_policy(use_simulation=True, show_gui=False, run_name='run0'):
    """Train throwing policy using trajectory optimization"""
    
    print("\n" + "="*70)
    print(" TRAINING LEARNED THROWING POLICY")
    print(f" Mode: {'SIMULATION-IN-THE-LOOP' if use_simulation else 'ANALYTICAL MODEL'}")
    if use_simulation and show_gui:
        print(" GUI: ENABLED (showing Webots window)")
    print("="*70)
    
    print(f"\nTraining Configuration:")
    print(f"  Start Pose: {START_THROW_POSE}")
    print(f"  End Pose: {END_THROW_POSE}")
    print(f"  Timesteps: {N_TIMESTEPS}")
    print(f"  dt: {DT}s")
    print(f"  Total Duration: {N_TIMESTEPS * DT:.3f}s")
    
    if use_simulation:
        if not os.path.exists(WORLD_FILE):
            print(f"\n✗ ERROR: World file not found at {WORLD_FILE}")
            print("  Falling back to analytical model...")
            use_simulation = False
        else:
            print(f"  World File: {WORLD_FILE}")
    
    # Create optimizer
    optimizer = TrajectoryOptimizer(
        start_pose=START_THROW_POSE,
        end_pose=END_THROW_POSE,
        n_timesteps=N_TIMESTEPS,
        dt=DT,
        world_file=WORLD_FILE if use_simulation else None,
        use_simulation=use_simulation,
        show_gui=show_gui,
        run_name=run_name
    )
    
    # Run optimization
    result = optimizer.optimize(max_iter=MAX_ITER)
    
    if result['success']:
        print("\n✓ Optimization successful!")
        
        # Extract trajectories
        q_traj = result['q_trajectory']
        qd_traj = result['qd_trajectory']
        
        # Compute end-effector trajectory for analysis
        ee_positions = []
        for i in range(len(q_traj)):
            ee_pos = optimizer.dynamics.forward_kinematics(q_traj[i])
            ee_positions.append(ee_pos)
        ee_positions = np.array(ee_positions)
        
        print(f"\nTrajectory Analysis:")
        print(f"  Start EE position: {ee_positions[0]}")
        print(f"  End EE position: {ee_positions[-1]}")
        print(f"  EE displacement: {np.linalg.norm(ee_positions[-1] - ee_positions[0]):.3f}m")
        
        # Estimate release velocity
        if len(ee_positions) > 1:
            ee_vel_final = (ee_positions[-1] - ee_positions[-2]) / DT
            ee_speed_final = np.linalg.norm(ee_vel_final)
            print(f"  Release velocity: {ee_vel_final}")
            print(f"  Release speed: {ee_speed_final:.3f} m/s")
        
        # Check joint velocity limits
        max_qd = np.max(np.abs(qd_traj), axis=0)
        print(f"\nJoint Velocity Check:")
        for i, (qd, qd_max) in enumerate(zip(max_qd, optimizer.dynamics.qd_max)):
            status = "✓" if qd < qd_max else "✗"
            print(f"  Joint {i}: {qd:.3f}/{qd_max:.3f} rad/s {status}")
        
        # Save policy to policy_weights directory
        policy_data = {
            'q_trajectory': q_traj,
            'qd_trajectory': qd_traj,
            'start_pose': START_THROW_POSE,
            'end_pose': END_THROW_POSE,
            'n_timesteps': N_TIMESTEPS,
            'dt': DT,
            'cost': result['cost'],
            'ee_positions': ee_positions
        }
        
        # Create policy_weights directory if it doesn't exist
        policy_dir = os.path.join(os.path.dirname(__file__), 'policy_weights')
        os.makedirs(policy_dir, exist_ok=True)
        
        save_path = os.path.join(policy_dir, 'learned_policy.pkl')
        with open(save_path, 'wb') as f:
            pickle.dump(policy_data, f)
        
        print(f"\n✓ Policy saved to: {save_path}")
        
        # Also save as numpy for easy inspection
        np_save_path = os.path.join(policy_dir, 'learned_trajectory.npz')
        np.savez(np_save_path, 
                 q_trajectory=q_traj,
                 qd_trajectory=qd_traj,
                 ee_positions=ee_positions)
        print(f"✓ Numpy arrays saved to: {np_save_path}")
        
    else:
        print("\n✗ Optimization failed!")
        print(f"  Message: {result['result'].message}")
    
    print("\n" + "="*70)
    print(" TRAINING COMPLETE")
    print("="*70 + "\n")
    
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Train throwing policy')
    parser.add_argument('--no-sim', action='store_true', 
                       help='Use analytical model instead of Webots simulation')
    parser.add_argument('--gui', action='store_true',
                       help='Show Webots GUI during simulation (for debugging)')
    parser.add_argument('--run-name', type=str, default='run0',
                        help='Name of the training run (for logging purposes)')
    args = parser.parse_args()
    
    result = train_throwing_policy(use_simulation=not args.no_sim, show_gui=args.gui, run_name=args.run_name)
