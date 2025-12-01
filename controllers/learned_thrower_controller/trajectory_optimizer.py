"""
Trajectory optimization for robot throwing using direct collocation.
Optimizes joint trajectories subject to dynamic constraints.
Now with Webots simulation-in-the-loop for accurate evaluation.
"""

import numpy as np
from scipy.optimize import minimize
from typing import Tuple, Optional
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from utils.kinematic_helpers import T01, T02, T03, T04, T05, T06
from webots_simulator import WebotsSimulator


class UR5eSimpleDynamics:
    """Simplified dynamics model for UR5e (without Pinocchio for CPU efficiency)"""
    
    def __init__(self):
        # UR5e approximate link masses (kg) and inertias
        self.link_masses = np.array([3.7, 8.4, 2.3, 1.2, 1.2, 0.25])
        self.link_lengths = np.array([0.0, 0.425, 0.3922, 0.0, 0.0, 0.0])
        
        # Approximate link center of mass distances from joint
        self.link_com_dist = np.array([0.0, 0.2125, 0.1961, 0.0, 0.0, 0.0])
        
        # Approximate link inertias (kg*m^2) - simplified as cylinders
        self.link_inertias = np.array([0.01, 0.5, 0.1, 0.01, 0.01, 0.001])
        
        # Joint limits (rad)
        self.q_min = np.array([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi])
        self.q_max = np.array([2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi])
        
        # Velocity limits (rad/s)
        self.qd_max = np.array([3.15, 3.15, 3.15, 3.2, 3.2, 3.2])
        
        # Torque limits (Nm)
        self.tau_max = np.array([150, 150, 150, 28, 28, 28])
        
        # Gravity
        self.g = 9.81
    
    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Compute end-effector position from joint angles"""
        T = T06(q[0], q[1], q[2], q[3], q[4], q[5])
        return T[:3, 3]
    
    def compute_inertia_matrix(self, q: np.ndarray) -> np.ndarray:
        """Simplified inertia matrix (diagonal approximation)"""
        M = np.diag(self.link_inertias)
        return M
    
    def compute_coriolis(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        """Simplified Coriolis/centrifugal terms (approximation)"""
        # Simplified: C(q,qd) ≈ 0 for trajectory optimization
        return np.zeros(6)
    
    def compute_gravity(self, q: np.ndarray) -> np.ndarray:
        """Gravity torques using simplified model"""
        g_vec = np.zeros(6)
        
        # Only shoulder_lift (joint 1) and elbow (joint 2) significantly affected
        # Approximate gravity load based on configuration
        g_vec[1] = -self.link_masses[1] * self.g * self.link_com_dist[1] * np.cos(q[1])
        g_vec[1] += -self.link_masses[2] * self.g * self.link_lengths[1] * np.cos(q[1])
        
        g_vec[2] = -self.link_masses[2] * self.g * self.link_com_dist[2] * np.cos(q[1] + q[2])
        
        return g_vec
    
    def inverse_dynamics(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
        """Compute required torques using simplified dynamics"""
        M = self.compute_inertia_matrix(q)
        C = self.compute_coriolis(q, qd)
        G = self.compute_gravity(q)
        
        # tau = M*qdd + C*qd + G
        tau = M @ qdd + C + G
        return tau


class TrajectoryOptimizer:
    """Direct collocation trajectory optimizer for throwing with Webots simulation"""
    
    def __init__(self, 
                 start_pose: np.ndarray,
                 end_pose: np.ndarray,
                 n_timesteps: int = 30,
                 dt: float = 0.016,
                 world_file: Optional[str] = None,
                 use_simulation: bool = True,
                 show_gui: bool = False,
                 run_name: str = 'run0'):
        """
        Args:
            start_pose: Initial joint configuration (6,)
            end_pose: Final joint configuration (6,)
            n_timesteps: Number of timesteps in trajectory
            dt: Time step duration (seconds)
            world_file: Path to Webots world file for simulation evaluation
            use_simulation: If True, evaluate trajectories in Webots; if False, use analytical model
            show_gui: If True, show Webots GUI during simulation (for debugging)
        """
        self.start_pose = start_pose
        self.end_pose = end_pose
        self.n_timesteps = n_timesteps
        self.dt = dt
        self.use_simulation = use_simulation
        self.run_name = run_name
        self.dynamics = UR5eSimpleDynamics()
        
        # Setup Webots simulator if enabled
        if use_simulation and world_file:
            self.simulator = WebotsSimulator(world_file, dt, headless=not show_gui, run_name=run_name)
            mode = "GUI" if show_gui else "headless"
            print(f"[OPTIMIZER] Webots simulation enabled ({mode}): {world_file}")
        else:
            self.simulator = None
            print("[OPTIMIZER] Using analytical dynamics model")
        
        # Find which joints actually change between start and end
        self.active_joints = np.where(np.abs(start_pose - end_pose) > 0.01)[0]
        self.n_active = len(self.active_joints)
        
        # Cache for simulation results (avoid re-simulating identical trajectories)
        self.simulation_cache = {}
        self.eval_count = 0
        
        print(f"Active joints for optimization: {self.active_joints}")
        print(f"  Start: {start_pose[self.active_joints]}")
        print(f"  End: {end_pose[self.active_joints]}")
    
    def _pack_decision_vars(self, q_traj: np.ndarray, qd_traj: np.ndarray) -> np.ndarray:
        """Pack trajectory into decision variable vector"""
        # Only optimize active joints
        q_active = q_traj[:, self.active_joints].flatten()
        qd_active = qd_traj[:, self.active_joints].flatten()
        return np.concatenate([q_active, qd_active])
    
    def _unpack_decision_vars(self, x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Unpack decision variables into full trajectories"""
        n_q = self.n_timesteps * self.n_active
        
        q_active = x[:n_q].reshape(self.n_timesteps, self.n_active)
        qd_active = x[n_q:].reshape(self.n_timesteps, self.n_active)
        
        # Create full trajectories with constant inactive joints
        q_traj = np.tile(self.start_pose, (self.n_timesteps, 1))
        qd_traj = np.zeros((self.n_timesteps, 6))
        
        q_traj[:, self.active_joints] = q_active
        qd_traj[:, self.active_joints] = qd_active
        
        return q_traj, qd_traj
    
    def _compute_accelerations(self, q_traj: np.ndarray, qd_traj: np.ndarray) -> np.ndarray:
        """Compute accelerations using finite differences"""
        qdd_traj = np.zeros_like(qd_traj)
        qdd_traj[1:] = (qd_traj[1:] - qd_traj[:-1]) / self.dt
        return qdd_traj
    
    def objective(self, x: np.ndarray) -> float:
        """Objective: minimize control effort and maximize release velocity (from simulation or model)"""
        self.eval_count += 1
        q_traj, qd_traj = self._unpack_decision_vars(x)
        qdd_traj = self._compute_accelerations(q_traj, qd_traj)
        
        # Use Webots simulation if enabled
        if self.simulator is not None:
            # Create cache key from trajectory
            cache_key = hash(x.tobytes())
            
            if cache_key in self.simulation_cache:
                sim_result = self.simulation_cache[cache_key]
            else:
                # Run simulation to evaluate this trajectory
                sim_result = self.simulator.evaluate_trajectory(
                    q_trajectory=q_traj,
                    initial_block_pos=np.array([0, 0, 0.5]),  # Approximate block position
                    gripper_close_time=130,
                    gripper_open_time=150 + 50 + len(q_traj),  # After pre-throw sequence
                    train_step=self.eval_count
                )
                self.simulation_cache[cache_key] = sim_result
                
                if self.eval_count % 10 == 0:
                    print(f"  [Eval {self.eval_count}] Sim result: velocity={sim_result['release_velocity']}, "
                          f"dropped={sim_result['block_dropped']}, distance={sim_result['final_distance']:.3f}m")
            
            # Penalize if block was dropped
            drop_penalty = 1000.0 if sim_result['block_dropped'] else 0.0
            
            # Reward high release velocity (use actual from simulation)
            release_vel = np.array(sim_result['release_velocity'])
            release_speed = np.linalg.norm(release_vel)
            velocity_cost = -10.0 * release_speed
            
            # Reward distance traveled
            distance_cost = -5.0 * sim_result['final_distance']
            
            # Still penalize control effort
            total_torque = 0.0
            for i in range(self.n_timesteps):
                tau = self.dynamics.inverse_dynamics(q_traj[i], qd_traj[i], qdd_traj[i])
                total_torque += np.sum(tau**2)
            effort_cost = 0.01 * total_torque
            
            return drop_penalty + effort_cost + velocity_cost + distance_cost
        
        else:
            # Fallback: analytical model
            # Compute required torques
            total_torque = 0.0
            for i in range(self.n_timesteps):
                tau = self.dynamics.inverse_dynamics(q_traj[i], qd_traj[i], qdd_traj[i])
                total_torque += np.sum(tau**2)
            
            # Penalize control effort
            effort_cost = 0.01 * total_torque
            
            # Reward high end-effector velocity at release (negative = maximize)
            ee_vel_cost = 0.0
            if self.n_timesteps > 1:
                # Approximate end-effector velocity at release
                ee_pos_final = self.dynamics.forward_kinematics(q_traj[-1])
                ee_pos_prev = self.dynamics.forward_kinematics(q_traj[-2])
                ee_vel = (ee_pos_final - ee_pos_prev) / self.dt
                ee_speed = np.linalg.norm(ee_vel)
                ee_vel_cost = -10.0 * ee_speed  # Maximize speed (negative cost)
            
            return effort_cost + ee_vel_cost
    
    def constraints(self, x: np.ndarray) -> np.ndarray:
        """Dynamics constraints (collocation)"""
        q_traj, qd_traj = self._unpack_decision_vars(x)
        
        violations = []
        
        # Position integration constraints: q[i+1] = q[i] + qd[i]*dt
        for i in range(self.n_timesteps - 1):
            q_expected = q_traj[i] + qd_traj[i] * self.dt
            violation = q_traj[i+1] - q_expected
            violations.extend(violation[self.active_joints])
        
        return np.array(violations)
    
    def inequality_constraints(self, x: np.ndarray) -> np.ndarray:
        """Inequality constraints (torque limits, velocity limits)"""
        q_traj, qd_traj = self._unpack_decision_vars(x)
        qdd_traj = self._compute_accelerations(q_traj, qd_traj)
        
        violations = []
        
        for i in range(self.n_timesteps):
            # Torque limits
            tau = self.dynamics.inverse_dynamics(q_traj[i], qd_traj[i], qdd_traj[i])
            tau_violation_upper = tau - self.dynamics.tau_max
            tau_violation_lower = -self.dynamics.tau_max - tau
            violations.extend(tau_violation_upper)
            violations.extend(tau_violation_lower)
            
            # Velocity limits
            qd_violation = np.abs(qd_traj[i]) - self.dynamics.qd_max
            violations.extend(qd_violation)
        
        return np.array(violations)
    
    def optimize(self, max_iter: int = 100) -> dict:
        """Run trajectory optimization"""
        print("\n" + "="*60)
        print("Starting Trajectory Optimization")
        print("="*60)
        
        # Initial guess: linear interpolation
        t_vec = np.linspace(0, 1, self.n_timesteps)
        q_init = np.outer(1 - t_vec, self.start_pose) + np.outer(t_vec, self.end_pose)
        
        # Initial velocities from finite differences
        qd_init = np.zeros((self.n_timesteps, 6))
        qd_init[1:] = (q_init[1:] - q_init[:-1]) / self.dt
        
        x0 = self._pack_decision_vars(q_init, qd_init)
        
        # Boundary constraints
        bounds = []
        for i in range(self.n_timesteps):
            for j in self.active_joints:
                # Position bounds
                if i == 0:
                    # Fix start
                    bounds.append((self.start_pose[j], self.start_pose[j]))
                elif i == self.n_timesteps - 1:
                    # Fix end
                    bounds.append((self.end_pose[j], self.end_pose[j]))
                else:
                    # Allow movement within joint limits
                    bounds.append((self.dynamics.q_min[j], self.dynamics.q_max[j]))
        
        # Velocity bounds
        for i in range(self.n_timesteps):
            for j in self.active_joints:
                bounds.append((-self.dynamics.qd_max[j], self.dynamics.qd_max[j]))
        
        # Define constraint dictionaries
        eq_constraints = {
            'type': 'eq',
            'fun': self.constraints
        }
        
        ineq_constraints = {
            'type': 'ineq',
            'fun': lambda x: -self.inequality_constraints(x)  # ineq: g(x) >= 0
        }
        
        # Optimize
        print(f"\nOptimizing {self.n_active} active joints over {self.n_timesteps} timesteps...")
        print(f"Decision variables: {len(x0)}")
        
        result = minimize(
            self.objective,
            x0,
            method='SLSQP',
            bounds=bounds,
            constraints=[eq_constraints, ineq_constraints],
            options={'maxiter': max_iter, 'disp': True, 'ftol': 1e-6}
        )
        
        print("\n" + "="*60)
        print("Optimization Complete")
        print("="*60)
        print(f"Success: {result.success}")
        print(f"Message: {result.message}")
        print(f"Final cost: {result.fun:.6f}")
        print(f"Iterations: {result.nit}")
        
        # Unpack solution
        q_traj, qd_traj = self._unpack_decision_vars(result.x)
        
        return {
            'success': result.success,
            'q_trajectory': q_traj,
            'qd_trajectory': qd_traj,
            'cost': result.fun,
            'result': result
        }
