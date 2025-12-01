# Learned Thrower Controller

This controller uses trajectory optimization to learn an optimal throwing motion for the UR5e robot arm.

## Training

### Command Line Flags

Run training with:
```bash
python train_policy.py [OPTIONS]
```

**Available flags:**

- `--gui` - Show Webots GUI during training (slower, but useful for debugging)
  - Default: headless mode (no GUI, faster)
  - Example: `python train_policy.py --gui`

- `--no-sim` - Use analytical dynamics model instead of full simulation
  - Default: uses Webots simulation (accurate but slow)
  - Analytical model is much faster but less accurate
  - Example: `python train_policy.py --no-sim`

- `--run-name NAME` - Name for this training run (used in log files)
  - Default: `run0`
  - Example: `python train_policy.py --run-name experiment1`

**Example commands:**
```bash
# Standard training (simulation, headless)
python train_policy.py

# Training with visualization
python train_policy.py --gui

# Fast testing with analytical model
python train_policy.py --no-sim

# Named run with GUI
python train_policy.py --gui --run-name test_run_1
```

## Log Files and Outputs

### Training Outputs

**`trajectory_archive/`**
- Location for all trajectory evaluation logs during training
- One JSONL file per training run

**`trajectory_archive/eval_traj_<run_name>.jsonl`**
- JSONL format (one JSON object per line)
- Each line represents one trajectory candidate tested during optimization
- Fields:
  - `trajectory`: Array of joint angles at each timestep [[q1,q2,q3,q4,q5,q6], ...]
  - `gripper_close_time`: Timestep when gripper closes (default: 130)
  - `gripper_open_time`: Timestep when gripper opens/releases block (default: 200)
  - `timestamp`: When this trajectory was evaluated (YYYYMMDD_HHMMSS)
  - `run_name`: Name of the training run
  - `train_step`: Optimization iteration number

### Evaluation Results

**`../eval_thrower_controller/eval_results/`**
- Results from running each candidate trajectory in simulation

**`../eval_thrower_controller/eval_results/eval_results_<run_name>.jsonl`**
- JSONL format with results for each evaluated trajectory
- Fields:
  - `run_name`: Training run name
  - `train_step`: Optimization iteration
  - `success`: Boolean - whether trajectory succeeded
  - `block_dropped`: Boolean - whether block was dropped during throw
  - `block_dropped_time`: Timestep when drop occurred (or 'N/A')
  - `release_velocity`: [vx, vy, vz] - block velocity at release
  - `final_distance`: Distance block traveled (meters)
  - `position_at_checkpoint`: Block position when passing checkpoint
  - `final_arm_pos`: Final arm end-effector position
  - `final_block_pos`: Final block position

**`../eval_thrower_controller/eval_data.json`**
- Temporary file containing current trajectory being evaluated
- Overwritten for each evaluation
- Used by eval_thrower_controller to load trajectory during simulation

**`../eval_thrower_controller/eval_result.json`**
- Temporary file containing results of most recent evaluation
- Read by optimizer, then deleted
- Same format as entries in eval_results_*.jsonl

## Training Process

1. **Initialization**: Optimizer creates initial trajectory (linear interpolation between start/end poses)

2. **Evaluation Loop** (repeated for MAX_ITER iterations):
   - Generate candidate trajectory based on current optimization state
   - Save trajectory to `trajectory_archive/eval_traj_<run_name>.jsonl` (append)
   - Write trajectory to `../eval_thrower_controller/eval_data.json`
   - Launch Webots with train.wbt world
   - eval_thrower_controller reads eval_data.json and executes trajectory
   - Results written to `../eval_thrower_controller/eval_result.json`
   - Optimizer reads results and updates trajectory
   - Results archived to `../eval_thrower_controller/eval_results/eval_results_<run_name>.jsonl`

3. **Completion**:
   - Best trajectory saved to `policy_weights/learned_policy.pkl`
   - Training summary printed to console

## Analyzing Results

### View all tested trajectories for a run:
```powershell
Get-Content trajectory_archive/eval_traj_run0.jsonl
```

### View all evaluation results for a run:
```powershell
Get-Content ../eval_thrower_controller/eval_results/eval_results_run0.jsonl
```

### Count successful throws:
```powershell
(Get-Content ../eval_thrower_controller/eval_results/eval_results_run0.jsonl | ConvertFrom-Json | Where-Object {$_.success -eq $true}).Count
```

### Find best throw distance:
```powershell
Get-Content ../eval_thrower_controller/eval_results/eval_results_run0.jsonl | ConvertFrom-Json | Sort-Object final_distance -Descending | Select-Object -First 1
```

### Extract specific trajectory for replay:
```powershell
# Get trajectory from iteration 10
Get-Content trajectory_archive/eval_traj_run0.jsonl | Select-Object -Index 9 | Out-File ../eval_thrower_controller/eval_data.json -Encoding utf8

# Then run eval_thrower_controller in Webots to replay it
```

## File Structure

```
learned_thrower_controller/
├── learned_thrower_controller.py  # Main execution controller (loads final policy)
├── train_policy.py                # Training script (run this to train)
├── trajectory_optimizer.py        # Optimization logic (SLSQP)
├── webots_simulator.py            # Simulation interface
├── trajectory_archive/            # All tested trajectories
│   └── eval_traj_<run_name>.jsonl
└── README.md                      # This file

eval_thrower_controller/
├── eval_thrower_controller.py     # Evaluation controller (runs during training)
├── eval_data.json                 # Current trajectory being tested
├── eval_result.json               # Latest evaluation result
└── eval_results/                  # Archived results
    └── eval_results_<run_name>.jsonl
```

## Notes

- Training with simulation (`--gui` or default) uses real Webots physics for accurate evaluation
- Analytical model (`--no-sim`) is faster but less accurate - good for quick testing
- Each training run should use a unique `--run-name` to avoid mixing logs
- JSONL format allows efficient appending without rewriting entire file
- All print statements from eval_thrower_controller appear in training console
