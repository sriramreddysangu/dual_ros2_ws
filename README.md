# Dual Arm Synchronization System

A ROS 2 package for synchronized control of dual robotic arms using B-spline trajectory generation, Kuramoto synchronization model, and Iterative Learning Control (ILC).

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Dual Arm Sync System                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────────┐      ┌──────────────────┐               │
│  │  Trajectory      │─────>│   Kuramoto       │               │
│  │  Generator       │      │   Sync           │               │
│  │  (B-splines)     │      │  (Phase Sync)    │               │
│  └──────────────────┘      └────────┬─────────┘               │
│           │                          │                          │
│           v                          v                          │
│  ┌──────────────────────────────────────────┐                  │
│  │         ILC Controller                    │                  │
│  │    (Iterative Learning)                   │                  │
│  └────────────────┬─────────────────────────┘                  │
│                   │                                             │
│                   v                                             │
│  ┌──────────────────────────────────────────┐                  │
│  │        Publisher Node                     │                  │
│  │    (Command Distribution)                 │                  │
│  └────────┬─────────────────────┬───────────┘                  │
│           │                     │                               │
│           v                     v                               │
│  ┌───────────────┐     ┌───────────────┐                       │
│  │   Robot 1     │     │   Robot 2     │                       │
│  │  (dsr01)      │     │  (dsr02)      │                       │
│  └───────────────┘     └───────────────┘                       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Features

- **B-Spline Trajectory Generation**: Smooth, continuous trajectories using cubic B-splines
- **Kuramoto Synchronization**: Phase-based synchronization between dual arms
- **Iterative Learning Control**: Improves tracking performance over repeated trials
- **Modular Design**: Separate launch files for Gazebo, RViz, MoveIt, and synchronization
- **Cross-Workspace Support**: Works with dsr_description2 from separate workspace

## Package Structure

```
dual_arm_ws/
├── src/
│   ├── dual_arm_sync/
│   │   ├── dual_arm_sync/
│   │   │   ├── __init__.py
│   │   │   ├── trajectory_generator.py    # B-spline trajectory generation
│   │   │   ├── kuramoto_sync.py           # Kuramoto synchronization
│   │   │   ├── ilc_controller.py          # Iterative Learning Control
│   │   │   └── publisher_node.py          # Command publisher
│   │   ├── launch/
│   │   │   ├── dual_gazebo.launch.py      # Gazebo simulation
│   │   │   ├── dual_rviz.launch.py        # RViz visualization
│   │   │   ├── dual_moveit.launch.py      # MoveIt motion planning
│   │   │   └── dual_sync_system.launch.py # Complete sync system
│   │   ├── rviz/
│   │   │   └── dual_arm.rviz              # RViz configuration
│   │   ├── worlds/                         # Gazebo world files
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
```

## Installation

### Prerequisites

1. ROS 2 (Humble or later)
2. Python packages: numpy, scipy
3. `ros2_ws` with dsr_description2, dsr_bringup2, dsr_gazebo2, dsr_controller2

### Install Python Dependencies

```bash
pip3 install numpy scipy
```

### Build the Package

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Source ros2_ws (contains dsr packages)
source ~/ros2_ws/install/setup.bash

# Build dual_arm_ws
cd ~/dual_arm_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### Quick Start (All-in-One)

Launch everything (Gazebo + RViz + MoveIt + Sync System):

```bash
# Terminal 1: Gazebo
ros2 launch dual_arm_sync dual_gazebo.launch.py

# Terminal 2: RViz (optional)
ros2 launch dual_arm_sync dual_rviz.launch.py

# Terminal 3: MoveIt (optional)
ros2 launch dual_arm_sync dual_moveit.launch.py

# Terminal 4: Synchronization System
ros2 launch dual_arm_sync dual_sync_system.launch.py
```

### Individual Component Launch

#### 1. Gazebo Only
```bash
ros2 launch dual_arm_sync dual_gazebo.launch.py \
    color1:=white color2:=blue \
    x1:=0.0 y1:=0.5 \
    x2:=0.0 y2:=-0.5
```

#### 2. RViz Visualization
```bash
ros2 launch dual_arm_sync dual_rviz.launch.py \
    color1:=white color2:=blue
```

#### 3. MoveIt Motion Planning
```bash
ros2 launch dual_arm_sync dual_moveit.launch.py \
    model1:=m1013 model2:=m1013
```

#### 4. Synchronization System Only
```bash
ros2 launch dual_arm_sync dual_sync_system.launch.py \
    trajectory_duration:=10.0 \
    control_frequency:=100.0 \
    learning_rate:=0.5 \
    coupling_strength:=2.0
```

### Running Individual Nodes

#### Trajectory Generator
```bash
ros2 run dual_arm_sync trajectory_generator \
    --ros-args \
    -p robot1_namespace:=dsr01 \
    -p robot2_namespace:=dsr02 \
    -p trajectory_duration:=10.0
```

#### Kuramoto Synchronization
```bash
ros2 run dual_arm_sync kuramoto_sync \
    --ros-args \
    -p coupling_strength:=2.0 \
    -p natural_frequency:=1.0
```

#### ILC Controller
```bash
ros2 run dual_arm_sync ilc_controller \
    --ros-args \
    -p learning_rate:=0.5 \
    -p max_iterations:=50
```

#### Publisher Node
```bash
ros2 run dual_arm_sync publisher_node \
    --ros-args \
    -p use_ilc:=true
```

## Parameter Configuration

### Trajectory Generator Parameters
- `robot1_namespace`: Namespace for robot 1 (default: 'dsr01')
- `robot2_namespace`: Namespace for robot 2 (default: 'dsr02')
- `num_joints`: Number of joints (default: 6)
- `trajectory_duration`: Duration of trajectory cycle (default: 10.0 seconds)
- `control_frequency`: Control update frequency (default: 100.0 Hz)
- `spline_order`: B-spline order (default: 3 for cubic)
- `num_control_points`: Number of control points (default: 5)

### Kuramoto Synchronization Parameters
- `coupling_strength`: Kuramoto coupling strength K (default: 2.0)
- `natural_frequency`: Natural frequency ω (default: 1.0 rad/s)
- `sync_frequency`: Synchronization update frequency (default: 100.0 Hz)
- `phase_tolerance`: Phase synchronization tolerance (default: 0.1 rad)

### ILC Controller Parameters
- `learning_rate`: ILC learning rate γ (default: 0.5)
- `max_iterations`: Maximum learning iterations (default: 50)
- `convergence_threshold`: RMSE threshold for convergence (default: 0.01)

### Publisher Node Parameters
- `use_ilc`: Enable ILC correction (default: true)
- `command_timeout`: Command validity timeout (default: 0.1 seconds)
- `position_tolerance`: Position tracking tolerance (default: 0.01 rad)

## Topics

### Published Topics
- `/<robot_ns>/joint_trajectory` - Full trajectory for robot
- `/<robot_ns>/desired_joint_positions` - Current desired positions
- `/<robot_ns>/synchronized_positions` - Kuramoto-synchronized positions
- `/<robot_ns>/ilc_corrected_positions` - ILC-corrected positions
- `/<robot_ns>/dsr_controller2/commands` - Final commands to controller
- `/synchronization_error` - Phase synchronization error
- `/ilc_iteration` - Current ILC iteration number
- `/ilc_rmse` - ILC RMSE values

### Subscribed Topics
- `/<robot_ns>/joint_states` - Current joint states from robot

## Monitoring and Debugging

### Monitor Synchronization Error
```bash
ros2 topic echo /synchronization_error
```

### Monitor ILC Progress
```bash
ros2 topic echo /ilc_iteration
ros2 topic echo /ilc_rmse
```

### Monitor Joint States
```bash
ros2 topic echo /dsr01/joint_states
ros2 topic echo /dsr02/joint_states
```

### Visualize with RQT
```bash
rqt_plot /synchronization_error/data[0] /synchronization_error/data[1]
```

## Troubleshooting

### Issue: Robots not spawning in Gazebo
**Solution**: Make sure ros2_ws is sourced before dual_arm_ws
```bash
source ~/ros2_ws/install/setup.bash
source ~/dual_arm_ws/install/setup.bash
```

### Issue: RViz shows no robot models
**Solution**: Check that dual_arm.rviz is installed
```bash
ls ~/dual_arm_ws/install/dual_arm_sync/share/dual_arm_sync/rviz/
```

### Issue: ILC not converging
**Solution**: Adjust learning rate or increase max iterations
```bash
ros2 launch dual_arm_sync dual_sync_system.launch.py \
    learning_rate:=0.3 \
    max_iterations:=100
```

### Issue: Synchronization oscillating
**Solution**: Reduce coupling strength
```bash
ros2 launch dual_arm_sync dual_sync_system.launch.py \
    coupling_strength:=1.0
```

## Development

### Adding Custom Trajectories

Edit `trajectory_generator.py` and modify the `generate_control_points()` method:

```python
def generate_control_points(self, start_pos, end_pos, num_points, joint_idx):
    # Add your custom control point generation logic here
    pass
```

### Tuning Kuramoto Parameters

The coupling strength K affects synchronization speed:
- Higher K (e.g., 5.0): Faster synchronization but may oscillate
- Lower K (e.g., 0.5): Slower but smoother synchronization

### Tuning ILC Parameters

The learning rate γ affects learning speed:
- Higher γ (e.g., 0.8): Faster learning but may overshoot
- Lower γ (e.g., 0.2): Slower but more stable learning

## Mathematical Background

### B-Spline Trajectory
Position: `p(t) = Σ Nᵢ,ₖ(t) * Pᵢ`
where Nᵢ,ₖ are B-spline basis functions

### Kuramoto Model
`dθᵢ/dt = ωᵢ + K * Σⱼ sin(θⱼ - θᵢ)`
where θ is phase, ω is natural frequency, K is coupling

### ILC Update Law
`uₖ₊₁(t) = uₖ(t) + γ * eₖ(t)`
where u is control input, γ is learning rate, e is error

## Citation

If you use this package in your research, please cite:

```bibtex
@software{dual_arm_sync,
  title={Dual Arm Synchronization with B-splines, Kuramoto, and ILC},
  author={Your Name},
  year={2024},
  url={https://github.com/yourusername/dual_arm_sync}
}
```

## License

BSD License - See LICENSE file for details

## Contact

For questions and support:
- Email: your_email@example.com
- Issues: GitHub Issues page

## Acknowledgments

- Based on dsr_description2 from Doosan Robotics
- Kuramoto synchronization theory
- Iterative Learning Control principles
