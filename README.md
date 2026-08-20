# Differential-drive trajectory-control comparison

ROS 2 and Gazebo Classic implementation of a timed trajectory-tracking study
for a differential-drive mobile robot. The project compares three controllers
through the same state estimate, reference trajectory, actuator limits,
completion rule, perturbation paths, and evaluator:

- cascaded PID;
- finite-horizon time-varying LQR (called LQR in reports);
- constrained linear time-varying MPC solved with OSQP.

The repository contains everything needed to build and run the software: C++
sources, ROS packages, robot/world models, controller configurations, benchmark
tracks, launch files, unit tests, experiment scripts, and a development-container
definition. Generated build trees, raw experiment results, and local thesis
materials are intentionally not versioned.

## Recommended environment

The reference development environment is based on Ubuntu 22.04, ROS 2 Humble,
Gazebo Classic 11, and C++17. The simplest reproducible setup uses Docker and
the supplied [development container](.devcontainer/README.md).

### VS Code Dev Containers on Windows/WSL2

1. Install Docker Desktop, WSL2, Visual Studio Code, and the Dev Containers
   extension.
2. Clone the repository inside the WSL filesystem.
3. Open the repository in VS Code and select **Dev Containers: Reopen in
   Container**.
4. Wait for `rosdep` to install the package dependencies.

The checked-in devcontainer forwards WSLg/X11 and GPU devices for Gazebo and
RViz. On a host without WSLg, use headless mode (`gui:=false`) or adapt the GUI
socket mounts to that host's display system.

### Plain Docker, including headless hosts

From the repository root:

```bash
docker build -f .devcontainer/Dockerfile -t diff-drive-thesis .
docker run --rm -it --net=host --privileged \
  -v "$(pwd):/home/ws" \
  diff-drive-thesis bash
```

Inside the container, install any dependency newly declared in `package.xml`
and build the overlay:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

The campaign scripts intentionally assume that the repository is mounted at
`/home/ws`; the ROS launch files themselves locate installed assets through the
ament index and do not depend on the checkout location.

## Native ROS 2 installation

An Ubuntu 22.04 system with ROS 2 Humble can build the project without Docker.
After installing ROS 2 Humble and `python3-rosdep`:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

The analysis and final-figure scripts additionally use Python 3, PyYAML, and
Matplotlib. They are installed by the Dockerfile as `python3-yaml` and
`python3-matplotlib`. Thesis figures are exported as vector PDF files and
matching 300 dpi PNG previews.

## Running the simulation

Open two terminals in the built environment and source both ROS layers in each:

```bash
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash
```

In terminal 1, start Gazebo, the robot, wheel/IMU simulation, EKF, and RViz:

```bash
ros2 launch my_robot_description display.launch.py gui:=true
```

For a machine without graphical forwarding:

```bash
ros2 launch my_robot_description display.launch.py gui:=false
```

In terminal 2, start exactly one controller:

```bash
# Cascaded PID
ros2 launch my_robot_controller pid.launch.py

# LQR
ros2 launch my_robot_controller lqr.launch.py

# MPC
ros2 launch my_robot_controller mpc.launch.py
```

The default launch uses the straight track. A different installed track can be
selected explicitly, for example:

```bash
ros2 launch my_robot_controller pid.launch.py \
  csv_path:=/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_5_figure_eight.csv
```

Controller and evaluator CSV files are written to the current terminal
directory unless absolute output paths are supplied. Do not run more than one
controller launch at the same time because they publish to the same robot.

## Perturbation and comparison experiments

The perturbation suite accepts the same scenario definitions for PID, LQR, and
MPC. A short headless example is:

```bash
PERTURBATION_CONTROLLER_FAMILY=pid \
PERTURBATION_SCENARIOS="nominal angular_pulse_train left_wheel_loss_persistent" \
PERTURBATION_GUI=false \
  bash scripts/run_pid_perturbation_suite.sh /home/ws/results/pid_smoke
```

The complete paired comparison is launched with:

```bash
bash scripts/run_controller_comparison.sh /home/ws/results/controller_comparison
```

This is a long campaign. For a smoke test, reduce it without editing source:

```bash
COMPARISON_REPETITIONS=1 \
COMPARISON_NOMINAL_TRACKS="straight" \
COMPARISON_ROBUSTNESS_SCENARIOS="nominal" \
  bash scripts/run_controller_comparison.sh /home/ws/results/comparison_smoke
```

Raw results are ignored by Git because they are large, generated artifacts.
Each formal campaign writes its resolved protocol, source/configuration
fingerprints, per-run logs, summaries, statistical tables, and audit findings
inside its chosen result directory.

## Verification

Run the project consistency audit before an experiment:

```bash
python3 scripts/audit_project_consistency.py
```

Build and execute the ROS/C++ tests and linters:

```bash
colcon build --symlink-install \
  --packages-select my_robot_description my_robot_controller
source install/setup.bash
colcon test --packages-select my_robot_description my_robot_controller
colcon test-result --verbose
```

Run the dependency-free Python analysis tests:

```bash
python3 -m unittest discover -s scripts -p 'test_*.py'
```

Regenerate the seven deterministic benchmark CSV files from the repository root
with `python3 generate_tracks.py`. The consistency audit verifies their format,
spacing, smoothness assumptions, and correspondence with the installed project.

## Repository layout

```text
src/my_robot_description/   robot URDF, Gazebo worlds/plugins, EKF and RViz
src/my_robot_controller/    PID, LQR, MPC, reference, faults, evaluator, tests
scripts/                    experiment orchestration and statistical analysis
track_*.csv                 deterministic benchmark paths installed by CMake
protocols/                  frozen experiment-protocol notes
.devcontainer/              reproducible ROS/Gazebo development image
```

The project is distributed under the Apache License 2.0; see [LICENSE](LICENSE).
