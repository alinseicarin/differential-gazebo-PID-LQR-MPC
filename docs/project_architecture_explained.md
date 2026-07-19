# Project Architecture and First PID Milestone

This document explains the current ROS 2 and Gazebo project from a control-
engineering perspective. It assumes familiarity with control theory and MATLAB,
but not with C++, ROS 2, or software-project structure.

## 1. System overview

The project is a closed-loop simulation with the following signal flow:

```text
Reference path
(track CSV)
     |
     v
PID path-following node
calculates v and omega
     |
     | /cmd_vel
     v
Gazebo differential-drive robot
(the simulated plant)
     |
     +-- wheel odometry --+
     +-- IMU -------------+
                          v
                         EKF
                   state estimation
                          |
                          | /odometry/filtered
                          v
                 PID path-following node
```

In control terminology:

- The track CSV is the reference.
- The C++ PID node is the controller.
- Gazebo and the robot model are the plant.
- Wheel odometry and the IMU are the sensors.
- The EKF is the state estimator.
- `/cmd_vel` is the control input: linear velocity `v` and angular velocity
  `omega`.
- `/odometry/filtered` is the estimated feedback state: `x`, `y`, and heading
  `theta`.
- RViz is only a display and does not affect the control loop.

## 2. What happens during a simulation

1. Gazebo creates the simulated world.
2. Gazebo loads the robot from the URDF model.
3. The differential-drive plugin waits for velocity commands.
4. Gazebo generates wheel odometry and simulated IMU measurements.
5. The EKF combines those measurements into an estimated robot pose.
6. The PID controller loads the desired path from a CSV file.
7. Whenever new estimated odometry arrives, the controller finds the robot's
   location on the path, selects a point ahead, calculates errors, calculates
   `v` and `omega`, sends those commands to Gazebo, and records the sample.
8. When the robot reaches the endpoint, the controller commands zero velocity
   and flushes the output CSV.

The feedback loop normally updates at approximately 30 Hz in simulation time.

## 3. Current PID control law

The current controller does not apply independent PID controllers to global
`x` and `y`. A differential-drive robot cannot command independent global x and
y velocities.

Instead, it uses two PID controllers:

```text
v_raw = Kp_v e_d + Ki_v integral(e_d dt) + Kd_v de_d/dt

omega = Kp_omega e_theta
      + Ki_omega integral(e_theta dt)
      + Kd_omega de_theta/dt
```

where:

- `e_d` is the distance to a lookahead waypoint.
- `e_theta` is the angular error between the robot heading and the direction
  toward that waypoint.
- `v` is commanded forward velocity.
- `omega` is commanded yaw rate.

Forward speed is reduced when the robot is not pointing toward the target:

```text
v = v_raw max(0, cos(e_theta))
```

Therefore:

- The robot moves normally when it points toward the target.
- It slows as heading error increases.
- At a heading error of 90 degrees or more, it stops translating and turns in
  place.

Commands are then limited to:

```text
0 <= v <= 1.0 m/s
-1.5 <= omega <= 1.5 rad/s
```

### Important classification

Cross-track error is calculated and logged, but is not directly fed into the
current PID law. Steering uses the bearing to a lookahead point. The most
accurate description is therefore:

> A waypoint-based PID path-following controller using longitudinal distance
> error and lookahead heading error.

It is a PID controller, but it is not the same as a classical lateral-error PID
controller.

## 4. How the current path follower works

The reference path is a sequence of headerless `x,y` points.

### 4.1 Find the current path location

The controller searches forward through a limited number of waypoints and
finds the closest one. It remembers path progress so it does not jump backward
to a passed point or choose the wrong branch at a path intersection.

### 4.2 Project onto the path

The controller calculates the closest point on the current line segment. This
provides geometric cross-track error instead of comparing the robot with an
equally numbered CSV row, which would be invalid when reference and robot do not
advance at identical rates.

The signed cross-track error indicates which side of the path contains the
robot.

### 4.3 Select a lookahead point

The controller aims five waypoints ahead. Aiming ahead is smoother than steering
toward the closest point, which may already be beside or behind the robot.

### 4.4 Calculate errors

Distance to the lookahead point drives the linear PID. Bearing to the lookahead
point minus current yaw drives the angular PID.

Angular error is wrapped into `[-pi, pi]`. Otherwise, changing from +179 degrees
to -179 degrees would appear to be a 358-degree error instead of a 2-degree
error.

### 4.5 Detect completion

The robot finishes only when it has progressed into the final portion of the
path and is within 8 cm of the endpoint. Requiring path progress prevents false
completion when a closed or intersecting path ends near its start.

## 5. Controller improvements made during the first milestone

The main controller files are:

- `src/my_robot_controller/include/my_robot_controller/pid_node.hpp`
- `src/my_robot_controller/src/pid_node.cpp`

The `.hpp` file declares the classes, functions, remembered variables, and ROS
interfaces. The `.cpp` file contains the implementation.

### 5.1 Deterministic initialization

All pose, PID-memory, waypoint, timing, completion, and watchdog variables are
explicitly initialized. In C++, an uninitialized primitive variable may contain
unpredictable memory data, which can make runs inconsistent.

### 5.2 Safe CSV loading

The controller requires a valid input path, at least two waypoints, exactly two
columns per row, and finite numeric values. It refuses to start with missing,
empty, malformed, or inappropriate data rather than driving from a partial
path.

### 5.3 Configurable paths

Input and output locations are ROS parameters:

```text
csv_path
output_csv_path
```

The controller no longer relies on one hard-coded workspace location.

### 5.4 Simulation-time PID calculations

The PID sample time is calculated from odometry message timestamps:

```text
dt = t_simulation,new - t_simulation,old
```

This keeps integral and derivative calculations correct if Gazebo runs slower
or faster than real time.

### 5.5 Odometry-driven control

One control calculation is performed for every new filtered odometry message:

```text
new state estimate -> calculate errors -> PID -> publish command
```

The controller does not calculate repeated commands from an unchanged state.

### 5.6 Derivative startup protection

The first derivative contribution after initialization or reset is zero because
no previous error sample exists. This prevents an artificial derivative kick.

### 5.7 Integral anti-windup

Integral memory is limited:

```text
-I_max <= integral(e dt) <= I_max
```

This prevents large stored integral action when the robot is stuck or an output
is saturated.

### 5.8 Timing-discontinuity handling

The controller rejects non-increasing timestamps. After an abnormally long
odometry interval, it resets PID memory rather than calculating misleading
integral and derivative terms.

### 5.9 Odometry watchdog

If filtered odometry disappears for more than two real seconds, the controller
commands zero velocity. The watchdog uses a monotonic wall clock so it still
works if Gazebo pauses or simulation time disappears.

### 5.10 Safe stopping

Zero velocity is explicitly published at track completion, after odometry
timeout, during orderly shutdown, and after invalid estimator output.

## 6. Robot model changes

The robot model is in `src/my_robot_description/urdf/my_robot.urdf`. URDF defines
robot geometry, mass and inertia, joints, sensors, and Gazebo plugins.

### 6.1 Consistent wheel geometry

The wheel centers are separated by 0.35 m and the wheels have diameter 0.20 m.
The differential-drive plugin now uses those same values.

This is required by the differential-drive kinematics:

```text
v = r/2 (omega_R + omega_L)
theta_dot = r/L (omega_R - omega_L)
```

where `r` is wheel radius and `L` is wheel separation. Plugin/model mismatch
would create inconsistent simulated rotation and odometry.

### 6.2 Single transform ownership

ROS TF describes coordinate-frame relationships such as:

```text
odom -> base_footprint -> base_link -> wheels
```

Now:

- Gazebo publishes raw `/odom` data but not the odometry TF.
- The EKF alone publishes filtered `odom -> base_footprint`.
- `robot_state_publisher` publishes body, wheel, and sensor transforms.

This avoids competing publishers for the same transform.

### 6.3 Simulated joint states

Gazebo publishes wheel angles on `/joint_states`. `robot_state_publisher` uses
them to publish wheel transforms and show rotating wheels.

### 6.4 Removed inappropriate GUI joint-state publisher

The GUI-oriented publisher used for manually manipulating a static URDF is no
longer used as a simulation data source. Gazebo owns the simulated joint state.

## 7. EKF architecture

The EKF configuration is `src/my_robot_description/config/ekf.yaml`.

It receives:

- `/odom` from the Gazebo differential-drive plugin.
- `/demo/imu` from the simulated IMU.

It produces:

- `/odometry/filtered`.
- The `odom -> base_footprint` transform.

The EKF uses 2D mode, so the controller works with `x`, `y`, and `theta` rather
than vertical position, roll, and pitch.

Both sensor sources are simulated. This supports architecture development, but
does not automatically reproduce every physical effect such as wheel slip,
bias, latency, calibration error, or nonlinear actuator behavior.

## 8. Launch files

### 8.1 Simulation launch

`src/my_robot_description/launch/display.launch.py` starts Gazebo, the selected
world, the robot, the EKF, `robot_state_publisher`, and optionally RViz.

Important arguments include:

```text
world:=empty.world
gui:=true
```

With `gui:=false`, physics and ROS continue without rendering graphical windows.
This headless mode is appropriate for automated experiments.

### 8.2 Controller launch

`src/my_robot_controller/launch/pid.launch.py` starts the PID controller and
accepts:

```text
csv_path
output_csv_path
config_path
use_sim_time
```

A track or tuning profile can therefore be changed without recompiling C++.

## 9. PID tuning profiles

The named profiles are:

- `pid_baseline.yaml`
- `pid_fast.yaml`
- `pid_robust.yaml`

`pid.yaml` remains as a compatibility/default copy, while the launch file uses
the explicitly named baseline profile by default.

### 9.1 Baseline

```text
Linear:  Kp = 1.0, Ki = 0.1, Kd = 0.2
Angular: Kp = 1.5, Ki = 0.0, Kd = 0.3
```

This is the control-group tuning.

### 9.2 Fast

The fast profile uses greater proportional response, slightly more linear
integral action, and less derivative damping. It tends to finish faster at the
cost of more path error and control effort.

### 9.3 Robust

The robust profile uses gentler longitudinal response, more angular derivative
damping, a small angular integral term, and tighter angular anti-windup. It
tends to follow curves more accurately but considerably more slowly.

### 9.4 Where tuning belongs

Normal tuning changes should be made only in YAML. Values in C++ are fallback
defaults; launch-loaded YAML overrides them. The C++ describes the algorithm,
while YAML describes an experiment's tuning.

## 10. Experiment logging

Every control update records:

```text
time
actual_x
actual_y
actual_yaw
reference_x
reference_y
reference_yaw
cross_track_error
path_heading_error
target_heading_error
linear_command
angular_command
waypoint_index
```

This supports MATLAB plots of desired versus actual trajectory, cross-track and
heading errors, velocity commands, completion time, and control effort.

Logged `reference_x` and `reference_y` are the robot's projection onto the path,
not a same-numbered waypoint. The full original reference remains available in
the input track CSV.

## 11. Automated benchmarks

`scripts/run_pid_benchmarks.sh` runs three profiles on five nominal paths:

```text
3 PID profiles x 5 tracks = 15 experiments
```

The paths are a 5 m straight line, sinusoidal curve, 90-degree corner, circle,
and figure-eight. The circle tests sustained curvature and closed-path
completion. The figure-eight tests steering reversal, intersection handling,
and closed-path completion.

For every trial, the script starts a fresh headless simulation, waits for the
robot and EKF, starts the selected controller, waits for completion or timeout,
saves the trajectory, stops all processes, calculates metrics, and then starts
a clean simulation for the next trial. A clean start prevents previous robot or
EKF state from contaminating another experiment.

### 11.1 Reported metrics

- Success or failure.
- Completion time.
- Final position error.
- Cross-track RMSE and maximum absolute cross-track error.
- Heading-error RMSE and maximum absolute heading error.
- Control effort.
- Maximum commanded `v` and `omega`.
- Final position and number of samples.

Current control effort is defined as:

```text
J_u = integral(v^2 + 0.2 omega^2) dt
```

The factor 0.2 is an experiment-design weighting, not a physical constant. The
same definition must be retained for PID, LQR, and MPC comparisons.

### 11.2 Preliminary single-run observations

- Fast was approximately 26% faster than baseline.
- Fast had approximately 37% more curve/corner cross-track error.
- Fast used approximately 46% more control effort.
- Robust reduced curve/corner cross-track error by approximately 41%.
- Robust used slightly less control effort.
- Robust was approximately 61% slower.
- All nine trials in the earlier three-track matrix reached the endpoint within
  the 8 cm tolerance. The circle and figure-eight were added afterward and must
  be reported separately until their trial matrix has been run.

These are preliminary observations, not statistically strong thesis results.
Final experiments should use repeated runs and report statistics such as mean,
standard deviation, and possibly confidence intervals.

## 12. Worlds and tracks

The worlds are:

```text
empty.world
incline.world
rough.world
```

`empty.world` is the flat nominal environment. The current `rough.world`
contains raised boxes that behave as steps or obstacles and can throw the robot
away from the path. It is therefore excluded from the default benchmark because
it does not represent a controlled, interpretable disturbance.

A more useful robustness environment could introduce low-amplitude surface
irregularities, a mild slope, reduced-friction regions, asymmetric wheel
friction, small external disturbances, or controlled parameter mismatch in
mass, inertia, wheel radius, or wheel separation.

## 13. Directory structure

```text
my_robotics_ws/
|-- src/
|   |-- my_robot_controller/
|   `-- my_robot_description/
|-- scripts/
|-- track_*.csv
|-- generate_tracks.py
|-- build/
|-- install/
`-- log/
```

### `src/my_robot_controller`

Contains the C++ PID algorithm, declarations, tuning files, and controller
launch file.

### `src/my_robot_description`

Contains robot geometry and physics, sensors, the differential-drive plugin,
EKF configuration, worlds, RViz settings, and simulation launch file.

### `scripts`

Contains experiment automation.

### Track CSV files

Contain the reference paths.

### `generate_tracks.py`

Generates reference-track coordinates. Python is not required during controller
execution; it is only used to prepare data.

### `build`, `install`, and `log`

These are generated by `colcon build`:

- `src` is editable source.
- `build` contains intermediate compiler work.
- `install` contains the runnable compiled packages.
- `log` contains build logs.

Edit source, run `colcon build`, source the installed workspace, and then launch
the installed version.

### `CMakeLists.txt`

This is the build recipe. It identifies source files, required ROS libraries,
the executable to create, and data files to install.

### `package.xml`

This contains ROS package identity and dependency metadata rather than control
logic.

### `.devcontainer`

This describes the Docker development environment, including ROS Humble,
Gazebo, build tools, and GUI setup.

## 14. Docker path mapping

The same mounted workspace appears at two paths:

```text
Host:      /home/alin/my_robotics_ws
Container: /home/ws
```

`/tmp/pid_benchmarks` is temporary container storage and may disappear if the
container is deleted.

## 15. First-milestone outcome

The first implementation milestone established the following:

- Consistent robot and wheel parameters.
- Deterministic controller initialization.
- Simulation-time PID calculations.
- Safe handling of missing, malformed, or stale data.
- Integral anti-windup and derivative startup protection.
- Safe stopping on completion or estimator failure.
- Clear single-owner TF architecture.
- YAML-based tuning without C++ recompilation.
- Configurable reference and output files.
- MATLAB-ready experiment logs.
- Repeatable headless benchmark execution.
- Tested baseline, fast, and robust profiles.
- Explanatory comments throughout source and configuration files.
- A workspace that builds and passes its automated checks.

The main conceptual limitation is that the current PID is a lookahead waypoint
follower. Cross-track error is an evaluation signal but not a direct controller
input. This should be stated precisely and considered when designing a fair
PID/LQR/MPC comparison.
