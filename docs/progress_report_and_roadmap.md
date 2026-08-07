# Project Progress Report and Roadmap

Last updated: 2026-08-07

## 1. Executive summary

Most of the simulation infrastructure is complete, and the project has a
functional first PID baseline.

Current status after the scope and architecture review:

```text
Simulation and ROS architecture:       functional
Robot model and EKF integration:       functional
Original lookahead PID:                retained as a historical implementation
PID experiment logging:                functional
PID configuration profiles:            created
Automated benchmark runner:            created
Five reference paths:                  created
Cascaded path-error PID:                implemented and selected as thesis PID
Ground-truth evaluation:                implemented and isolated from feedback
MATLAB evaluation scripts:             not implemented
System identification:                 removed; unnecessary for current plant
LQR:                                   not started
MPC:                                   not started
Formal statistical experiments:        not started
```

The project is no longer at the "make the robot move" stage. It is
transitioning from infrastructure development into formal controller design
and experimental methodology.

The current Docker environment is:

```text
Name:   recursing_bhaskara
ID:     52b800f8190c0424e2d17490e266089c45485daad3055e81b06beb2d149c27dc
Status: started only while building or running Gazebo experiments
```

The workspace test result at the time of this report was:

```text
13 CTest targets
0 errors
0 failures
15 cppcheck source checks skipped by the ROS wrapper because of tool version
```

## 2. Current interpretation of the thesis

Based on the preliminary thesis description, the most appropriate scope is:

> Comparison of PID, LQR, and MPC for geometric path following of a
> differential-drive robot under a prescribed reference speed, common actuator
> limits, and controlled disturbances.

This scope is definitively geometric path tracking, not time-indexed trajectory
tracking. The thesis will state that choice explicitly.

The current path files contain:

```text
x,y
```

They do not contain:

```text
time,x,y,heading,v,w
```

They therefore define geometric paths rather than complete time-parameterized
trajectories.

The recommended common controlled quantities are:

```text
cross-track error
heading error
velocity error
```

The robot does not have to be at a specific point at an exact timestamp.
Completion time remains a measured performance result.

This interpretation is consistent with the preliminary thesis description
because it emphasizes:

```text
cross-track error
control effort
robustness
predefined paths
kinematic and dynamic constraints
```

## 3. Current software architecture

The present closed loop is:

```text
Track CSV
    |
    v
Current PID controller
    |
    | /cmd_vel
    v
Gazebo differential-drive plugin
    |
    +---- /odom
    |
    +---- /demo/imu
             |
             v
             EKF
             |
             | /odometry/filtered
             v
       PID controller

Gazebo exact body state
    |
    | /ground_truth/odom
    v
trajectory evaluator (measurement only)
```

Additional components are:

```text
robot_state_publisher:
publishes robot body, wheel, and sensor transforms

RViz:
visualization only

Gazebo:
plant physics, sensor simulation, and wheel actuation

EKF:
state estimation

benchmark script:
starts trials, stops trials, saves CSVs, and calculates metrics
```

## 4. Work completed so far

### 4.1 Project and Docker access

Access was established to the ROS workspace and Docker environment.

The project is mounted approximately as:

```text
Host:
  /home/alin/my_robotics_ws

Container:
  /home/ws
```

The development container contains ROS 2 Humble, Gazebo Classic, robot
localization, the C++ compiler, colcon, and the required ROS build and testing
dependencies.

### 4.2 PID controller rewrite and stabilization

The main controller files are:

```text
src/my_robot_controller/include/my_robot_controller/pid_node.hpp
src/my_robot_controller/src/pid_node.cpp
```

The controller was substantially rewritten to remove unsafe behavior and make
experiments reproducible.

#### Deterministic initialization

All C++ state variables are explicitly initialized, including:

- Robot position and heading.
- Previous timestamps.
- Previous PID errors.
- Integral memory.
- Waypoint progress.
- Completion state.
- Watchdog state.

This removes unpredictable behavior caused by uninitialized memory.

#### Safe waypoint loading

The controller checks that:

- The input path exists.
- The path is not empty.
- At least two valid points exist.
- Every row has exactly two columns.
- Every coordinate is finite.
- Input and output files are different.

Invalid input produces a clear startup failure rather than undefined robot
motion.

#### Simulation-time control

PID timing uses timestamps from filtered odometry. Integral and derivative
calculations therefore follow Gazebo simulation time rather than computer
wall-clock performance.

#### Odometry-driven updates

Each new `/odometry/filtered` message triggers one control update. The
controller does not repeatedly calculate commands from an unchanged state.

#### PID safety behavior

The PID implementation includes:

- Integral limiting.
- First-sample derivative suppression.
- PID-memory reset after large timing gaps.
- Rejection of invalid state estimates.
- Command saturation.
- Explicit zero command at completion.
- Explicit zero command during shutdown.

#### Odometry watchdog

If filtered odometry disappears for more than two real seconds, the controller
sends zero velocity. The watchdog continues to work if Gazebo simulation time
pauses.

### 4.3 Current path-following algorithm

The current PID is still the lookahead implementation.

It performs:

1. Forward-only closest-waypoint search.
2. Projection onto the current path segment.
3. Cross-track error calculation.
4. Selection of a waypoint five samples ahead.
5. Distance calculation to that lookahead waypoint.
6. Bearing calculation toward the lookahead waypoint.
7. Linear PID on lookahead distance.
8. Angular PID on lookahead bearing error.
9. Forward-speed reduction using heading error.
10. Command saturation and logging.

The current control law is conceptually:

```text
distance to lookahead point -> linear PID -> v_cmd

bearing to lookahead point
minus actual heading        -> angular PID -> w_cmd
```

Cross-track error is calculated and logged, but it is not directly used as a
PID input.

The current controller should be described as:

> A waypoint-based PID path follower using lookahead distance and heading
> error.

It should not yet be described as the proposed cascaded cross-track PID.

### 4.4 Path projection and progress logic

The controller does not compare the robot to a CSV row with the same sample
number. It projects the robot onto the local path segment.

This provides:

- Geometric reference position.
- Signed cross-track error.
- Path tangent heading.
- Meaningful MATLAB comparison data.

The waypoint search only moves forward through a limited window. This prevents:

- Jumping backward to passed points.
- Selecting the wrong figure-eight branch.
- Immediate completion on a closed path.

### 4.5 Completion logic

A track is complete only when:

- The controller has reached the final path region.
- The robot is within 8 cm of the endpoint.

This handles closed tracks whose final point equals their starting point.

The controller records a final CSV row with:

```text
v_cmd = 0
w_cmd = 0
```

and flushes the file.

### 4.6 Experiment logging

The controller records:

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

This supports MATLAB plots of:

- Reference path versus actual path.
- Cross-track error.
- Heading error.
- Linear command.
- Angular command.
- Completion time.
- Final error.
- Control-command effort.

### 4.7 Robot model corrections

The main robot model is:

```text
src/my_robot_description/urdf/my_robot.urdf
```

The differential-drive plugin was corrected to match the physical URDF
geometry:

```text
wheel separation = 0.35 m
wheel diameter   = 0.20 m
```

This prevents inconsistencies between visual robot geometry, wheel kinematics,
simulated turning, and generated odometry.

### 4.8 Transform architecture

The ROS transform ownership was corrected.

Current ownership is:

```text
Gazebo:
publishes encoder-integrated odometry data

EKF:
publishes odom -> base_footprint

robot_state_publisher:
publishes base and wheel transforms
```

Gazebo no longer competes with the EKF for the same odometry transform.

### 4.9 Joint-state publishing

Gazebo publishes simulated wheel joint angles on `/joint_states`.
`robot_state_publisher` uses them to publish wheel transforms.

The GUI-oriented joint-state publisher was removed because Gazebo should own
simulated joint positions.

### 4.10 EKF configuration

The EKF configuration is:

```text
src/my_robot_description/config/ekf.yaml
```

It combines:

```text
/odom
/demo/imu
```

and produces:

```text
/odometry/filtered
```

It operates in planar 2D mode and resets correctly if Gazebo time jumps
backward. The EKF is the sole owner of the filtered odometry transform.

The EKF fuses encoder velocities and the nonholonomic lateral-velocity
constraint with IMU yaw rate and longitudinal acceleration. Absolute IMU
orientation is excluded because the simulated plugin obtains it from exact
Gazebo attitude. A separate `/ground_truth/odom` stream is reserved for scoring
only; it is not feedback.

### 4.11 Launch architecture

The simulation launch file is:

```text
src/my_robot_description/launch/display.launch.py
```

It starts:

- Gazebo.
- The selected world.
- The robot.
- The EKF.
- Robot state publishing.
- RViz when requested.

It supports:

```text
gui:=true
gui:=false
```

Headless mode runs Gazebo physics without rendering and is used for automated
benchmarks.

The PID launch file is:

```text
src/my_robot_controller/launch/pid.launch.py
```

It supports configurable input track, output CSV, PID profile, and simulation
time.

### 4.12 PID profiles

The following profiles exist:

```text
pid_baseline.yaml
pid_fast.yaml
pid_robust.yaml
```

Their purpose was exploratory:

```text
baseline:
reference behavior

fast:
lower completion time, potentially more error and effort

robust:
lower path error and stronger damping, potentially slower
```

They all use the same velocity limits, goal tolerance, lookahead count, search
window, timing limits, and watchdog timeout. Preliminary differences therefore
came primarily from PID tuning.

For formal thesis experiments, a different PID profile should not be selected
for every evaluation track. One final controller should be tuned on a predefined
tuning set, frozen, and evaluated everywhere.

### 4.13 Reference tracks

The project contains five formal evaluation tracks and two isolated capture
tests for initial lateral and heading error.

#### Straight

```text
File:   track_1_straight.csv
Points: 401
Length: 20 m
```

The benchmark script uses the first 5 m for scale consistency.

#### Sinusoidal curve

```text
File:   track_2_curve.csv
Points: 101
```

This tests smoothly changing steering direction.

#### Sharp 90-degree corner

```text
File:   track_3_corner.csv
Points: 121
Length: 6 m
```

This is deliberately infeasible at nonzero speed at the exact corner. It is a
stress test rather than a normal smooth path.

#### Circle

```text
File:   track_4_circle.csv
Points: 127
Radius: 1.0 m
Length: approximately 6.283 m
```

It tests sustained curvature, closed-path progress, steady curve error, and
endpoint handling when start equals finish.

#### Figure-eight

```text
File:        track_5_figure_eight.csv
Points:      189
Lobe radius: 0.75 m
Length:      approximately 9.423 m
```

It tests path intersections, correct branch selection, steering reversal, and
closed-path completion.

The track generator is:

```text
generate_tracks.py
```

### 4.14 Benchmark runner

The benchmark script is:

```text
scripts/run_pid_benchmarks.sh
```

The default matrix is:

```text
1 cascaded PID x 5 tracks x 3 reproducible seeds = 15 trials
```

For every trial, it:

1. Starts a fresh headless simulation.
2. Waits for the robot to spawn.
3. Allows the EKF to initialize.
4. Starts the selected PID only after the downstream command subscriber exists.
5. Waits for completion or timeout.
6. Holds zero command for a fixed simulation-time settling interval.
7. Checks Gazebo-truth linear and angular velocity before terminating.
8. Stops the controller and Gazebo.
9. Calculates estimator-diagnostic and ground-truth summary metrics separately.
10. Starts a clean trial.

It calculates:

```text
success
duration
final error
cross-track RMSE
maximum cross-track error
heading RMSE
maximum heading error
normalized command activity
maximum v_cmd
maximum w_cmd
final position
sample count
```

It also supports focused runs, for example:

```text
PID_BENCHMARK_TRACKS="circle figure_eight" \
PID_BENCHMARK_REPETITIONS=1 \
bash scripts/run_pid_benchmarks.sh /tmp/pid_closed_paths
```

Cleanup handling was improved so interruption does not intentionally leave
active simulation groups behind.

### 4.15 Documentation

The main explanatory documents are:

```text
docs/project_architecture_explained.md
docs/pid_controller_recommendation.md
```

The source, launch, YAML, URDF, world, build, and container files also contain
explanatory comments.

## 5. Verification completed

At the time this report was prepared, the current container reported:

```text
22 tests
0 errors
0 failures
2 skipped
```

Previous verification included:

- Successful ROS builds.
- C++ style validation.
- Python style validation.
- CMake validation.
- XML validation.
- URDF validation.
- YAML validation.
- Launch-file validation.
- Shell syntax validation.
- CSV geometry checks.
- Full Gazebo path-following trials.

## 6. Preliminary PID results

An earlier 3-profile by 3-track matrix completed successfully on:

```text
straight
sinusoidal curve
sharp corner
```

The preliminary tendencies were:

```text
Fast profile:
about 26 percent faster than baseline
higher curve and corner error
higher control effort

Robust profile:
lower curve and corner error
slightly lower control effort
considerably slower
```

Baseline-only functional tests also completed on the circle and figure-eight.

Observed focused results were approximately:

```text
Track         Duration   Final error   CTE RMSE
circle        18.400 s   0.0716 m      0.0175 m
figure-eight  27.696 s   0.0735 m      0.0233 m
```

These results are preliminary only.

The raw result CSVs were stored under `/tmp` in an earlier container and did not
survive the container replacement. The current container did not contain the
old benchmark summaries when this report was prepared.

Therefore:

```text
The behavior was observed and discussed,
but the data is not yet a formal thesis artifact.
```

Future results should be written under the mounted workspace, for example:

```text
/home/ws/results/
```

which corresponds to:

```text
/home/alin/my_robotics_ws/results/
```

## 7. Important unfinished issues

### 7.1 No Git checkpoint

The workspace contains a large set of modified and new files, but no checkpoint
commit has been made.

The worktree includes:

- Controller rewrite.
- Robot model corrections.
- Launch changes.
- EKF changes.
- New tuning profiles.
- New tracks.
- Benchmark automation.
- Documentation.

Before another major controller rewrite, this state should be reviewed and
saved in Git.

### 7.2 Generated trajectory file

The workspace-level `robot_actual_trajectory.csv` contains generated experiment
data. It should not be treated as a stable source file.

A decision is needed to:

- Remove it from version control.
- Keep one deliberately named example dataset.
- Move generated runs under `results/`.

Arbitrary buffered experiment output should not accidentally enter the
implementation checkpoint.

### 7.3 The current PID is not the recommended final PID

The cascaded cross-track and heading PID has been designed conceptually but has
not been implemented. The current controller remains a lookahead waypoint
follower.

### 7.4 Common reference speed is not formalized

The current linear PID produces speed from distance to the lookahead waypoint.
Speed therefore depends partly on:

- Lookahead count.
- Waypoint spacing.
- Linear PID gains.

For a fair PID/LQR/MPC comparison, a common reference-speed definition is still
needed.

### 7.5 Curvature calculation is missing

The current reference manager calculates path tangent but not path curvature.

Curvature is needed for:

```text
w_feedforward = v_ref * curvature_ref
```

and will also be useful for the LQR and MPC reference models.

### 7.6 Sharp-corner classification

The 90-degree corner has discontinuous direction and theoretically unbounded
curvature at the corner.

It should remain a stress test, but a rounded-corner track should also be
generated for physically feasible high-curvature evaluation.

### 7.7 Rough world is not a fair disturbance test

The current raised boxes behave as collision obstacles or steps. In an earlier
run, they threw the robot far away from the path.

A better disturbance suite should use controlled and repeatable disturbances:

- Heading impulses.
- Lateral displacement.
- Wheel-radius mismatch.
- Wheel-separation mismatch.
- Mass or inertia mismatch.
- Friction changes.
- Mild slope.
- Sensor noise.
- Command delay.

### 7.8 No formal repeated-trial campaign

Only preliminary single trials have been run.

Thesis-quality conclusions still require:

- Frozen controller parameters.
- Separate tuning and evaluation tracks.
- Repeated trials.
- Persistent raw results.
- Statistical summaries.
- Consistent initial conditions.
- Controlled disturbance magnitudes.

### 7.9 No MATLAB analysis pipeline

The C++ logger produces suitable data, but MATLAB scripts for import, plotting,
metric verification, and statistical comparison have not yet been written.

## 8. Recommended following phases

### Phase 0: Secure the current milestone

Goal:

```text
Preserve the working baseline before further redesign.
```

Tasks:

1. Review the current Git diff.
2. Separate generated data from source.
3. Decide what to do with `robot_actual_trajectory.csv`.
4. Rebuild both ROS packages in the current container.
5. Run the full test suite.
6. Run one smoke-test simulation.
7. Create a Git checkpoint commit or tag.
8. Create a persistent results directory.

Suggested result layout:

```text
results/
  pid_lookahead/
    preliminary/
    formal/
```

Completion condition:

```text
The existing lookahead PID can be reproduced from a clean checkout.
```

### Phase 1: Freeze the research problem and fairness rules

Goal:

```text
Define exactly what PID, LQR, and MPC will be asked to do.
```

Recommended decision:

```text
Geometric path following with prescribed reference speed.
```

Define:

- Geometric path-tracking terminology and its distinction from timed tracking.
- Controller inputs and outputs.
- Common state estimates.
- Common reference manager.
- Common velocity and angular-velocity limits.
- Common acceleration limits, if used.
- Common completion conditions.
- Common disturbances.
- Common metrics.
- Tuning tracks versus evaluation tracks.

Suggested controlled variables:

```text
cross-track error
heading error
linear velocity error
angular velocity error
```

Completion condition:

```text
A short methodology specification exists before new controller code is written.
```

### Phase 2: Build the common reference manager

Goal:

```text
Give PID, LQR, and MPC identical path information.
```

Implement:

1. Arc-length progress along the path.
2. Robust projection onto path segments.
3. Reference heading.
4. Reference curvature.
5. Signed cross-track error.
6. Heading error.
7. Reference speed as a function of path progress.
8. Endpoint slowdown.
9. Monotonic branch handling.
10. Closed-path completion.

The path data can remain waypoint-based, but waypoints should describe a
continuous path instead of acting as individual targets.

Add tests for:

- Straight path.
- Circle curvature.
- Figure-eight crossing.
- Closed-path completion.
- Duplicate or zero-length segments.
- Sharp-corner behavior.

Completion condition:

```text
All controllers can consume the same reference-state structure.
```

### Phase 3: Implement the cascaded PID

Goal:

```text
Make cross-track and heading error explicit feedback variables.
```

Recommended first version:

```text
Outer loop:
cross-track P or PD
output = desired heading correction

Inner loop:
heading PD
output = angular velocity correction

Feedforward:
w_feedforward = v_ref * curvature_ref

Linear command:
common v_ref with heading, curvature, and endpoint slowdown
```

Retain:

- Integral limiting.
- Timing protection.
- Watchdog.
- Safe stopping.
- Command saturation.
- CSV logging.

Add logging for:

```text
reference speed
reference curvature
desired heading
heading correction
w_feedforward
w_feedback
saturation state
```

Preserve the original lookahead controller as a separate baseline rather than
overwriting its experimental identity.

Completion condition:

```text
The cascaded PID completes all nominal tracks safely.
```

### Phase 4: Tune and freeze the final PID

Goal:

```text
Produce one defensible PID configuration for formal comparison.
```

Recommended tuning sequence:

1. Tune the inner heading loop first.
2. Freeze the inner loop.
3. Tune outer proportional cross-track action.
4. Add derivative damping if path crossings oscillate.
5. Add integral only if persistent disturbance bias requires it.
6. Tune curvature feedforward.
7. Tune the common speed policy.
8. Validate on tracks not used for initial tuning.
9. Freeze the final gain set.

Recommended development tests:

```text
heading step
lateral-offset recovery
straight path
circle
sinusoidal curve
figure-eight
rounded corner
sharp-corner stress test
```

Completion condition:

```text
One final PID profile is frozen before LQR development.
```

### Phase 5: Finalize benchmark and MATLAB infrastructure

Goal:

```text
Create a repeatable measurement pipeline before adding LQR.
```

Add:

- Persistent result directories.
- Run metadata.
- Controller name and parameter snapshot.
- Track name.
- World and disturbance definition.
- Trial number.
- Random seed, if applicable.
- Git revision.
- Simulation configuration.

MATLAB should produce:

- Desired versus actual XY path.
- Cross-track error versus time.
- Heading error versus time.
- Velocity tracking error.
- Linear and angular commands.
- RMSE tables.
- Maximum-error tables.
- Completion time.
- Control-command effort.
- Saturation duration.
- Disturbance recovery time.
- Mean and standard deviation across trials.

Completion condition:

```text
A PID experiment can be launched once and converted into thesis-ready plots and
tables.
```

### Phase 6: Derive and validate the analytical control model

Goal:

```text
Derive the path-error model used by LQR and MPC from the known differential-
drive kinematics and the explicitly configured Gazebo actuator constraints.
```

The simulated plant accepts body-velocity references through a Gazebo joint
velocity servo. Its wheel acceleration bound is already specified in the URDF,
so black-box identification would mostly rediscover a known rate limiter and is
outside the final thesis scope.

Use the known kinematics:

```text
x_dot       = v * cos(heading)
y_dot       = v * sin(heading)
heading_dot = w
```

Tasks:

1. Express the model in cross-track and heading-error coordinates.
2. Linearize around the common nonzero path-reference speed.
3. Discretize at the measured controller sample period.
4. Verify signs and one-step predictions against Gazebo data.
5. Apply the known velocity, yaw-rate, and wheel-acceleration limits.

Completion condition:

```text
The analytical discrete model reproduces local path-error evolution with the
accuracy required for LQR and MPC design.
```

### Phase 7: Design the LQR

Goal:

```text
Build the second formal controller using the analytical path-error model.
```

Use path-relative error states such as:

```text
cross-track error
heading error
linear velocity error
angular velocity error
```

Tasks:

1. Use the linearized analytical path-error dynamics.
2. Linearize around a nonzero reference speed.
3. Check controllability.
4. Select state and input scaling.
5. Choose Q and R weights.
6. Compute the LQR gain.
7. Add reference feedforward.
8. Apply the same actuator limits.
9. Decide whether integral augmentation is required.
10. Validate on the same nominal and disturbance tracks.

Completion condition:

```text
LQR completes the same benchmark suite with frozen weights.
```

### Phase 8: Implement MPC

Goal:

```text
Use the same model and errors in a constrained predictive controller.
```

Define:

- Prediction horizon.
- Control horizon.
- Sample time.
- State-error weights.
- Command-effort weights.
- Command-rate weights.
- Velocity constraints.
- Angular-velocity constraints.
- Acceleration constraints.
- Solver failure behavior.

Log:

```text
solver time
solver status
constraint activity
predicted cost
```

Completion condition:

```text
MPC runs in real time at the selected control frequency and completes the
benchmark suite.
```

### Phase 9: Formal comparative experiments

Goal:

```text
Collect final thesis evidence.
```

All three controllers must use:

```text
same robot
same estimator
same paths
same initial conditions
same reference speed
same limits
same disturbances
same completion rule
same metrics
```

Suggested experiment groups follow.

#### Nominal geometry

```text
straight
sinusoid
rounded corner
circle
figure-eight
```

#### Stress reference

```text
sharp 90-degree corner
higher reference speeds
```

#### Controlled disturbances

```text
heading impulse
lateral displacement
parameter mismatch
friction change
mild slope
sensor noise increase
command delay
```

#### Repeated trials

Where randomness is present, report:

```text
mean
standard deviation
minimum
maximum
confidence interval, if justified
```

Completion condition:

```text
A frozen dataset supports the final comparative conclusions.
```

### Phase 10: Thesis writing and reproducibility

Goal:

```text
Turn the implementation into a defensible engineering study.
```

Recommended chapter structure:

1. Problem definition.
2. Differential-drive model.
3. Simulation and estimation architecture.
4. Common path-reference formulation.
5. PID design.
6. Analytical model and LQR design.
7. MPC design.
8. Experimental methodology.
9. Results.
10. Discussion and limitations.
11. Conclusions and future work.

Include:

- Block diagrams.
- Signal and topic diagrams.
- Controller equations in the thesis document.
- Parameter tables.
- Track definitions.
- Disturbance definitions.
- Tuning procedure.
- Model validation plots.
- Statistical result tables.
- Reproduction instructions.
- Git revision used for formal results.

## 9. Recommended immediate next milestone

System identification has been removed from the project scope because the
Gazebo plant uses known kinematics and an explicitly configured wheel-speed
rate limiter. The next model-based milestone is therefore analytical LQR.

The recommended order is:

```text
1. Secure and commit the current working baseline.
2. Freeze the path-following problem definition.
3. Implement the common reference manager.
4. Implement the cascaded PID.
5. Tune and validate the final PID.
6. Build the MATLAB reporting pipeline.
7. Derive and validate the analytical path-error model.
8. Implement LQR.
9. Implement MPC.
10. Run formal comparisons.
```

## 10. Overall assessment

```text
Simulation infrastructure:       advanced
Current PID baseline:            advanced
Final PID methodology:           designed but not implemented
Benchmark infrastructure:        functional but not formalized
Formal results pipeline:         early
Analytical LQR model:            not started
LQR:                             not started
MPC:                             not started
```

The foundational work is substantial, but most of the thesis's comparative
control contribution still lies ahead. The next major step is converting the
current working lookahead baseline into a shared path-error architecture and
then implementing the final PID on top of it.
