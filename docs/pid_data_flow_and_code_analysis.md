# PID data flow and code analysis

## Purpose and scope

This document explains, in deliberately detailed and beginner-friendly terms, how data moves through the current mobile-robot project and how the PID controller is constructed. It is intended to answer questions such as:

- Where does the controller's position information come from?
- What exactly enters and leaves each PID loop?
- Why is a path projection required?
- What is stored from one update to the next?
- Which code performs control mathematics, and which code is ROS/Gazebo plumbing?
- What command does the controller request, and what command reaches Gazebo during a disturbance test?
- How do the source files, configuration files, launch files, tests, and logs fit together?

The main subject is the cascaded PID mode selected by `pid_cascade.yaml`. The older lookahead controller is also described where it shares code or helps explain the architecture. LQR and MPC are not implemented yet, but several current components were intentionally designed so those controllers can reuse exactly the same path interpretation and benchmark infrastructure.

The controller is intentionally a **spatial path-tracking controller**, not a strict time-indexed trajectory tracker. It keeps the robot on a geometric curve and supplies a reference speed based on curvature and distance from the endpoint. The robot is not required to occupy a particular path coordinate at a prescribed absolute time; completion time is instead an evaluated performance result.

---

## 1. The complete closed loop in one picture

For an ordinary run, the main feedback loop is:

```text
path CSV
   |
   v
PathReferenceManager <------------------------------+
   |                                                |
   | local path position, tangent, curvature,       |
   | cross-track error, heading error, v_ref, w_ref |
   v                                                |
CascadedPidController                               |
   |                                                |
   | requested body velocity: v_cmd and w_cmd       |
   v                                                |
/cmd_vel                                            |
   |                                                |
   v                                                |
Gazebo differential-drive plugin                    |
   |                                                |
   | wheel motion changes simulated robot pose      |
   v                                                |
Gazebo wheel odometry /odom + IMU /demo/imu         |
   |                                                |
   v                                                |
robot_localization EKF                              |
   |                                                |
   | filtered pose /odometry/filtered               |
   +------------------------------------------------+
```

This is a closed loop because the command changes the simulated robot, the simulated sensors observe the changed robot, and the next command is calculated from the new observation.

During the command-disturbance experiment, one extra block is inserted after the controller:

```text
PID node
   |
   | /cmd_vel_nominal: what PID requested
   v
CommandDisturbanceInjectorNode
   |
   | /cmd_vel: nominal command plus scheduled bias, with safety limits
   v
Gazebo differential-drive plugin
```

This placement matters. The PID log records what the controller believed it was commanding. The injector log records both that nominal command and the command actually delivered to Gazebo. The PID does not receive a special flag saying that a fault is active. It only sees the resulting pose error later through normal sensor feedback, so the recovery is genuine closed-loop disturbance rejection.

---

## 2. What “data” means in this project

There are four different categories of data. Separating them makes the code much easier to understand.

### 2.1 Configuration data

Configuration is chosen before an experiment and normally remains constant throughout the run. Examples are:

- PID gains;
- speed and yaw-rate limits;
- nominal reference speed;
- the path CSV filename;
- output CSV filename;
- disturbance start time, duration, and magnitude;
- whether ROS should use Gazebo simulation time.

Most controller configuration comes from a YAML file, and experiment-specific filenames or fault values come from launch arguments. These values are read once in a node constructor.

### 2.2 Current measured data

This is the latest estimate of what the robot is doing:

- `current_x_` in metres;
- `current_y_` in metres;
- `current_theta_` in radians;
- the odometry timestamp in seconds.

The PID node receives these values through `/odometry/filtered`.

### 2.3 Derived reference and error data

The path manager compares the current robot pose with the path and calculates:

- closest continuous point on the relevant path segment;
- path progress in metres of arc length;
- remaining path length;
- path tangent heading;
- signed cross-track error;
- path-heading error;
- reference curvature;
- reference linear velocity `v_ref`;
- reference angular velocity `w_ref`.

These are not raw sensor measurements. They are computed from the measured pose and the stored path.

### 2.4 Controller memory

A PID is not purely a function of the current error. Its integral and derivative terms require information retained between updates. Each `PIDController` object stores:

- accumulated integral error;
- previous error;
- a Boolean saying whether a previous error exists.

The cascaded architecture owns two separate `PIDController` objects, so the outer and inner loops have independent memory.

---

## 3. Coordinate frames, directions, and sign conventions

Sign mistakes are among the most common controller bugs. The present code uses a consistent planar ROS convention:

- `x` points forward along the robot body;
- `y` points to the robot's left;
- `z` points upward;
- positive yaw is counter-clockwise when viewed from above;
- positive `linear.x` means forward motion;
- positive `angular.z` means a left turn;
- negative `angular.z` means a right turn.

The EKF publishes the robot pose in the `odom` frame and identifies the robot body frame as `base_footprint`. The PID only needs planar `x`, `y`, and yaw.

The path has an orientation because its CSV points are ordered. A segment from point A to point B has a tangent vector:

```text
delta = B - A = (delta_x, delta_y)
```

and segment heading:

```text
path_heading = atan2(delta_y, delta_x)
```

The cross-track sign is defined relative to that oriented tangent:

```text
cross_track_error = tangent_x * error_y - tangent_y * error_x
```

where `error = robot_position - projected_path_position` and `tangent` is a unit vector along the path.

Consequently:

- positive cross-track error means the robot is on the **left** side of the oriented path;
- negative cross-track error means it is on the **right** side;
- a robot left of the path needs a negative/right-turning heading correction;
- this is why the outer PID output is negated in `CascadedPidController::calculate()`.

Reversing the order of points in a CSV reverses the path orientation and therefore changes the meanings of left, right, heading, progress, and curvature. A track file is not just an unordered collection of coordinates.

---

## 4. Startup: how an experiment is assembled

Two ROS launch graphs are normally started.

### 4.1 Simulation and state estimation

`src/my_robot_description/launch/display.launch.py` starts:

1. **Gazebo**, using `empty.world` by default.
2. **robot_state_publisher**, which publishes transforms derived from the URDF and joint states.
3. **spawn_entity.py**, which inserts the URDF robot into Gazebo.
4. **robot_localization's EKF**, which fuses wheel odometry and IMU information.
5. **RViz**, only when `gui:=true`; RViz is visualization and is not in the control loop.

All of these actions are launched concurrently. ROS topics and Gazebo services allow dependent components to wait until their inputs become available.

### 4.2 Controller without a disturbance

`src/my_robot_controller/launch/pid.launch.py` starts one `pid_node` process. It supplies:

- a path CSV;
- an output CSV;
- a YAML configuration;
- the `use_sim_time` setting.

With the cascade profile, the relevant invocation selects `pid_cascade.yaml`. The PID node publishes directly to `/cmd_vel`, which the Gazebo differential-drive plugin subscribes to.

### 4.3 Controller with a command disturbance

`src/my_robot_controller/launch/pid_command_disturbance.launch.py` starts:

1. `pid_node`, but remaps its `cmd_vel` output to `cmd_vel_nominal`;
2. `command_disturbance_injector`, which subscribes to `cmd_vel_nominal` and publishes the actual `cmd_vel`.

ROS remapping changes the connection without changing the PID source code. The same PID executable is therefore used in nominal and disturbed tests.

### 4.4 What the PID constructor does

`PidNode::PidNode()` performs all startup work before the ROS event loop begins:

1. Declares every permitted ROS parameter and its fallback default.
2. Reads and validates integer, timing, tolerance, and limit parameters.
3. Converts the text `controller_mode` into an internal enum.
4. Configures the legacy distance and angular PIDs.
5. Builds a `CascadedPidConfig` and configures both cascade loops.
6. Builds a `PathReferenceConfig` and configures the common path manager.
7. Checks that input and output filenames are sensible and different.
8. Loads and preprocesses the path CSV.
9. Opens a new experiment CSV and writes its 26-column header.
10. Creates the velocity publisher.
11. Creates the filtered-odometry subscriber.
12. Creates a 100 ms wall-clock safety watchdog.

If a required parameter or file is invalid, the constructor throws an exception. `main()` catches it, prints a fatal ROS message, and exits with an error instead of running a partially initialized controller.

---

## 5. Where the measured pose comes from

### 5.1 Gazebo's differential-drive plugin

The URDF contains `libgazebo_ros_diff_drive.so`. Its important settings are:

- update rate: 30 Hz;
- wheel separation: 0.35 m;
- wheel diameter: 0.2 m;
- maximum wheel torque: 20;
- maximum wheel acceleration: 1.0;
- input topic: `/cmd_vel`;
- output wheel odometry topic: `/odom`.

The plugin translates a body-level command `(v, w)` into differential wheel motion. Conceptually, for wheel radius `r` and track width `L`, the ideal wheel angular speeds are:

```text
right_wheel_speed = (v + w*L/2) / r
left_wheel_speed  = (v - w*L/2) / r
```

The exact motor/contact response is then handled by Gazebo, including the configured acceleration, torque, mass, inertia, friction, and physics time steps.

The plugin does **not** publish the `odom -> base_footprint` transform in this project. That avoids having two competing publishers because the EKF owns the filtered transform.

### 5.2 Simulated IMU

The Gazebo IMU plugin publishes `/demo/imu` at 100 Hz. Its measurements include small Gaussian noise. The faster IMU rate lets the 30 Hz EKF use a recent inertial measurement at each filter update.

### 5.3 EKF fusion

`src/my_robot_description/config/ekf.yaml` configures `robot_localization` in two-dimensional mode. It fuses selected values from:

- `/odom`: planar position, yaw, planar body velocity, and yaw rate;
- `/demo/imu`: yaw, yaw rate, and longitudinal acceleration.

The result is published as `/odometry/filtered`. This message contains:

- header timestamp;
- pose position;
- pose orientation as a quaternion;
- velocity and covariance information.

The current PID uses the filtered pose and timestamp. It does not presently use the velocity fields in the odometry message directly.

### 5.4 Quaternion-to-yaw conversion

ROS stores a 3D orientation as quaternion components `(qx, qy, qz, qw)`. The callback extracts planar yaw using:

```text
yaw = atan2(
    2 * (qw*qz + qx*qy),
    1 - 2 * (qy*qy + qz*qz))
```

This gives the robot heading in radians. Even though the EKF runs in 2D mode, the message format remains a general 3D quaternion, so this conversion is necessary.

The callback rejects non-finite `x`, `y`, or yaw values before they can propagate into geometry or velocity commands.

---

## 6. What causes one control update

The PID is **odometry-driven**, not timer-driven.

Every accepted `/odometry/filtered` message invokes:

```text
PidNode::odom_callback(message)
```

That callback computes the time step `dt` and then calls:

```text
control_loop(stamp_seconds, dt)
```

This has an important consequence: one new state estimate causes one new control calculation. The 100 ms wall timer in the class is only a watchdog; it does not calculate PID outputs.

### 6.1 Time source

When `use_sim_time` is true, ROS `now()` and sensor timestamps follow Gazebo's `/clock`. If simulation runs at half real-time speed, a simulated 0.033 s step is still treated as 0.033 s by the PID. Controller dynamics therefore follow simulation time rather than host-computer speed.

### 6.2 Time-step calculation

For the first odometry sample:

```text
dt = 1 / nominal_control_frequency
```

With the current 30 Hz parameter, the fallback is approximately 0.03333 s.

For later samples:

```text
dt = current_odometry_stamp - previous_odometry_stamp
```

Special cases are handled deliberately:

- if time repeats or moves backwards, the sample is ignored;
- if `dt` exceeds `max_control_dt` (currently 0.2 s), all PID memory is reset and one nominal interval is used;
- if odometry disappears for more than `odom_timeout` wall seconds (currently 2 s), the watchdog publishes zero velocity and resets PID memory.

These protections prevent division by zero, derivative spikes, or a large integral update after a broken data stream.

---

## 7. Converting a waypoint file into a continuous path

The CSV contains rows of the form:

```text
x,y
```

It has no header. `PathReferenceManager::load_csv()` requires exactly two finite numeric values per row and at least two points. Consecutive duplicate points are rejected because they would create a zero-length segment with no valid tangent.

### 7.1 Why the waypoints are not used only as isolated targets

If the controller only selected the nearest CSV row, its error would jump whenever the nearest row changed. Results would also depend strongly on waypoint spacing. Instead, adjacent points define straight segments, producing a piecewise-linear continuous path.

For each segment, preprocessing stores:

- start point;
- displacement vector `delta`;
- length;
- squared length;
- tangent heading;
- estimated curvature;
- cumulative arc length at the segment start.

This work is done once when the path loads, not on every control update.

### 7.2 Path progress

Let segment `i` begin at cumulative length `s_i`, have length `L_i`, and let `t` be a fraction from 0 to 1 along it. Continuous path progress is:

```text
s = s_i + t * L_i
```

This `s` is measured in metres along the path, not straight-line distance from the origin. It gives a common one-dimensional coordinate for straight lines, bends, circles, and figure-eight paths.

---

## 8. Projection: what `t` means, why it is needed, and how it is calculated

Projection answers this question:

> Which point on the nearby path best corresponds to the robot's current position?

Suppose a segment starts at `A`, ends at `B`, and the robot position is `R`:

```text
D = B - A
E = R - A
```

The unbounded projection fraction is:

```text
t_raw = dot(E, D) / dot(D, D)
```

In coordinates:

```text
t_raw = ((R_x-A_x)*D_x + (R_y-A_y)*D_y) / (D_x^2 + D_y^2)
```

The fraction is clamped to the finite segment:

```text
t = clamp(t_raw, 0, 1)
```

The projected point is then:

```text
P = A + t*D
```

Interpretation:

- `t = 0`: projection is at the segment start;
- `t = 0.5`: projection is halfway along the segment;
- `t = 1`: projection is at the segment end;
- `t_raw < 0`: the closest point on the infinite line lies before A, so the finite-segment answer is A;
- `t_raw > 1`: it lies after B, so the finite-segment answer is B.

The dot product is used because the closest point has a robot-to-path error perpendicular to the segment tangent. Algebraically, minimizing squared distance from `R` to `A + tD` and differentiating with respect to `t` produces the formula above.

### 8.1 Why projection is better than nearest waypoint

Consider a straight path with waypoints at `(0,0)` and `(1,0)` and a robot at `(0.37,0.20)`. The nearest continuous path point is `(0.37,0)`, not either endpoint. Projection provides:

- a meaningful cross-track error of `+0.20 m`;
- continuous progress of `0.37 m`;
- a stable tangent heading of `0 rad`.

Nearest-waypoint logic would report a diagonal error toward `(0,0)` or `(1,0)` and would abruptly switch between them.

### 8.2 Searching without jumping at intersections

The manager does not compare the robot against every segment forever. It searches from the remembered current segment through a configurable forward window, currently 20 segments. Among those candidates, it chooses the smallest squared projection distance.

The stored path progress is monotonic: it does not move backwards. This is crucial for the figure eight, where multiple path locations share or nearly share the same Cartesian coordinates. Without ordered local search and monotonic progress, the robot could jump from the first crossing to the final crossing and falsely declare completion.

The implementation only advances on a distance tie at an adjacent shared vertex. That avoids arbitrary jumps when two connected segments have the same closest endpoint.

This policy assumes forward path traversal. It is not designed for reversing along the path or teleporting far ahead by more than the search window.

---

## 9. Reference quantities produced after projection

`PathReferenceManager::update(robot_x, robot_y, robot_heading)` returns a `PathReference` containing path-only geometry and robot-relative errors.

### 9.1 Reference position

`reference.path.position` is the continuous projected point `P`, not necessarily one of the CSV waypoints.

### 9.2 Path heading

`reference.path.heading` is the tangent heading of the selected piecewise-linear segment.

### 9.3 Cross-track error

The robot-minus-projection vector is resolved with the tangent cross product. It is a signed perpendicular displacement in metres:

```text
e_y > 0: robot left of path
e_y < 0: robot right of path
```

### 9.4 Path-heading error

The common evaluation error is:

```text
path_heading_error = wrap(path_heading - robot_heading)
```

`wrap()` maps angular differences into `[-pi, pi]`. For example, a raw difference near `+2*pi` represents almost zero physical error and should not produce a full rotation command.

This path-heading error is intentionally distinct from the cascade's inner-loop error. The inner loop follows a corrected heading, not always the raw path tangent.

### 9.5 Curvature

For polygonal waypoints, curvature is estimated from the change in neighboring segment headings divided by distance between their segment centres. Heading differences are wrapped before division. Endpoint segments use a one-sided estimate; interior segments use neighboring information. Estimated magnitude is clamped by `maximum_reference_curvature`, currently `5.0 1/m`, to limit extreme values at sharp polygon corners.

The resulting curvature is an approximation to:

```text
kappa = d(path_heading) / ds
```

Positive curvature means a left-turning path and negative curvature means a right-turning path.

### 9.6 Common reference speed

The nominal speed is reduced for curvature:

```text
curvature_factor = 1 / (1 + curvature_speed_gain * abs(curvature))
```

Near the endpoint it is also reduced linearly:

```text
endpoint_factor = clamp(remaining_length / endpoint_slowdown_distance, 0, 1)
```

Therefore:

```text
v_ref = nominal_speed * curvature_factor * endpoint_factor
w_ref = v_ref * curvature
```

`w_ref` is the ideal kinematic yaw rate for following the local curvature at `v_ref`. It is feedforward: it asks for the turn that the known path geometry predicts will be necessary, even when tracking errors are zero.

These reference definitions live outside the PID so future LQR and MPC implementations can be compared using the same path, progress, curvature, and speed policy.

---

## 10. The generic discrete PID implementation

The reusable PID mathematics is in:

- `include/my_robot_controller/pid_controller.hpp`;
- `src/pid_controller.cpp`.

The header declares what the class owns and what functions callers can use. The `.cpp` file defines how those functions behave.

For each accepted update `k`, the inputs are:

```text
e[k]  = current scalar error
dt[k] = elapsed time since the previous accepted update
```

### 10.1 Proportional term

```text
P[k] = Kp * e[k]
```

The proportional term reacts to the present error. A larger error immediately produces a larger contribution.

### 10.2 Integral term

The code uses rectangular numerical integration:

```text
I_state[k] = clamp(
    I_state[k-1] + e[k]*dt[k],
    -integral_limit,
    +integral_limit)

I[k] = Ki * I_state[k]
```

The stored state represents accumulated error over time. It can remove a persistent steady-state offset that proportional feedback alone cannot overcome.

The clamp is a simple anti-windup measure. It limits the **stored integral state**, so it cannot grow without bound during a long error.

This is not full actuator-aware back-calculation anti-windup. The PID class does not know whether a later outer correction or final velocity command saturates. It only clamps its own integral state. That is adequate as a simple baseline, especially because both present cascade integral gains are zero, but it is an important technical limitation to state accurately.

### 10.3 Derivative term

After at least one previous sample:

```text
D_state[k] = (e[k] - e[k-1]) / dt[k]
D[k] = Kd * D_state[k]
```

The derivative term reacts to how quickly the error changes. It often adds damping, but it is also sensitive to noise and irregular timing.

On the first update after construction or reset, no valid previous error exists, so the code deliberately uses:

```text
D_state = 0
```

This prevents a derivative kick caused only by initialization.

### 10.4 Final generic PID output

```text
u[k] = Kp*e[k] + Ki*I_state[k] + Kd*D_state[k]
```

The generic class does not know whether `u` represents metres per second, radians, or radians per second. Meaning and units come from how the containing controller uses it.

### 10.5 Invalid inputs

If error or `dt` is non-finite, or if `dt <= 0`, the generic `calculate()` returns zero. The cascade additionally validates all of its inputs and throws an exception for invalid values. At the ROS boundary, odometry validity and timestamp ordering are checked before normal cascade calculation.

### 10.6 Reset behavior

`reset()` sets:

```text
integral = 0
previous_error = 0
has_previous_error = false
```

The next valid update then has a zero derivative term. Reset occurs on configuration, excessive odometry gaps, or watchdog timeouts.

---

## 11. How the cascaded PID is built

The cascade is implemented by `CascadedPidController`, which owns:

```text
PIDController cross_track_pid_;  // outer loop
PIDController heading_pid_;      // inner loop
```

They use identical generic PID code but have different errors, gains, output meanings, and independent memory.

The word “cascade” means that the output of the outer loop helps form the reference for the inner loop:

```text
cross-track error
      |
      v
outer PID
      |
      v
heading correction ---> desired heading
                              |
measured robot heading -------+--> heading error
                                      |
                                      v
                                  inner PID
                                      |
curvature feedforward ---------------+--> angular command
```

It does **not** mean that the robot waits for one loop to finish before running the next. Both loops are evaluated during the same odometry callback.

---

## 12. Exact inputs and outputs of the cascade

`CascadedPidController::calculate()` receives six quantities:

| Input | Meaning | Unit | Source |
|---|---|---:|---|
| `cross_track_error` | Signed perpendicular position error | m | Path manager |
| `path_heading` | Local path tangent angle | rad | Path manager |
| `robot_heading` | Estimated robot yaw | rad | EKF odometry |
| `reference_linear_velocity` | Geometry-based forward-speed reference | m/s | Path manager |
| `reference_angular_velocity` | Curvature feedforward yaw rate | rad/s | Path manager |
| `dt` | Time since previous accepted odometry | s | PID node |

It returns a `CascadedPidOutput` containing:

| Output | Meaning | Unit |
|---|---|---:|
| `linear_command` | Requested forward velocity | m/s |
| `angular_command` | Requested yaw rate | rad/s |
| `desired_heading` | Path heading after lateral correction | rad |
| `heading_correction` | Bounded outer-loop correction | rad |
| `cross_track_pid_output` | Raw outer PID output before sign and clamp | rad by intended tuning |
| `heading_error` | Desired heading minus robot heading, wrapped | rad |
| `heading_pid_output` | Inner feedback yaw rate | rad/s |
| `heading_speed_factor` | Slowdown due to misalignment | dimensionless |
| `cross_track_speed_factor` | Slowdown due to lateral displacement | dimensionless |

Only `linear_command` and `angular_command` are sent to the robot. The other outputs are diagnostics logged for explanation, tuning, and thesis plots.

---

## 13. Outer loop: cross-track error to heading correction

The outer loop executes:

```text
u_cte = PID_cte(e_y, dt)
heading_correction = clamp(-u_cte, -0.7, +0.7)
desired_heading = wrap(path_heading + heading_correction)
```

With the current cascade YAML:

```text
Kp_cte = 1.5
Ki_cte = 0.0
Kd_cte = 0.20
integral_state_limit = 0.5
maximum_heading_correction = 0.7 rad
```

The intended engineering dimensions are:

- `Kp_cte`: rad/m;
- `Ki_cte`: rad/(m*s);
- `Kd_cte`: rad*s/m;
- outer integral state: m*s;
- outer PID output: rad.

Radians are mathematically dimensionless, but retaining these engineering units makes the controller meaning clearer.

### 13.1 The negative sign

Assume the path points along positive x and the robot is 0.2 m above it, on its left side:

```text
e_y = +0.2 m
```

The outer PID output is positive for positive gains. A correction back toward the path must turn the desired heading clockwise/right, which is negative yaw. Therefore:

```text
heading_correction = -u_cte
```

For a robot on the right side, `e_y` is negative, the negation produces a positive/left correction, and the symmetry is preserved.

### 13.2 Why limit the correction to 0.7 rad

`0.7 rad` is approximately 40 degrees. A bounded heading correction prevents a large lateral error or derivative spike from asking the inner loop to point almost perpendicular or backwards relative to the path. It also gives the controller a well-defined safety envelope for tuning.

The outer PID continues to calculate its raw output even when the correction is clamped. Since current `Ki_cte` is zero, integral windup has no effect on the output at present. If a nonzero integral gain is introduced later, saturation behavior should be evaluated carefully.

---

## 14. Inner loop: desired heading to yaw-rate feedback

The corrected inner error is:

```text
e_heading = wrap(desired_heading - robot_heading)
```

Then:

```text
u_heading = PID_heading(e_heading, dt)
w_cmd = clamp(w_ref + u_heading, -1.5, +1.5)
```

With the current cascade YAML:

```text
Kp_heading = 3.0
Ki_heading = 0.0
Kd_heading = 0.20
integral_state_limit = 0.5
maximum angular command = 1.5 rad/s
```

The intended engineering dimensions are:

- `Kp_heading`: (rad/s)/rad, commonly read as 1/s;
- `Ki_heading`: (rad/s)/(rad*s), commonly read as 1/s^2;
- `Kd_heading`: (rad/s)/(rad/s), dimensionless;
- inner integral state: rad*s;
- inner output: rad/s.

### 14.1 Feedback plus feedforward

The angular command has two conceptually different parts:

```text
w_cmd = w_ref + u_heading
```

- `w_ref = v_ref*kappa` is **feedforward** based on known path geometry.
- `u_heading` is **feedback** based on measured tracking error.

On a curve with perfect position and heading, PID feedback can be zero while `w_ref` remains nonzero. This lets the robot keep turning with the path rather than waiting to develop an error first.

On a straight path, curvature is zero, so `w_ref = 0` and all turning comes from feedback.

### 14.2 Why a spun robot is corrected immediately

Suppose the robot is exactly on a straight path:

```text
e_y = 0
path_heading = 0
```

The outer correction is zero, so:

```text
desired_heading = 0
```

Now suppose an external effect rotates the robot to `+pi/2` rad without yet moving its position sideways:

```text
robot_heading = +pi/2
e_heading = wrap(0 - pi/2) = -pi/2
```

The inner loop immediately receives a large negative error and commands a right turn. It does not wait for cross-track error to grow. This is the essential answer to the concern that lateral correction alone might be slow after a pure rotation: heading is an independently measured inner-loop variable.

---

## 15. Forward-speed command

Forward speed is not produced by a third PID in cascade mode. It begins with the common geometry-based `v_ref` and is multiplied by two safety/performance factors.

### 15.1 Heading factor

```text
heading_speed_factor = max(0, cos(e_heading))
```

Examples:

- heading error `0 deg`: factor `1`, no reduction;
- heading error `60 deg`: factor `0.5`;
- heading error `90 deg`: factor `0`;
- error beyond `90 deg`: cosine is negative but `max(0, ...)` keeps the factor at zero.

Thus a badly rotated robot turns toward the desired direction before it drives forward in the wrong direction.

### 15.2 Cross-track factor

```text
cross_track_speed_factor = 1 / (1 + 1.5*abs(e_y))
```

This factor is symmetric on either side of the path. It approaches 1 at zero displacement and decreases smoothly as displacement grows.

### 15.3 Final linear command

```text
v_cmd = clamp(
    v_ref * heading_speed_factor * cross_track_speed_factor,
    0,
    1.0)
```

The lower bound is zero, so this controller never requests reverse motion. The upper bound is the configured actuator-level safety limit.

There are therefore three successive reasons for slowing down:

1. path curvature, applied by the common reference manager;
2. proximity to the endpoint, applied by the common reference manager;
3. current tracking difficulty, applied by the cascaded controller through heading and cross-track factors.

---

## 16. One complete cascade update, step by step

For every valid filtered odometry message, the exact logical sequence is:

1. Read robot `x` and `y` from the odometry pose.
2. Convert the orientation quaternion into yaw.
3. Reject non-finite pose data.
4. Read the ROS timestamp.
5. Calculate `dt`, or use the safe nominal value when required.
6. Update watchdog state to mark odometry healthy.
7. Project the robot onto nearby forward path segments.
8. Select the closest valid projection without decreasing stored progress.
9. Calculate path position, tangent, progress, remaining length, and curvature.
10. Calculate signed cross-track error.
11. Calculate raw path-heading error for common evaluation.
12. Calculate curvature- and endpoint-adjusted `v_ref`.
13. Calculate curvature feedforward `w_ref = v_ref*kappa`.
14. Check completion using both progress and Euclidean goal distance.
15. Run the outer cross-track PID.
16. Negate and clamp the outer output into a heading correction.
17. Add it to the path tangent to form desired heading.
18. Compare desired heading with measured robot heading.
19. Run the inner heading PID.
20. Add curvature feedforward and clamp the angular command.
21. Calculate heading and cross-track speed factors.
22. Multiply and clamp the linear command.
23. Put `v_cmd` in `Twist.linear.x`.
24. Put `w_cmd` in `Twist.angular.z`.
25. Publish the `Twist`.
26. Write the measured state, path reference, errors, diagnostics, and nominal commands to CSV.

The actual P and D arithmetic is only a few lines. Most surrounding code exists to make sure those few lines receive the correct geometry, time interval, configuration, and safe data and to make the experiment repeatable and measurable.

---

## 17. Illustrative numerical update

The following is an illustrative calculation, not a row copied from a real test. It omits derivative history by assuming both derivatives are zero in this instant.

Assume:

```text
cross-track error e_y = +0.10 m
path heading          = 0.20 rad
robot heading         = 0.10 rad
v_ref                 = 0.40 m/s
w_ref                 = 0.08 rad/s
dt                    = 0.10 s
```

Outer proportional output:

```text
u_cte = 1.5 * 0.10 = 0.15 rad
```

The robot is left of the path, so correction is negative:

```text
heading_correction = -0.15 rad
desired_heading = 0.20 - 0.15 = 0.05 rad
```

Inner heading error:

```text
e_heading = 0.05 - 0.10 = -0.05 rad
```

Inner proportional output:

```text
u_heading = 3.0 * (-0.05) = -0.15 rad/s
```

Angular command:

```text
w_cmd = w_ref + u_heading
      = 0.08 - 0.15
      = -0.07 rad/s
```

Speed factors:

```text
heading_factor = cos(-0.05) approximately 0.9988
cross-track factor = 1/(1 + 1.5*0.10) approximately 0.8696
```

Linear command:

```text
v_cmd = 0.40 * 0.9988 * 0.8696
      approximately 0.347 m/s
```

This example also shows why the angular command cannot be understood from cross-track error alone. The path curvature feedforward and the robot's measured heading both participate.

---

## 18. Four headings that must not be confused

The code logs several angular quantities because each serves a different purpose.

### 18.1 `reference_yaw`

This is the local path tangent heading. It describes the curve, independent of controller correction.

### 18.2 `path_heading_error`

```text
wrap(reference_yaw - actual_yaw)
```

This is the common geometric evaluation error and should be comparable across PID, LQR, and MPC.

### 18.3 `desired_heading`

```text
wrap(reference_yaw + heading_correction)
```

This is the direction the cascade temporarily asks the robot to face in order to remove cross-track error.

### 18.4 `control_heading_error`

In cascade mode:

```text
wrap(desired_heading - actual_yaw)
```

This is the error actually sent to the inner PID. It can differ from `path_heading_error` whenever the outer loop is asking the robot to converge laterally.

For example, a robot can be parallel to the path, making `path_heading_error = 0`, but displaced left. The outer loop then chooses a right-tilted desired heading, so `control_heading_error < 0` and the robot approaches the path.

---

## 19. Publishing the command and moving the robot

The PID node creates a `geometry_msgs::msg::Twist` message:

```text
command.linear.x  = linear_command
command.angular.z = angular_command
```

All unused components remain zero. A planar differential-drive robot cannot independently command lateral body velocity, so `linear.y` is not used.

ROS publishing is asynchronous. The PID does not call a Gazebo motor function directly. It places a typed message on a topic; the Gazebo plugin receives it through ROS middleware and performs the body-to-wheel conversion.

The publisher/subscriber queue depth is 10. This is middleware buffering, not ten control iterations executed at once.

---

## 20. Disturbance-injection data flow

The disturbance experiment is intentionally downstream of the controller.

### 20.1 Scheduling

The injector's first received nominal command defines elapsed time zero. For every later command:

```text
elapsed = ROS_now - first_command_time
```

The fault is active in a half-open time interval:

```text
start_delay <= elapsed < start_delay + duration
```

A small numerical tolerance prevents a timestamp infinitesimally below the mathematical end from adding one unintended final fault sample.

The standard visible test uses:

```text
start delay = 5.0 s
duration = 1.0 s
linear bias = 0.0 m/s
angular bias = +0.6 rad/s
```

The smaller defaults embedded in the C++ constructors are fallbacks. Launch values override them for this experiment.

### 20.2 Applied command

When inactive:

```text
v_applied = clamp(v_nominal)
w_applied = clamp(w_nominal)
```

When active:

```text
v_applied = clamp(v_nominal + linear_bias)
w_applied = clamp(w_nominal + angular_bias)
```

The standard limits are `1.0 m/s` and `1.5 rad/s` in magnitude.

### 20.3 Why this is preferable to the earlier force disturbance

A physical force applied at an unsuitable point can alter wheel normal forces or lift the robot, especially with the frictionless spherical caster. Then the trial tests contact mechanics and the disturbance implementation as much as controller rejection. A command-path yaw bias preserves wheel contact and represents a temporary actuator, steering, calibration, or command-channel fault more cleanly.

### 20.4 Watchdog

If nominal commands stop arriving for more than 2 wall-clock seconds, the injector publishes zero velocity. It uses `steady_clock`, so safety still works if simulation time pauses.

### 20.5 What the PID knows during the fault

The PID continues to publish nominal commands and does not know the bias value. Gazebo receives the biased yaw rate, the robot deviates, the EKF observes that deviation, and subsequent PID updates react to the changed cross-track and heading errors.

---

## 21. Completion and stopping logic

Completion requires two conditions:

```text
remaining_path_length <= goal_tolerance
distance_to_final_waypoint <= goal_tolerance
```

The current tolerance is `0.08 m`.

Both conditions are required because a circle or figure eight can end close to where it starts. Goal distance alone could claim completion immediately at the beginning. Progress alone could claim completion if projection advanced to the end while the robot was physically far from the final point.

On completion, the node:

1. marks the track complete;
2. publishes a zero `Twist`;
3. logs a terminal row with zero commands;
4. flushes the CSV;
5. reports `Track complete`.

If later odometry messages arrive, the node reinforces the stop rather than restarting the path.

---

## 22. Controller experiment CSV: all 26 columns

The PID output file is the main source for MATLAB analysis.

| Column | Unit | Meaning |
|---|---:|---|
| `time` | s | Time since first accepted odometry sample |
| `actual_x` | m | EKF-estimated robot x |
| `actual_y` | m | EKF-estimated robot y |
| `actual_yaw` | rad | EKF-estimated robot heading |
| `reference_x` | m | Continuous projected path x |
| `reference_y` | m | Continuous projected path y |
| `reference_yaw` | rad | Local path tangent heading |
| `cross_track_error` | m | Signed lateral path error |
| `path_heading_error` | rad | Path tangent minus robot heading |
| `control_heading_error` | rad | Actual error used by active heading controller |
| `linear_command` | m/s | Nominal PID forward command |
| `angular_command` | rad/s | Nominal PID yaw-rate command |
| `waypoint_index` | index | Nearest endpoint representation used for diagnostics |
| `segment_index` | index | Selected path segment |
| `segment_fraction` | 0 to 1 | Projection fraction `t` on that segment |
| `path_progress` | m | Monotonic arc length from path start |
| `remaining_path_length` | m | Arc length still remaining |
| `reference_curvature` | 1/m | Estimated local signed curvature |
| `reference_linear_velocity` | m/s | Common geometry-based `v_ref` |
| `reference_angular_velocity` | rad/s | Common curvature feedforward `w_ref` |
| `desired_heading` | rad | Outer-loop corrected heading |
| `heading_correction` | rad | Bounded signed outer correction |
| `cross_track_pid_output` | rad intended | Raw outer PID output before negation/clamp |
| `heading_pid_output` | rad/s | Inner feedback before adding feedforward |
| `heading_speed_factor` | dimensionless | `max(0, cos(inner error))` |
| `cross_track_speed_factor` | dimensionless | Lateral-error slowdown factor |

During a disturbance run, `linear_command` and `angular_command` remain the **nominal controller outputs**. Use the injector CSV to see what Gazebo actually received.

---

## 23. Disturbance CSV: all 6 columns

| Column | Unit | Meaning |
|---|---:|---|
| `time` | s | Time since injector's first nominal command |
| `nominal_linear_command` | m/s | PID-requested forward speed |
| `nominal_angular_command` | rad/s | PID-requested yaw rate |
| `applied_linear_command` | m/s | Forward speed delivered to Gazebo after bias/clamp |
| `applied_angular_command` | rad/s | Yaw rate delivered to Gazebo after bias/clamp |
| `fault_active` | 0 or 1 | Whether the scheduled bias was active |

The controller and injector clocks begin from slightly different triggering events, so analysis scripts use the actual logged fault transition rather than assuming it occurs at exactly the controller's 5.000 s row.

---

## 24. Source-file analysis

### 24.1 `pid_controller.hpp` and `pid_controller.cpp`

These files contain the smallest reusable control primitive.

The header contains:

- public constructor;
- `configure()`;
- `calculate()`;
- `reset()`;
- private gains and retained state.

The implementation contains parameter validation and the discrete P, I, and D equations. It has no ROS includes, topics, files, or Gazebo calls. This separation makes the control arithmetic reusable and directly testable.

### 24.2 `cascaded_pid_controller.hpp` and `.cpp`

The header defines two plain data structures:

- `CascadedPidConfig`: everything tunable;
- `CascadedPidOutput`: command plus diagnostics.

It then declares `CascadedPidController`, which owns the two generic PIDs.

The implementation:

1. validates gains and limits;
2. configures both PID objects;
3. calculates outer correction;
4. calculates inner feedback;
5. adds curvature feedforward;
6. calculates speed factors;
7. applies command limits;
8. exposes reset for both loops.

This file also has no ROS dependency. That is architecturally important: controller mathematics can be tested without launching a robot simulator.

### 24.3 `path_reference_manager.hpp` and `.cpp`

These files own all controller-independent path geometry:

- CSV parsing and validation;
- segment preprocessing;
- curvature approximation;
- projection;
- signed error calculation;
- monotonic progress;
- reference speed and yaw-rate generation;
- lookahead access for the older controller.

Centralizing this logic prevents a future LQR from receiving a slightly different cross-track error than PID, which would make a thesis comparison less fair.

### 24.4 `pid_node.hpp`

The node header is the structural overview of the ROS adapter. It declares:

- the selectable controller mode enum;
- logged diagnostics;
- callbacks and helper functions;
- path manager and controller objects;
- ROS publisher, subscriber, and watchdog timer;
- latest pose;
- timing and safety settings;
- lifecycle flags.

Members ending in `_` are class-owned state. This trailing-underscore convention helps distinguish persistent members from local variables and function arguments.

### 24.5 `pid_node.cpp`

This is the integration layer. It is long because it connects configuration, files, ROS messages, timing, safety, geometry, controller selection, publication, and logging.

Its main sections are:

- constructor: build and validate the experiment;
- destructor: stop and close files;
- `publish_stop()`: send an all-zero `Twist`;
- `reset_controllers()`: clear all PID memory;
- `watchdog_callback()`: react to missing odometry;
- `odom_callback()`: extract state and determine `dt`;
- `control_loop()`: reference, completion, control, publish;
- `log_sample()`: serialize one 26-column row;
- `main()`: initialize ROS, construct, spin, and shut down.

`rclcpp::spin(node)` is the event loop. It waits for incoming messages and timer events, then invokes registered callbacks. Without `spin`, constructing subscriptions would not cause callbacks to execute.

### 24.6 `command_disturbance.hpp` and `.cpp`

These define controller-independent fault arithmetic. The class receives scalar nominal velocities and elapsed time and returns nominal/applied velocities plus a Boolean state. Because it has no ROS dependency, timing boundaries, saturation, and sign behavior can be unit-tested deterministically.

### 24.7 `command_disturbance_injector_node.cpp`

This is the ROS adapter around the pure disturbance class. It:

- reads fault parameters;
- subscribes to nominal `Twist` messages;
- calculates elapsed simulation time;
- calls `CommandDisturbance::apply()`;
- republishes the modified `Twist`;
- logs both versions;
- reports fault start/end transitions;
- stops the robot if controller input disappears.

### 24.8 YAML configuration

`pid_cascade.yaml` is the experiment tuning record. It selects cascade mode and supplies all gains, reference settings, limits, and timing values. The C++ defaults are safety fallbacks and parameter declarations; the YAML is where normal experimental tuning should occur.

To make the robot modestly faster in a normal tuned experiment, change `reference_linear_velocity` in a YAML profile. Do not duplicate the same tuning change in C++ unless the intention is to change the fallback default used when no YAML supplies that parameter.

### 24.9 Launch files

Launch files describe which processes run, their parameters, and topic remappings. They do not implement PID mathematics. Their main scientific role is reproducibility: the same executable can be used with a named configuration, track, output location, and fault schedule.

### 24.10 URDF and EKF YAML

The URDF is not merely visual geometry. It specifies masses, inertias, wheels, joints, friction, sensors, and the differential-drive actuator plugin. The EKF YAML decides which simulated measurements form the pose fed back to the controller. Both influence closed-loop behavior even though neither contains a PID gain.

### 24.11 CMake

`CMakeLists.txt` tells the compiler which libraries and executables exist and how they depend on each other:

```text
pid_controller library --------+
                               |
path_reference_manager library +--> cascaded_pid_controller library
                               |                 |
                               +-----------------+--> pid_node executable

command_disturbance library --> command_disturbance_injector executable
```

It also registers unit tests and installs headers, configuration, launch files, and five tracks so ROS can locate them through the package index.

---

## 25. Header files versus implementation files

For a programmer coming from MATLAB, a useful approximation is:

- `.hpp` says **what exists and what can be called**;
- `.cpp` says **how it works**.

The header lets one source file compile against a class without copying its implementation everywhere. Private members hide internal state so outside code cannot accidentally overwrite the PID integral or previous error.

For example:

```cpp
double calculate(double error, double dt);
```

in the header declares the interface. The code beginning with:

```cpp
double PIDController::calculate(double error, double dt)
```

in the `.cpp` defines the function. The `::` means “the `calculate` function belonging to `PIDController`.”

---

## 26. Why there are many lines when the equation is short

Yes: much of the project is “plumbing,” but that plumbing is scientifically important.

The actual generic PID calculation is approximately:

```text
integral += error*dt
derivative = (error - previous_error)/dt
output = Kp*error + Ki*integral + Kd*derivative
```

Everything else answers necessary experimental questions:

- How is error defined on a curved or self-intersecting path?
- Where did the measured pose come from?
- Are angles wrapped correctly?
- What is `dt` if Gazebo slows down?
- What happens on missing or corrupted data?
- How are gains selected without recompiling?
- What prevents unsafe command values?
- How does the robot stop at the endpoint?
- How can MATLAB reconstruct every internal quantity?
- How can a disturbance be identical for PID, LQR, and MPC?
- How can core behavior be tested without manually watching Gazebo?

The controller law is the small mathematical core. The surrounding software makes that law observable, repeatable, safe, and comparable. For a thesis, these properties are not decorative code; they support the validity of the experiment.

---

## 27. Test-suite analysis

The project has three focused C++ functional test executables plus ROS package style/metadata checks.

### 27.1 Path-reference tests: 8 cases

They verify:

1. projection onto a straight segment;
2. cross-track sign on both sides;
3. projection clamping before/after a finite path;
4. independence of heading and cross-track errors;
5. approximate unit-circle curvature and reference speed;
6. correct progress through a closed/self-intersecting path;
7. rejection of duplicate consecutive waypoints;
8. endpoint speed ramping to zero.

These tests protect the definitions used for performance metrics, not just implementation details.

### 27.2 Cascade tests: 6 cases

They verify:

1. aligned, on-path motion uses common reference speed;
2. a robot left of the path is commanded right;
3. a sudden heading rotation causes an immediate inner-loop response even at zero CTE;
4. curvature feedforward remains active when errors are zero;
5. heading correction and actuator limits are applied;
6. invalid configuration is rejected.

### 27.3 Command-disturbance tests: 8 cases

They verify:

1. nominal pass-through before the fault;
2. bias activation at the start boundary;
3. activation inside the window;
4. deactivation at the end boundary;
5. robustness to floating-point timestamp rounding near the end;
6. reusable linear bias behavior;
7. command saturation;
8. rejection of invalid duration.

### 27.4 Why the complete report contains more than 22 tests

The 22 cases above are the project-specific functional tests. `colcon test` also runs automatically registered checks for formatting, XML/package metadata, Python style, CMake style, and similar package quality rules. That is why the previously validated total was 72 tests, with 14 skipped checks and no errors or failures.

Unit tests do not replace Gazebo trials. They answer deterministic questions about equations, signs, boundaries, and geometry. Gazebo trials answer system-level questions involving dynamics, contact, estimator behavior, timing, and recovery.

---

## 28. Benchmark script and metrics

`scripts/run_pid_benchmarks.sh` runs named profiles on:

- straight;
- smooth curve;
- corner;
- circle;
- figure eight.

Each trial starts a fresh headless Gazebo instance so robot state, EKF state, and simulation time do not leak from one run into another.

The summary calculates:

- success/completion;
- duration;
- final Euclidean position error;
- time-weighted cross-track RMSE;
- maximum absolute cross-track error;
- time-weighted path-heading RMSE;
- maximum absolute path-heading error;
- integrated control effort;
- maximum absolute forward command;
- maximum absolute yaw-rate command;
- final position;
- number of logged samples.

The control-effort definition is fixed as:

```text
integral(v_cmd^2 + 0.2*w_cmd^2) dt
```

The coefficient `0.2` is a chosen numerical weighting, not a physical energy model. It must remain unchanged across controllers for the comparison to be meaningful.

### 28.1 Disturbance recovery script

`scripts/run_cascade_command_disturbance.sh` runs the 5 m straight track with the scheduled yaw bias. Recovery requires ten consecutive controller samples satisfying both:

```text
abs(cross_track_error) <= 0.005 m
abs(path_heading_error) <= 0.01 rad
```

The ten-sample condition avoids declaring recovery from one noisy or momentary threshold crossing.

The final headless validation with the tuned cascade profile produced:

```text
fault duration:                   1.000 s
peak absolute cross-track error:  0.052960 m
peak absolute path-heading error: 0.150037 rad (about 8.60 deg)
recovery after fault:             4.404 s
final absolute cross-track error: 0.000110 m
track completed:                  yes
```

These values describe this controller, path, disturbance, model, estimator, and threshold definition together. They should not be generalized as universal PID performance.

---

## 29. Important implementation limitations and thesis interpretation

### 29.1 Path following, not strict timed tracking

The reference is indexed by nearest monotonic path progress, not by a desired time schedule. This is a deliberate thesis-scope decision. The comparison evaluates path-tracking quality, completion time, command activity, and disturbance recovery without introducing a time-parameterized reference layer.

### 29.2 Piecewise-linear path

Heading is constant within each segment and curvature is an estimate based on neighboring segments. Dense, smooth waypoint generation makes this approximation better. A sharp corner remains a non-smooth reference and is intentionally a demanding benchmark.

### 29.3 No reverse behavior

The cascade clamps forward speed at zero and assumes monotonically increasing progress. It is a forward-only path follower.

### 29.4 Derivative on error, without filtering

Both PIDs differentiate error directly and do not low-pass filter the derivative. EKF filtering helps, but aggressive `Kd` can amplify measurement noise or reference discontinuities. A future refinement might use a filtered derivative or derivative on measurement, but changing that would define a different controller and should be documented as such.

### 29.5 Integral gains are currently zero

Both cascade `Ki` values are zero. Strictly speaking, the implemented software is PID-capable, while the current tuned cascade behaves as two PD feedback loops plus curvature feedforward and nonlinear speed scheduling. It is acceptable to call the architecture cascaded PID, but the thesis should transparently report the actual tuned gains.

Two targeted checks support retaining this choice. The robot completed a 15 m straight containing a sustained 15-degree incline, with horizontal speed decreasing from approximately 0.400 m/s on the flat to 0.382 m/s on the ramp and with no stalled samples. A body speed of 0.4 m/s on a 15-degree ramp projects to 0.386 m/s horizontally; the measurement therefore corresponds to approximately 0.396 m/s along the ramp and shows almost no loss of commanded speed. This load is handled by the lower-level Gazebo differential-drive velocity actuation; lateral or heading integral action would not directly increase longitudinal wheel torque.

Under a deliberately persistent `+0.08 rad/s` yaw-command bias, the proportional cascade settled with a mean CTE of `0.01731 m`. The proportional equilibrium prediction is `bias/(Kp_heading*Kp_cross_track) = 0.01778 m`. The small nonzero offset is therefore expected: it supplies the feedback command needed to cancel a constant bias.

A larger study applied `+0.20 rad/s` for 15 seconds and compared inner heading `Ki` values of `0`, `0.2`, `0.4`, and `0.6`. Increasing `Ki` progressively reduced the final active-fault mean CTE from `0.04445 m` to `0.00464 m`; `Ki=0.6` reduced active-fault CTE IAE by about 65%. It also improved nominal circle and figure-eight CTE RMSE. After fault removal, however, every nonzero candidate produced a larger opposite-side residual, and `Ki=0.4` and `0.6` failed the strict recovery criterion before the endpoint. On the original one-second strong fault, `Ki=0` recovered in `4.404 s` and ended at `0.00011 m` CTE, whereas `Ki=0.6` did not meet the recovery criterion and ended at `0.00829 m`.

The selected `Ki=0` is consequently a documented priority choice: it favors rejection and recovery from temporary faults. If rejection of permanent actuator bias becomes the primary objective, `Ki=0.6` is the better tested steady-state choice.

### 29.6 Integral clamping is simple

The stored integral is bounded, but there is no back-calculation based on final angular-command saturation or outer heading-correction saturation. More importantly, clamping cannot remove every integral transient. In the `Ki=0.4` and `Ki=0.6` constant-bias trials, the estimated integral states remained inside their limits and the angular command did not saturate. Their post-fault overshoot was therefore not saturation windup: the legitimate stored command needed to cancel the constant fault had to unwind after that fault disappeared. Conditional integration based only on saturation would not eliminate this memory tradeoff.

### 29.7 Controller update rate follows incoming odometry

The nominal frequency supplies a fallback `dt`; it is not a control timer. The real update sequence follows `/odometry/filtered`. When analyzing results, CSV timestamps are the authority rather than assuming exactly 30 samples each second.

### 29.8 Nominal versus applied effort during faults

The PID CSV contains nominal commands. For a disturbance experiment, physical input effort and saturation analysis must use the injector's applied-command CSV or explicitly distinguish the two definitions.

### 29.9 The state estimate is part of the evaluated system

The controller acts on EKF output, not perfect Gazebo ground truth. Therefore
estimator drift and delay can change its commands and the resulting physical
path. Formal tracking metrics are now calculated independently from
`/ground_truth/odom`, while the controller CSV records what the controller
believed. The evaluator also reports localization error after interpolating
truth to the EKF message timestamp. This separation preserves realistic
feedback without allowing the estimator to grade itself.

---

## 30. What to tune and where

Normal experimental changes belong in YAML, not duplicated in C++.

For the cascade:

- outer responsiveness: `cascade_cross_track_kp`, `ki`, `kd`;
- inner heading response: `cascade_heading_kp`, `ki`, `kd`;
- maximum lateral steering intention: `cascade_max_heading_correction`;
- slowdown while displaced: `cascade_cross_track_speed_gain`;
- ordinary desired speed: `reference_linear_velocity`;
- curve slowdown: `curvature_speed_gain`;
- endpoint braking distance: `endpoint_slowdown_distance`;
- final safety limits: `max_linear_velocity`, `max_angular_velocity`.

C++ defaults should be changed only when changing the software's fallback behavior or interface. A tuning profile must not require recompilation.

The outer and inner loops should be interpreted separately during tuning:

1. make the inner heading response stable and sufficiently fast;
2. then increase outer lateral response without demanding headings the inner loop cannot follow;
3. evaluate the speed schedule because faster translation changes how much time is available to correct lateral error;
4. rerun all benchmark shapes and the identical disturbance, not only a straight nominal track.

---

## 31. Beginner C++ and ROS glossary for this code

- **Class**: a data structure plus functions that operate on its stored data.
- **Object**: one instance of a class, such as `cascaded_pid_`.
- **Constructor**: function run when an object is created; used here for configuration.
- **Destructor**: function run during orderly destruction; used to stop and close files.
- **Member variable**: state stored inside an object, often ending in `_` here.
- **Local variable**: temporary value existing only inside one function call.
- **`const`**: promises that a value or referenced object will not be modified through that name.
- **`&`**: reference; allows access without copying a complete structure.
- **`::`**: scope operator, meaning “inside this namespace/class.”
- **`std::`**: C++ standard-library namespace.
- **`SharedPtr`**: reference-counted pointer used by ROS messages and communication objects.
- **Callback**: function ROS calls when an event occurs, such as new odometry.
- **Publisher**: typed output endpoint for a ROS topic.
- **Subscriber**: typed input endpoint for a ROS topic.
- **Node**: one ROS participant that owns parameters, publishers, subscriptions, timers, and logging.
- **Topic**: named asynchronous message channel.
- **Message**: typed data object sent on a topic, such as `Twist` or `Odometry`.
- **Launch file**: Python description of processes, parameters, and remappings.
- **YAML**: human-editable parameter data.
- **CMake**: build instructions describing libraries, executables, dependencies, tests, and installation.
- **Clamp**: force a value to remain between a minimum and maximum.
- **Finite**: not infinity and not “not-a-number” (`NaN`).
- **Namespace**: grouping used to avoid naming collisions.
- **Enum**: restricted set of named choices, used here for lookahead versus cascade.
- **Struct**: simple class used mainly to group related values.

---

## 32. Compact signal dictionary

| Symbol/name | Interpretation |
|---|---|
| `x, y` | Filtered robot position in `odom` |
| `theta` | Filtered robot yaw |
| `t` | Fraction along one segment, not experiment time |
| `s` or `path_progress` | Arc length along full path |
| `e_y` | Signed cross-track error |
| `path_heading` | Tangent direction of current path segment |
| `desired_heading` | Tangent plus outer-loop correction |
| `e_heading` | Desired heading minus robot heading |
| `kappa` | Signed reference curvature |
| `v_ref` | Common path-based reference speed |
| `w_ref` | Curvature yaw-rate feedforward |
| `u_cte` | Raw outer PID output |
| `u_heading` | Inner PID yaw-rate feedback |
| `v_cmd` | Nominal forward command from cascade |
| `w_cmd` | Nominal angular command from cascade |
| `v_applied, w_applied` | Commands after optional disturbance and limits |
| `dt` | Time between accepted odometry updates |

---

## 33. Final mental model

The easiest way to understand the current system is to divide it into five layers:

1. **Simulation and sensing**: Gazebo moves the robot and produces wheel odometry and IMU measurements.
2. **State estimation**: the EKF produces the filtered planar pose used for feedback.
3. **Reference management**: the path manager turns a CSV curve plus robot pose into progress, local geometry, errors, and reference velocities.
4. **Control law**: the outer PID turns lateral error into a desired heading; the inner PID turns heading error into yaw-rate feedback; feedforward anticipates curvature; speed scheduling reduces forward motion when appropriate.
5. **Experiment infrastructure**: ROS topics carry data, launch/YAML make runs reproducible, watchdogs stop unsafe motion, CSV logs expose the calculation, disturbance injection tests rejection, and automated tests protect definitions and boundaries.

In one sentence:

> Each filtered pose is projected onto the ordered path; the outer loop decides which way the robot should point to remove lateral error, the inner loop decides how fast it should rotate toward that direction, the speed logic decides how safely it should advance, and Gazebo's resulting motion becomes the next filtered pose.
