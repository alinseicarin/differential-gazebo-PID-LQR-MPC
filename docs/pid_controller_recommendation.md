# Recommended PID Architecture for the Thesis

## 1. Recommendation in brief

The current waypoint/lookahead PID controller is useful and should be preserved
as the first working baseline. However, it should not be the main PID design in
the final PID-versus-LQR-versus-MPC comparison.

The recommended thesis controller is a cascaded path-error PID based on:

- Signed cross-track error, written as `e_y`.
- Heading error, written as `e_heading`.
- A reference forward velocity, written as `v_ref`.
- Reference path curvature, written as `kappa_ref`.

This gives PID, LQR, and MPC a common set of reference quantities and error
coordinates.

## 2. Is lookahead inherently bad?

No. Lookahead controllers are popular because they are simple, computationally
cheap, and often effective. Pure Pursuit is a well-known example.

The issue is experimental fairness. The current controller regulates distance
and bearing to a waypoint five samples ahead. A later LQR or MPC controller will
probably regulate geometric states such as cross-track and heading error.
Therefore, the controllers would not be solving exactly the same problem.

The current implementation is also sensitive to waypoint density. Five points
represent 0.25 m when points are spaced by 0.05 m, but 1.0 m when points are
spaced by 0.20 m.

For accurate reporting, the current implementation should be described as:

> A waypoint-based PID path-following controller using longitudinal distance
> error and lookahead heading error.

## 3. Recommended project progression

Keep the present controller as:

> PID Version 1: waypoint/lookahead PID baseline.

Implement the comparison controller as:

> PID Version 2: cascaded cross-track and heading-error PID.

PID Version 2 should be used in the main PID/LQR/MPC comparison. Version 1 can
remain in the development history or appear as a preliminary baseline.

## 4. Common reference manager

Before any controller calculates a command, a shared reference manager should
calculate:

- Path progress `s`.
- Closest projected path point `(x_ref, y_ref)`.
- Reference heading `heading_ref`.
- Reference curvature `kappa_ref`.
- Signed cross-track error `e_y`.
- Heading error `e_heading`.
- Reference forward velocity `v_ref`.

The conceptual architecture is:

```text
Path CSV
   |
   v
Common reference manager
   |
   +-- cross-track error
   +-- heading error
   +-- reference velocity
   +-- reference curvature
          |
          v
   PID, LQR, or MPC
          |
          v
     v_cmd, omega_cmd
```

Every controller should receive equivalent reference information and should
respect identical command limits.

## 5. Proposed cascaded PID controller

### 5.1 Outer cross-track loop

The outer PID converts lateral displacement into a desired heading correction:

```text
heading_correction = -(Kp_y * e_y
                     + Ki_y * integral(e_y dt)
                     + Kd_y * de_y/dt)
```

The desired heading becomes:

```text
heading_desired = heading_ref + heading_correction
```

The sign convention must be verified against the implemented cross-track error.
With the current convention, a robot to the left of the path should be asked to
turn toward the right.

The heading correction should be limited, for example to 30 or 45 degrees. This
prevents a large position error from requesting an unreasonable orientation.

### 5.2 Inner heading loop

The inner-loop error is:

```text
e_inner = wrap_angle(heading_desired - heading_actual)
```

The angular command is:

```text
omega_cmd = omega_feedforward
          + Kp_heading * e_inner
          + Ki_heading * integral(e_inner dt)
          + Kd_heading * de_inner/dt
```

The curvature feedforward term is:

```text
omega_feedforward = v_ref * kappa_ref
```

Feedforward supplies the nominal turning rate implied by the reference path.
The PID feedback then corrects model mismatch and tracking error.

### 5.3 Linear velocity

For the main comparison, all controllers should use the same nominal reference
velocity. The current PID should not obtain nominal speed from distance to an
arbitrary lookahead waypoint because that makes speed depend on waypoint
spacing and PID gains.

A simple common speed policy is:

```text
v_cmd = v_ref * slowdown_factor
```

The slowdown factor may decrease when:

- Absolute cross-track error is large.
- Absolute heading error is large.
- Reference curvature is high.
- The robot is approaching the endpoint.

The same policy should either be used by all controllers or be treated as part
of each controller and evaluated explicitly. This decision must be stated in
the experimental methodology.

An initial comparison at one fixed reference speed, such as 0.4 or 0.5 m/s,
would be easy to interpret. A later speed sweep could use:

```text
0.2 m/s
0.4 m/s
0.6 m/s
0.8 m/s
```

This would show how tracking performance changes as the demanded speed rises.

## 6. Why this is better for the thesis

The proposed controller:

- Directly regulates the cross-track quantity used for evaluation.
- Is independent of waypoint count and waypoint density.
- Uses reference curvature as physically meaningful feedforward.
- Has a clear cascaded control-theory interpretation.
- Uses error variables that can also be supplied to LQR and MPC.
- Makes tuning decisions easier to explain and defend.
- Produces a fairer controller comparison.

## 7. Methods that should not replace the main PID

Pure Pursuit and Stanley may be useful path followers, but they are separate
controller families. Replacing PID with either method would broaden the thesis
from PID versus LQR versus MPC to a larger comparison.

They could be discussed as related work or included as an optional additional
benchmark, but they are not necessary for the bachelor's-thesis scope.

## 8. Suggested implementation and experimental sequence

1. Preserve and document the existing lookahead PID.
2. Implement the cascaded cross-track and heading PID.
3. Resample reference paths at consistent spatial spacing.
4. Calculate path tangent and curvature robustly.
5. Establish a common reference-speed policy.
6. Tune PID only on a designated tuning track or tuning set.
7. Freeze gains before formal evaluation.
8. Evaluate on separate straight, curve, corner, circle, figure-eight, and
   disturbance tracks.
9. Use the same reference definitions, actuator limits, and metrics for LQR and
   MPC.
10. Repeat formal trials and report mean, standard deviation, and, if useful,
    confidence intervals.

This scope is ambitious enough for a strong bachelor's thesis without adding
unnecessary controller families.
