"""
Generate deterministic, headerless x,y paths for controller benchmarks.

Straight and discontinuous-corner tracks use a 5 cm waypoint spacing. Smooth
curves use a 1 cm spacing so their polygonal representation does not inject
large artificial heading steps into the derivative term of a controller. The
files deliberately contain no header because the C++ path loader expects
exactly two numeric columns on every row. Running this script replaces all
seven track CSV files in the current directory.
"""

import math
import csv


STRAIGHT_SPACING = 0.05
SMOOTH_TRACK_SPACING = 0.01
COORDINATE_DECIMALS = 5


# Round exactly as the CSV writer will, normalizing negative zero for clean diffs.
def quantize_waypoint(waypoint):
    """Apply exactly the coordinate quantization written to each CSV file."""
    rounded = [round(coordinate, COORDINATE_DECIMALS) for coordinate in waypoint]
    return tuple(0.0 if coordinate == 0.0 else coordinate for coordinate in rounded)


# Normalize angular differences so tangent changes use the shortest rotation.
def wrap_angle(angle):
    """Return the shortest signed representation of an angle in radians."""
    return math.atan2(math.sin(angle), math.cos(angle))


# Verify that quantization has not introduced duplicates or abrupt tangent jumps.
def validate_smooth_track(name, waypoints):
    """Reject a nominally smooth benchmark with artificial tangent steps."""
    quantized = [quantize_waypoint(waypoint) for waypoint in waypoints]
    headings = []
    for start, end in zip(quantized, quantized[1:]):
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        if math.hypot(delta_x, delta_y) == 0.0:
            raise ValueError(f'{name} contains a duplicate quantized waypoint')
        headings.append(math.atan2(delta_y, delta_x))

    initial_heading_error = abs(wrap_angle(headings[0]))
    maximum_heading_step = max(
        abs(wrap_angle(current - previous))
        for previous, current in zip(headings, headings[1:])
    )
    if initial_heading_error > 0.02 or maximum_heading_step > 0.025:
        raise ValueError(
            f'{name} is not sufficiently smooth after CSV quantization: '
            f'initial heading {initial_heading_error:.6f} rad, '
            f'maximum step {maximum_heading_step:.6f} rad'
        )


# Serialize an ordered waypoint list in the strict format expected by C++.
def save_to_csv(filename, waypoints):
    """Write an ordered waypoint sequence as two numeric CSV columns."""
    with open(filename, mode='w', newline='') as file:
        # Use repository-native LF endings on every platform. csv.writer's
        # default CRLF terminator is valid CSV but appears as trailing
        # whitespace to Git and creates noisy cross-platform diffs.
        writer = csv.writer(file, lineterminator='\n')
        for wp in waypoints:
            # Ten-micrometre precision prevents coordinate quantization from
            # recreating visible tangent steps on the 1 cm smooth-track grid.
            # Normalize tiny remnants so closed tracks end at 0.0, not -0.0.
            writer.writerow(quantize_waypoint(wp))
    print(f"Successfully generated: {filename} ({len(waypoints)} waypoints)")


# --- TRACK 1: Straight line / longitudinal baseline -------------------------
# A 20 m X-axis path isolates acceleration, steady tracking, and final stopping.
track_1 = []
for i in range(401):  # 0 to 400
    x = i * STRAIGHT_SPACING
    y = 0.0
    track_1.append((x, y))

# --- TRACK 2: Sinusoid / continuous-curvature tracking ----------------------
# y=1-cos(x) repeatedly changes steering direction without curvature jumps and
# has zero initial slope. The latter makes this a curvature-tracking benchmark,
# rather than accidentally combining it with the dedicated heading-capture test.
track_2 = []
curve_segment_count = round(5.0 / SMOOTH_TRACK_SPACING)
for i in range(curve_segment_count + 1):
    x = i * 5.0 / curve_segment_count
    y = 1.0 - math.cos(x)
    track_2.append((x, y))

# --- TRACK 3: Right-angle / discontinuous-curvature stress test -------------
# The ideal 90-degree corner is intentionally infeasible at nonzero speed. It
# tests braking, turn-in-place behavior, overshoot, and recovery after a corner.
track_3 = []
# Horizontal segment from the origin to (3, 0), including both endpoints.
for i in range(61):
    x = i * 0.05
    y = 0.0
    track_3.append((x, y))
# Vertical segment starts at y=0.05 to avoid duplicating the corner waypoint.
for i in range(1, 61):
    x = 3.0
    y = i * 0.05
    track_3.append((x, y))


# --- TRACK 4: Circle / sustained-curvature and closed-path test --------------
# A 1 m radius gives approximately the same 6 m path length as tracks 2 and 3.
# This parameterization starts at the origin with a +X tangent, matching the
# robot's spawn pose, and returns exactly to the origin after one revolution.
circle_radius = 1.0
circle_segments = round(
    2.0 * math.pi * circle_radius / SMOOTH_TRACK_SPACING
)
track_4 = []
for i in range(circle_segments + 1):
    angle = 2.0 * math.pi * i / circle_segments
    x = circle_radius * math.sin(angle)
    y = circle_radius * (1.0 - math.cos(angle))
    track_4.append((x, y))


# --- TRACK 5: Figure eight / crossing and steering-reversal test -------------
# Two radius-0.75 m circular lobes meet at the origin. Both lobes enter and
# leave the crossing with a +X tangent, so the path direction is continuous at
# the crossing even though curvature changes sign. The limited forward waypoint
# search must keep the controller on the correct branch at the shared point.
figure_eight_radius = 0.75
lobe_segments = round(
    2.0 * math.pi * figure_eight_radius / SMOOTH_TRACK_SPACING
)
track_5 = []

# Upper lobe: origin -> upper circle -> origin.
for i in range(lobe_segments + 1):
    angle = 2.0 * math.pi * i / lobe_segments
    x = figure_eight_radius * math.sin(angle)
    y = figure_eight_radius * (1.0 - math.cos(angle))
    track_5.append((x, y))


# Lower lobe: origin -> lower circle -> origin. Skip i=0 because the upper lobe
# already ended at the same crossing waypoint.
for i in range(1, lobe_segments + 1):
    angle = 2.0 * math.pi * i / lobe_segments
    x = figure_eight_radius * math.sin(angle)
    y = -figure_eight_radius * (1.0 - math.cos(angle))
    track_5.append((x, y))


# --- TRACK 6: Initial lateral offset / capture test -------------------------
# The robot spawns at (0, 0), while this line begins 0.2 m to its left. This
# measures convergence onto a path without changing the estimator's world/odom
# alignment or giving the controller a privileged initial pose correction.
track_6 = [(0.0, 0.2), (5.0, 0.2)]


# --- TRACK 7: Initial heading mismatch / capture test -----------------------
# The robot spawns facing +X while this 5 m line is directed at +45 degrees.
diagonal_endpoint = 5.0 / math.sqrt(2.0)
track_7 = [(0.0, 0.0), (diagonal_endpoint, diagonal_endpoint)]


# Avoid rewriting data merely by importing this module from another script.
if __name__ == '__main__':
    print("Generating High-Resolution Benchmark Tracks...")
    validate_smooth_track('track_2_curve', track_2)
    validate_smooth_track('track_4_circle', track_4)
    validate_smooth_track('track_5_figure_eight', track_5)
    save_to_csv('track_1_straight.csv', track_1)
    save_to_csv('track_2_curve.csv', track_2)
    save_to_csv('track_3_corner.csv', track_3)
    save_to_csv('track_4_circle.csv', track_4)
    save_to_csv('track_5_figure_eight.csv', track_5)
    save_to_csv('track_6_initial_lateral_offset.csv', track_6)
    save_to_csv('track_7_initial_heading_offset.csv', track_7)
