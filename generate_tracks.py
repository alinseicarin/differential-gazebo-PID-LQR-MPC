"""
Generate deterministic, headerless x,y paths for controller benchmarks.

Tracks use a nominal 5 cm waypoint spacing. They deliberately contain no header
because the C++ path loader expects exactly two numeric columns on every row.
Running this script replaces all seven track CSV files in the current directory.
"""

import math
import csv


def save_to_csv(filename, waypoints):
    """Write an ordered waypoint sequence as two numeric CSV columns."""
    with open(filename, mode='w', newline='') as file:
        # Use repository-native LF endings on every platform. csv.writer's
        # default CRLF terminator is valid CSV but appears as trailing
        # whitespace to Git and creates noisy cross-platform diffs.
        writer = csv.writer(file, lineterminator='\n')
        for wp in waypoints:
            # Millimetre precision is sufficient for the simulated robot while
            # keeping diffs and manual inspection manageable. Normalize tiny
            # floating-point remnants so closed tracks end at 0.0, not -0.0.
            rounded = [round(coordinate, 3) for coordinate in wp]
            cleaned = [0.0 if coordinate == 0.0 else coordinate for coordinate in rounded]
            writer.writerow(cleaned)
    print(f"Successfully generated: {filename} ({len(waypoints)} waypoints)")


# --- TRACK 1: Straight line / longitudinal baseline -------------------------
# A 20 m X-axis path isolates acceleration, steady tracking, and final stopping.
track_1 = []
for i in range(401):  # 0 to 400
    x = i * 0.05      # 5cm steps
    y = 0.0
    track_1.append((x, y))

# --- TRACK 2: Sinusoid / continuous-curvature tracking ----------------------
# y=sin(x) repeatedly changes steering direction without curvature jumps.
track_2 = []
for i in range(101):
    x = i * 0.05
    y = math.sin(x)  # Smooth curve
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
circle_segments = round(2.0 * math.pi * circle_radius / 0.05)
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
lobe_segments = round(2.0 * math.pi * figure_eight_radius / 0.05)
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
    save_to_csv('track_1_straight.csv', track_1)
    save_to_csv('track_2_curve.csv', track_2)
    save_to_csv('track_3_corner.csv', track_3)
    save_to_csv('track_4_circle.csv', track_4)
    save_to_csv('track_5_figure_eight.csv', track_5)
    save_to_csv('track_6_initial_lateral_offset.csv', track_6)
    save_to_csv('track_7_initial_heading_offset.csv', track_7)
