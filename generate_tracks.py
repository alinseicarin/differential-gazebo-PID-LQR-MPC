import math
import csv

def save_to_csv(filename, waypoints):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for wp in waypoints:
            # Rounding to 3 decimal places to keep the file clean
            writer.writerow([round(wp[0], 3), round(wp[1], 3)])
    print(f"Successfully generated: {filename} ({len(waypoints)} waypoints)")

# --- TRACK 1: The Straight Line (Step Response) ---
# Drives 5 meters straight forward on the X-axis
track_1 = []
for i in range(401):  # 0 to 400
    x = i * 0.05      # 5cm steps
    y = 0.0
    track_1.append((x, y))

# --- TRACK 2: The S-Curve (Continuous Tracking) ---
# Drives a smooth sine wave mimicking a lane change
track_2 = []
for i in range(101):
    x = i * 0.05
    y = math.sin(x)  # Smooth curve
    track_2.append((x, y))

# --- TRACK 3: The Sharp Corner (Obstacle Avoidance) ---
# Drives 3 meters forward, instantly turns 90 degrees left, and drives 3 meters up
track_3 = []
# Part A: Straight to X=3
for i in range(61):
    x = i * 0.05
    y = 0.0
    track_3.append((x, y))
# Part B: Straight up to Y=3
for i in range(1, 61):
    x = 3.0
    y = i * 0.05
    track_3.append((x, y))


# Generate the files
if __name__ == '__main__':
    print("Generating High-Resolution Benchmark Tracks...")
    save_to_csv('track_1_straight.csv', track_1)
    save_to_csv('track_2_curve.csv', track_2)
    save_to_csv('track_3_corner.csv', track_3)