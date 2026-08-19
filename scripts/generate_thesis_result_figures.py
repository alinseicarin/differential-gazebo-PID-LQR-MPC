#!/usr/bin/env python3
"""Generate thesis-ready PNG figures from the frozen comparison dataset.

The implementation deliberately depends only on the Python standard library
and Pillow.  It never alters the experimental CSV files.  Every generated set
also receives a manifest containing the hashes of the inputs and outputs used
for the thesis figures.
"""

import argparse
import csv
import hashlib
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


CONTROLLERS = ('pid', 'lqr', 'mpc')
LABELS = {'pid': 'PID', 'lqr': 'LQR', 'mpc': 'MPC'}
COLORS = {
    'pid': (213, 94, 0),
    'lqr': (0, 114, 178),
    'mpc': (0, 145, 90),
}
REFERENCE_COLOR = (35, 35, 35)
GRID_COLOR = (220, 224, 228)
TEXT_COLOR = (25, 25, 25)
FONT_PATH = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
FONT_BOLD_PATH = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')


# Load a consistent Pillow font object for every final figure.
def font(size, bold=False):
    """Load a Unicode font available in the WSL and container images."""
    path = FONT_BOLD_PATH if bold else FONT_PATH
    return ImageFont.truetype(str(path), size)


# Load one source CSV as header-keyed dictionaries.
def read_csv(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


# Parse a finite plotting value, returning NaN for invalid cells.
def numeric(row, key):
    value = row.get(key, '')
    return float(value) if value not in ('', 'nan') else math.nan


# Hash each campaign file used to build a thesis figure.
def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


# Generate evenly spaced ticks and widen a degenerate numeric range.
def tick_values(minimum, maximum, count=5):
    """Return readable, evenly spaced ticks including both plot limits."""
    if maximum <= minimum:
        return [minimum]
    raw = (maximum - minimum) / count
    exponent = 10.0 ** math.floor(math.log10(raw))
    normalized = raw / exponent
    step = (1.0 if normalized <= 1.0 else
            2.0 if normalized <= 2.0 else
            5.0 if normalized <= 5.0 else 10.0) * exponent
    first = math.ceil(minimum / step) * step
    values = []
    value = first
    while value <= maximum + 1.0e-9 * step:
        values.append(value)
        value += step
    return values


# Small Pillow-backed raster plotting surface that maps data coordinates to
# pixels and exposes only the primitives required by the thesis.
class Plot:
    """Small dependency-free plotting surface backed by Pillow."""

    # Initialize canvas, margins, title, and axis-label text.
    def __init__(self, title, x_label, y_label, width=1800, height=1100):
        self.image = Image.new('RGB', (width, height), 'white')
        self.draw = ImageDraw.Draw(self.image)
        self.width = width
        self.height = height
        self.left, self.right = 185, width - 65
        self.top, self.bottom = 115, height - 145
        self.draw.text(
            (width / 2, 38), title, anchor='ma',
            fill=TEXT_COLOR, font=font(42, bold=True),
        )
        self.x_label = x_label
        self.y_label = y_label

    # Define numeric bounds, optional equal aspect, grid, ticks, and labels.
    def axes(self, x_min, x_max, y_min, y_max, equal=False):
        if equal:
            plot_ratio = ((self.right - self.left) /
                          (self.bottom - self.top))
            data_ratio = (x_max - x_min) / max(y_max - y_min, 1.0e-12)
            if data_ratio > plot_ratio:
                wanted_y = (x_max - x_min) / plot_ratio
                extra = wanted_y - (y_max - y_min)
                y_min -= extra / 2.0
                y_max += extra / 2.0
            else:
                wanted_x = (y_max - y_min) * plot_ratio
                extra = wanted_x - (x_max - x_min)
                x_min -= extra / 2.0
                x_max += extra / 2.0
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max

        for value in tick_values(x_min, x_max):
            x = self.x(value)
            self.draw.line(
                (x, self.top, x, self.bottom), fill=GRID_COLOR, width=2
            )
            self.draw.text(
                (x, self.bottom + 18), f'{value:g}', anchor='ma',
                fill=TEXT_COLOR, font=font(25),
            )
        for value in tick_values(y_min, y_max):
            y = self.y(value)
            self.draw.line(
                (self.left, y, self.right, y), fill=GRID_COLOR, width=2
            )
            self.draw.text(
                (self.left - 18, y), f'{value:g}', anchor='rm',
                fill=TEXT_COLOR, font=font(25),
            )
        self.draw.line(
            (self.left, self.top, self.left, self.bottom),
            fill=TEXT_COLOR, width=3,
        )
        self.draw.line(
            (self.left, self.bottom, self.right, self.bottom),
            fill=TEXT_COLOR, width=3,
        )
        self.draw.text(
            ((self.left + self.right) / 2, self.height - 38),
            self.x_label, anchor='ms', fill=TEXT_COLOR, font=font(30),
        )
        label = Image.new('RGBA', (600, 60), (255, 255, 255, 0))
        label_draw = ImageDraw.Draw(label)
        label_draw.text(
            (300, 30), self.y_label, anchor='mm',
            fill=TEXT_COLOR, font=font(30),
        )
        label = label.rotate(90, expand=True)
        self.image.paste(
            label, (25, int((self.top + self.bottom - label.height) / 2)),
            label,
        )

    # Map one data x coordinate to horizontal pixels.
    def x(self, value):
        fraction = (value - self.x_min) / (self.x_max - self.x_min)
        return self.left + fraction * (self.right - self.left)

    # Map one data y coordinate to the inverted image pixel axis.
    def y(self, value):
        fraction = (value - self.y_min) / (self.y_max - self.y_min)
        return self.bottom - fraction * (self.bottom - self.top)

    # Draw a continuous colored series through data-coordinate points.
    def polyline(self, points, color, width=5):
        pixels = [(self.x(x), self.y(y)) for x, y in points]
        if len(pixels) >= 2:
            self.draw.line(pixels, fill=color, width=width, joint='curve')

    # Mark an event time, such as fault activation, with a dashed vertical line.
    def dashed_vertical(self, x_value, color, width=4, dash=18):
        x = self.x(x_value)
        y = self.top
        while y < self.bottom:
            self.draw.line(
                (x, y, x, min(y + dash, self.bottom)),
                fill=color, width=width,
            )
            y += 2 * dash

    # Draw a compact vertical legend in unused plot space.
    def legend(self, entries):
        x = self.right - 305
        y = self.top + 28
        row_height = 42
        box = (x - 25, y - 20, self.right - 15,
               y + row_height * len(entries) + 8)
        self.draw.rounded_rectangle(
            box, radius=12, fill=(255, 255, 255),
            outline=(150, 150, 150), width=2,
        )
        for index, (label, color) in enumerate(entries):
            row_y = y + index * row_height
            self.draw.line(
                (x, row_y, x + 58, row_y), fill=color, width=7
            )
            self.draw.text(
                (x + 75, row_y), label, anchor='lm',
                fill=TEXT_COLOR, font=font(25),
            )

    # Draw a single-row legend at a caller-selected vertical coordinate.
    def horizontal_legend(self, entries, y):
        """Draw one legend row outside the data rectangle."""
        entry_width = 230
        total_width = entry_width * len(entries)
        x0 = (self.left + self.right - total_width) / 2
        box = (x0 - 25, y - 25, x0 + total_width + 5, y + 27)
        self.draw.rounded_rectangle(
            box, radius=12, fill=(255, 255, 255),
            outline=(150, 150, 150), width=2,
        )
        for index, (label, color) in enumerate(entries):
            x = x0 + index * entry_width
            self.draw.line((x, y, x + 58, y), fill=color, width=7)
            self.draw.text(
                (x + 75, y), label, anchor='lm',
                fill=TEXT_COLOR, font=font(25),
            )

    # Encode the finished image according to the output filename extension.
    def save(self, path):
        self.image.save(path, dpi=(180, 180), optimize=True)


# Resolve PID/LQR/MPC truth logs for one seed and scenario.
def ground_truth_paths(root, seed, scenario):
    run_directories = list(root.glob(f'run_*_seed_{seed}'))
    if len(run_directories) != 1:
        raise RuntimeError(f'expected one run directory for seed {seed}')
    run = run_directories[0]
    return {
        controller: run / controller / 'track_figure_eight' / scenario /
        'ground_truth.csv'
        for controller in CONTROLLERS
    }


# Extract valid coordinate pairs, optionally cropped to a common horizon.
def finite_pairs(rows, x_key, y_key, maximum_time=None):
    points = []
    for row in rows:
        x = numeric(row, x_key)
        y = numeric(row, y_key)
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if maximum_time is not None and x > maximum_time:
            continue
        points.append((x, y))
    return points


# Add visual margin around a finite data interval.
def padded_bounds(values, fraction=0.06):
    minimum, maximum = min(values), max(values)
    span = maximum - minimum
    padding = max(span * fraction, 0.02)
    return minimum - padding, maximum + padding


# Overlay the timed reference and actual figure-eight paths for all methods.
def trajectory_figure(root, output, seed):
    paths = ground_truth_paths(root, seed, 'left_wheel_loss_persistent')
    datasets = {controller: read_csv(path) for controller, path in paths.items()}
    reference = finite_pairs(datasets['pid'], 'reference_x', 'reference_y')
    trajectories = {
        controller: finite_pairs(rows, 'truth_x', 'truth_y')
        for controller, rows in datasets.items()
    }
    all_x = [x for x, _ in reference]
    all_y = [y for _, y in reference]
    for points in trajectories.values():
        all_x.extend(x for x, _ in points)
        all_y.extend(y for _, y in points)
    x_min, x_max = padded_bounds(all_x)
    y_min, y_max = padded_bounds(all_y)
    plot = Plot(
        f'Traiectorii cu pierdere permanentă a roții, realizarea {seed}',
        'x [m]', 'y [m]',
    )
    plot.axes(x_min, x_max, y_min, y_max, equal=True)
    plot.polyline(reference, REFERENCE_COLOR, width=9)
    for controller in CONTROLLERS:
        plot.polyline(trajectories[controller], COLORS[controller], width=5)
    plot.legend([
        ('Referință', REFERENCE_COLOR),
        *((LABELS[c], COLORS[c]) for c in CONTROLLERS),
    ])
    plot.save(output)
    return list(paths.values())


# Plot cross-track error versus time with unobstructed labels.
def cte_time_figure(root, output, seed):
    paths = ground_truth_paths(root, seed, 'left_wheel_loss_persistent')
    datasets = {controller: read_csv(path) for controller, path in paths.items()}
    series = {}
    maximum = 0.0
    for controller, rows in datasets.items():
        points = finite_pairs(rows, 'time', 'true_cross_track_error', 30.0)
        points = [(time, abs(value)) for time, value in points]
        series[controller] = points
        maximum = max(maximum, max(value for _, value in points))
    y_max = math.ceil(maximum * 1.12 / 0.05) * 0.05
    plot = Plot(
        f'Eroarea transversală sub defect permanent, realizarea {seed}',
        'Timpul experimentului [s]', '|CTE| [m]',
    )
    plot.axes(0.0, 30.0, 0.0, y_max)
    plot.dashed_vertical(5.0, (180, 30, 30))
    plot.draw.text(
        (plot.x(5.0) + 12, plot.top + 15), 'activarea defectului',
        anchor='la', fill=(150, 20, 20), font=font(24),
    )
    for controller in CONTROLLERS:
        plot.polyline(series[controller], COLORS[controller], width=5)
    plot.legend([(LABELS[c], COLORS[c]) for c in CONTROLLERS])
    plot.save(output)
    return list(paths.values())


# Draw completion counts with the legend positioned away from bars.
def completion_figure(root, output):
    all_runs_path = root / 'all_runs.csv'
    rows = read_csv(all_runs_path)
    scenarios = ('nominal', 'left_wheel_loss_persistent')
    scenario_labels = ('Nominal', 'Pierdere permanentă roată')
    rates = {}
    for scenario in scenarios:
        for controller in CONTROLLERS:
            subset = [
                row for row in rows
                if row['scenario'] == scenario and
                row['controller_family'] == controller
            ]
            rates[(scenario, controller)] = (
                100.0 * sum(int(row['track_complete']) for row in subset) /
                len(subset)
            )
    plot = Plot(
        'Rata de finalizare în validarea independentă',
        'Condiția experimentală', 'Finalizări [%]',
    )
    # Reserve a dedicated strip for the legend so it cannot cover a bar.
    plot.top = 205
    plot.axes(-0.6, 1.6, 0.0, 100.0)
    group_centers = (0.0, 1.0)
    bar_width = 0.18
    offsets = (-0.22, 0.0, 0.22)
    for group_index, scenario in enumerate(scenarios):
        center = group_centers[group_index]
        for controller, offset in zip(CONTROLLERS, offsets):
            rate = rates[(scenario, controller)]
            x0 = plot.x(center + offset - bar_width / 2)
            x1 = plot.x(center + offset + bar_width / 2)
            y0 = plot.y(0.0)
            y1 = plot.y(rate)
            plot.draw.rectangle(
                (x0, y1, x1, y0), fill=COLORS[controller],
                outline=(50, 50, 50), width=2,
            )
            plot.draw.text(
                ((x0 + x1) / 2, y1 - 10), f'{rate:.0f}%',
                anchor='mb', fill=TEXT_COLOR, font=font(25, bold=True),
            )
        plot.draw.text(
            (plot.x(center), plot.bottom + 22), scenario_labels[group_index],
            anchor='ma', fill=TEXT_COLOR, font=font(24),
        )
    # Cover the numeric x ticks because this axis is categorical.
    plot.draw.rectangle(
        (plot.left - 5, plot.bottom + 5, plot.right + 5, plot.bottom + 55),
        fill='white',
    )
    plot.draw.line(
        (plot.left, plot.bottom, plot.right, plot.bottom),
        fill=TEXT_COLOR, width=3,
    )
    for group_index, label in enumerate(scenario_labels):
        plot.draw.text(
            (plot.x(group_centers[group_index]), plot.bottom + 22), label,
            anchor='ma', fill=TEXT_COLOR, font=font(24),
        )
    plot.horizontal_legend(
        [(LABELS[c], COLORS[c]) for c in CONTROLLERS], y=135
    )
    plot.save(output)
    return [all_runs_path]


# Compare residual errors after permanent wheel-performance loss.
def persistent_tail_figure(root, output):
    all_runs_path = root / 'all_runs.csv'
    rows = read_csv(all_runs_path)
    values = {}
    for controller in CONTROLLERS:
        values[controller] = [
            numeric(row, 'active_tail_mean_abs_cte_m')
            for row in rows
            if row['controller_family'] == controller and
            row['scenario'] == 'left_wheel_loss_persistent'
        ]
    maximum = max(max(items) for items in values.values())
    y_max = math.ceil(maximum * 1.15 / 0.025) * 0.025
    plot = Plot(
        'Eroarea transversală persistentă în cele zece realizări',
        'Regulatorul', 'Media |CTE| în coada activă [m]',
    )
    # Leave two clean text rows below the axis: category, then group mean.
    plot.bottom = 885
    plot.axes(-0.55, 2.55, 0.0, y_max)
    for controller_index, controller in enumerate(CONTROLLERS):
        items = values[controller]
        for index, value in enumerate(items):
            jitter = ((index % 5) - 2) * 0.035
            x = plot.x(controller_index + jitter)
            y = plot.y(value)
            radius = 9
            plot.draw.ellipse(
                (x - radius, y - radius, x + radius, y + radius),
                fill=COLORS[controller], outline=(40, 40, 40), width=2,
            )
        mean = sum(items) / len(items)
        plot.draw.line(
            (plot.x(controller_index - 0.22), plot.y(mean),
             plot.x(controller_index + 0.22), plot.y(mean)),
            fill=(20, 20, 20), width=7,
        )
    plot.draw.rectangle(
        (plot.left - 5, plot.bottom + 5, plot.right + 5, plot.bottom + 92),
        fill='white',
    )
    plot.draw.line(
        (plot.left, plot.bottom, plot.right, plot.bottom),
        fill=TEXT_COLOR, width=3,
    )
    for index, controller in enumerate(CONTROLLERS):
        plot.draw.text(
            (plot.x(index), plot.bottom + 22), LABELS[controller],
            anchor='ma', fill=TEXT_COLOR, font=font(27),
        )
        mean = sum(values[controller]) / len(values[controller])
        mean_label = f'medie = {mean:.3f} m'.replace('.', ',')
        plot.draw.text(
            (plot.x(index), plot.bottom + 58), mean_label,
            anchor='ma', fill=TEXT_COLOR, font=font(23, bold=True),
        )
    plot.save(output)
    return [all_runs_path]


# Record campaign path, representative seed, and input/output hashes.
def write_manifest(output_directory, root, seed, inputs, outputs):
    generator = Path(__file__).resolve()
    lines = [
        'THESIS RESULT FIGURE MANIFEST',
        f'confirmatory_directory={root.resolve()}',
        f'representative_seed={seed}',
        f'generator_sha256={sha256(generator)}',
    ]
    for path in sorted(set(inputs)):
        lines.append(f'input_sha256[{path.resolve()}]={sha256(path)}')
    for path in outputs:
        lines.append(f'output_sha256[{path.name}]={sha256(path)}')
    (output_directory / 'figures_manifest.txt').write_text(
        '\n'.join(lines) + '\n', encoding='utf-8'
    )


# Generate the selected high-resolution thesis figures and provenance manifest.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('confirmatory_directory', type=Path)
    parser.add_argument('output_directory', type=Path)
    parser.add_argument('--representative-seed', type=int, default=2509)
    arguments = parser.parse_args()

    root = arguments.confirmatory_directory
    output_directory = arguments.output_directory
    output_directory.mkdir(parents=True, exist_ok=True)
    if not (root / 'protocol_audit.txt').is_file():
        raise RuntimeError('confirmatory directory lacks its protocol audit')
    audit = (root / 'protocol_audit.txt').read_text(encoding='utf-8')
    if 'issues=0' not in audit:
        raise RuntimeError('refusing to plot a dataset that failed its audit')

    specifications = (
        ('traiectorii_pierdere_roata.png', trajectory_figure),
        ('cte_pierdere_roata.png', cte_time_figure),
        ('finalizare_confirmatorie.png', completion_figure),
        ('cte_coada_confirmatorie.png', persistent_tail_figure),
    )
    inputs = [root / 'protocol_audit.txt', root / 'protocol.txt']
    outputs = []
    for name, generator in specifications:
        output = output_directory / name
        if generator in (trajectory_figure, cte_time_figure):
            inputs.extend(generator(
                root, output, arguments.representative_seed
            ))
        else:
            inputs.extend(generator(root, output))
        outputs.append(output)
        print(f'Wrote {output}')
    write_manifest(
        output_directory, root, arguments.representative_seed,
        inputs, outputs,
    )
    print(f'Wrote {output_directory / "figures_manifest.txt"}')


if __name__ == '__main__':
    # Make plotting helpers importable without writing files automatically.
    main()
