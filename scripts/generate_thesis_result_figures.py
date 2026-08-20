#!/usr/bin/env python3
"""Generate thesis-ready vector and raster figures with Matplotlib.

The script reads only frozen experimental CSV files.  Each figure is exported
as vector PDF for the thesis and as a 300 dpi PNG for quick inspection.  A
manifest records the exact generator, inputs, and outputs used for the figures.
"""

import argparse
import csv
import hashlib
import math
from pathlib import Path

import matplotlib

# A non-interactive backend makes generation deterministic on headless hosts.
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import MultipleLocator  # noqa: E402


CONTROLLERS = ('pid', 'lqr', 'mpc')
LABELS = {'pid': 'PID', 'lqr': 'LQR', 'mpc': 'MPC'}
COLORS = {
    'pid': '#D55E00',
    'lqr': '#0072B2',
    'mpc': '#009E73',
}
REFERENCE_COLOR = '#202020'
FAULT_COLOR = '#C62828'


def configure_style():
    """Apply one compact, color-accessible style to every thesis figure."""
    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 9.5,
        'axes.titlesize': 11,
        'axes.labelsize': 10,
        'axes.edgecolor': '#303030',
        'axes.linewidth': 0.8,
        'axes.grid': True,
        'axes.axisbelow': True,
        'grid.color': '#D9DEE3',
        'grid.linewidth': 0.6,
        'grid.alpha': 0.85,
        'legend.fontsize': 9,
        'legend.frameon': True,
        'legend.framealpha': 0.95,
        'lines.linewidth': 1.6,
        'xtick.labelsize': 9,
        'ytick.labelsize': 9,
        # Preserve searchable text in vector outputs.
        'pdf.fonttype': 42,
        'ps.fonttype': 42,
        'svg.fonttype': 'none',
    })


def read_csv(path):
    """Load one CSV file as header-keyed dictionaries."""
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def numeric(row, key):
    """Parse a finite plotting value, returning NaN for invalid cells."""
    value = row.get(key, '')
    return float(value) if value not in ('', 'nan') else math.nan


def sha256(path):
    """Hash one input or output without loading the complete file in memory."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def output_paths(output_base):
    """Return the vector thesis file and matching raster preview path."""
    return output_base.with_suffix('.pdf'), output_base.with_suffix('.png')


def save_figure(figure, output_base):
    """Write a vector PDF and 300 dpi PNG, then release figure resources."""
    pdf_path, png_path = output_paths(output_base)
    figure.savefig(
        pdf_path, format='pdf', bbox_inches='tight',
        metadata={
            'Title': output_base.name,
            'Creator': 'generate_thesis_result_figures.py',
            'CreationDate': None,
            'ModDate': None,
        },
    )
    figure.savefig(
        png_path, format='png', dpi=300, bbox_inches='tight',
        metadata={'Software': 'generate_thesis_result_figures.py'},
    )
    plt.close(figure)
    return [pdf_path, png_path]


def ground_truth_paths(root, seed, scenario):
    """Resolve PID/LQR/MPC truth logs for one paired Gazebo realization."""
    run_directories = list(root.glob(f'run_*_seed_{seed}'))
    if len(run_directories) != 1:
        raise RuntimeError(f'expected one run directory for seed {seed}')
    run = run_directories[0]
    paths = {
        controller: run / controller / 'track_figure_eight' / scenario /
        'ground_truth.csv'
        for controller in CONTROLLERS
    }
    missing = [str(path) for path in paths.values() if not path.is_file()]
    if missing:
        raise RuntimeError('missing representative logs: ' + ', '.join(missing))
    return paths


def finite_pairs(rows, x_key, y_key, maximum_x=None, absolute_y=False):
    """Extract finite coordinate pairs, optionally cropped and rectified."""
    points = []
    for row in rows:
        x_value = numeric(row, x_key)
        y_value = numeric(row, y_key)
        if not math.isfinite(x_value) or not math.isfinite(y_value):
            continue
        if maximum_x is not None and x_value > maximum_x:
            continue
        points.append((x_value, abs(y_value) if absolute_y else y_value))
    return points


def nice_upper(maximum, step):
    """Round a positive plot limit upward to a stable readable tick step."""
    return max(step, math.ceil(maximum * 1.12 / step) * step)


def style_axes(axis, grid_axis='both'):
    """Remove decorative spines and retain a restrained engineering grid."""
    axis.spines['top'].set_visible(False)
    axis.spines['right'].set_visible(False)
    axis.grid(True, axis=grid_axis)


def trajectory_figure(root, output_base, seed):
    """Overlay the common reference and actual disturbed trajectories."""
    paths = ground_truth_paths(root, seed, 'left_wheel_loss_persistent')
    datasets = {controller: read_csv(path) for controller, path in paths.items()}
    reference = finite_pairs(datasets['pid'], 'reference_x', 'reference_y')

    figure, axis = plt.subplots(figsize=(6.7, 5.1), constrained_layout=True)
    axis.plot(
        [point[0] for point in reference],
        [point[1] for point in reference],
        color=REFERENCE_COLOR, linewidth=2.4, label='Referință', zorder=4,
    )
    for controller in CONTROLLERS:
        points = finite_pairs(datasets[controller], 'truth_x', 'truth_y')
        axis.plot(
            [point[0] for point in points],
            [point[1] for point in points],
            color=COLORS[controller], label=LABELS[controller], zorder=3,
        )
    axis.set_title(
        f'Pierdere permanentă a eficacității roții, realizarea {seed}'
    )
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_aspect('equal', adjustable='datalim')
    axis.legend(loc='upper right')
    style_axes(axis)
    outputs = save_figure(figure, output_base)
    return list(paths.values()), outputs


def plot_time_error(axis, datasets, value_key, title, y_label, tick_step,
                    absolute=False):
    """Draw one error signal for all controllers on a shared time horizon."""
    maximum = 0.0
    for controller in CONTROLLERS:
        points = finite_pairs(
            datasets[controller], 'time', value_key,
            maximum_x=30.0, absolute_y=absolute,
        )
        maximum = max(maximum, max(value for _, value in points))
        axis.plot(
            [point[0] for point in points],
            [point[1] for point in points],
            color=COLORS[controller], label=LABELS[controller],
        )
    axis.axvline(5.0, color=FAULT_COLOR, linestyle='--', linewidth=1.3)
    axis.text(
        5.18, 0.97, 'activarea defectului', color=FAULT_COLOR,
        transform=axis.get_xaxis_transform(), va='top', ha='left',
    )
    axis.set_title(title, loc='left')
    axis.set_ylabel(y_label)
    axis.set_xlim(0.0, 30.0)
    axis.set_ylim(0.0, nice_upper(maximum, tick_step))
    axis.yaxis.set_major_locator(MultipleLocator(tick_step))
    axis.legend(loc='upper right', ncol=3)
    style_axes(axis)


def combined_time_error_figure(root, output_base, seed):
    """Compare geometric CTE with distance to the timed reference."""
    paths = ground_truth_paths(root, seed, 'left_wheel_loss_persistent')
    datasets = {controller: read_csv(path) for controller, path in paths.items()}
    figure, axes = plt.subplots(
        2, 1, figsize=(7.0, 7.2), sharex=True, constrained_layout=True,
    )
    plot_time_error(
        axes[0], datasets, 'true_cross_track_error',
        '(a) Eroarea transversală geometrică', '|CTE| [m]',
        0.10, absolute=True,
    )
    plot_time_error(
        axes[1], datasets, 'true_position_error',
        '(b) Eroarea față de referința temporală', r'$e_p$ [m]', 0.25,
    )
    axes[1].set_xlabel('Timpul experimentului [s]')
    figure.suptitle(
        f'Defect permanent al roții stângi, realizarea {seed}',
        fontsize=12, fontweight='bold',
    )
    outputs = save_figure(figure, output_base)
    return list(paths.values()), outputs


def completion_figure(root, output_base):
    """Draw nominal and persistent-fault completion rates."""
    all_runs_path = root / 'all_runs.csv'
    rows = read_csv(all_runs_path)
    scenarios = ('nominal', 'left_wheel_loss_persistent')
    scenario_labels = ('Nominal', 'Pierdere permanentă\na eficacității roții')
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

    figure, axis = plt.subplots(figsize=(6.8, 4.3), constrained_layout=True)
    group_centers = (0.0, 1.0)
    offsets = (-0.23, 0.0, 0.23)
    width = 0.20
    for controller, offset in zip(CONTROLLERS, offsets):
        values = [rates[(scenario, controller)] for scenario in scenarios]
        bars = axis.bar(
            [center + offset for center in group_centers], values,
            width=width, color=COLORS[controller], edgecolor='#303030',
            linewidth=0.6, label=LABELS[controller],
        )
        axis.bar_label(bars, labels=[f'{value:.0f}%' for value in values],
                       padding=3, fontsize=9)
    axis.set_title('Finalizarea traiectoriei în validarea independentă')
    axis.set_ylabel('Finalizări [%]')
    axis.set_xticks(group_centers, scenario_labels)
    axis.set_ylim(0.0, 108.0)
    axis.yaxis.set_major_locator(MultipleLocator(20.0))
    axis.legend(loc='upper center', ncol=3)
    style_axes(axis, grid_axis='y')
    outputs = save_figure(figure, output_base)
    return [all_runs_path], outputs


def metric_values(rows, metric):
    """Group one persistent-fault metric in fixed controller order."""
    values = {}
    for controller in CONTROLLERS:
        items = [
            numeric(row, metric)
            for row in rows
            if row['controller_family'] == controller and
            row['scenario'] == 'left_wheel_loss_persistent'
        ]
        values[controller] = [value for value in items if math.isfinite(value)]
        if not values[controller]:
            raise RuntimeError(f'no finite {metric} values for {controller}')
    return values


def plot_metric_distribution(axis, rows, metric, title, y_label, tick_step,
                             mean_unit='m'):
    """Show all paired observations and a short segment at the group mean."""
    values = metric_values(rows, metric)
    maximum = max(max(items) for items in values.values())
    for controller_index, controller in enumerate(CONTROLLERS):
        items = values[controller]
        count = len(items)
        offsets = [
            0.0 if count == 1 else -0.16 + 0.32 * index / (count - 1)
            for index in range(count)
        ]
        axis.scatter(
            [controller_index + offset for offset in offsets], items,
            s=34, color=COLORS[controller], edgecolor='#303030',
            linewidth=0.55, zorder=3,
        )
        mean = sum(items) / count
        axis.hlines(
            mean, controller_index - 0.23, controller_index + 0.23,
            color='#202020', linewidth=2.0, zorder=4,
        )
    tick_labels = []
    for controller in CONTROLLERS:
        mean = sum(values[controller]) / len(values[controller])
        mean_text = f'{mean:.3f}'.replace('.', ',')
        tick_labels.append(
            f'{LABELS[controller]}\nmedia = {mean_text} {mean_unit}'
        )
    axis.set_xticks(range(len(CONTROLLERS)), tick_labels)
    axis.set_xlim(-0.55, len(CONTROLLERS) - 0.45)
    axis.set_ylim(0.0, nice_upper(maximum, tick_step))
    axis.yaxis.set_major_locator(MultipleLocator(tick_step))
    axis.set_title(title, loc='left')
    axis.set_ylabel(y_label)
    style_axes(axis, grid_axis='y')


def tracking_metric_distribution_figure(root, output_base):
    """Compare whole-window geometric and timed tracking across all runs."""
    all_runs_path = root / 'all_runs.csv'
    rows = read_csv(all_runs_path)
    figure, axes = plt.subplots(
        2, 1, figsize=(7.0, 7.2), constrained_layout=True,
    )
    plot_metric_distribution(
        axes[0], rows, 'cte_rmse_m',
        '(a) CTE RMS pe întreaga fereastră de 30 s', 'CTE RMS [m]', 0.05,
    )
    plot_metric_distribution(
        axes[1], rows, 'temporal_position_rmse_m',
        '(b) Eroarea temporală de poziție RMS pe 30 s',
        r'$e_{p,\mathrm{RMS}}$ [m]', 0.20,
    )
    figure.suptitle(
        'Indicatori continui sub defect permanent',
        fontsize=12, fontweight='bold',
    )
    outputs = save_figure(figure, output_base)
    return [all_runs_path], outputs


def persistent_tail_figure(root, output_base):
    """Plot the residual cross-track error over the final active window."""
    all_runs_path = root / 'all_runs.csv'
    rows = read_csv(all_runs_path)
    figure, axis = plt.subplots(figsize=(6.8, 4.3), constrained_layout=True)
    plot_metric_distribution(
        axis, rows, 'active_tail_mean_abs_cte_m',
        'Eroarea transversală persistentă în cele zece realizări',
        'Media |CTE| în coada activă [m]', 0.025,
    )
    outputs = save_figure(figure, output_base)
    return [all_runs_path], outputs


def active_cte_iae_figure(root, output_base):
    """Plot accumulated absolute CTE while the persistent fault is active."""
    all_runs_path = root / 'all_runs.csv'
    rows = read_csv(all_runs_path)
    figure, axis = plt.subplots(figsize=(6.8, 4.3), constrained_layout=True)
    plot_metric_distribution(
        axis, rows, 'active_cte_iae_m_s',
        'Eroarea transversală absolută acumulată sub defect permanent',
        'IAE CTE în fereastra activă [m s]', 1.0, mean_unit='m s',
    )
    outputs = save_figure(figure, output_base)
    return [all_runs_path], outputs


def write_manifest(output_directory, root, seed, inputs, outputs):
    """Record campaign, library version, and exact input/output hashes."""
    generator = Path(__file__).resolve()
    lines = [
        'THESIS RESULT FIGURE MANIFEST',
        f'confirmatory_directory={root.resolve()}',
        f'representative_seed={seed}',
        f'matplotlib_version={matplotlib.__version__}',
        f'generator_sha256={sha256(generator)}',
    ]
    for path in sorted(set(inputs)):
        lines.append(f'input_sha256[{path.resolve()}]={sha256(path)}')
    for path in outputs:
        lines.append(f'output_sha256[{path.name}]={sha256(path)}')
    (output_directory / 'figures_manifest.txt').write_text(
        '\n'.join(lines) + '\n', encoding='utf-8'
    )


def main():
    """Validate the frozen dataset and generate the selected thesis figures."""
    parser = argparse.ArgumentParser()
    parser.add_argument('confirmatory_directory', type=Path)
    parser.add_argument('output_directory', type=Path)
    parser.add_argument('--representative-seed', type=int, default=2509)
    arguments = parser.parse_args()

    root = arguments.confirmatory_directory
    output_directory = arguments.output_directory
    output_directory.mkdir(parents=True, exist_ok=True)
    audit_path = root / 'protocol_audit.txt'
    if not audit_path.is_file():
        raise RuntimeError('confirmatory directory lacks its protocol audit')
    audit = audit_path.read_text(encoding='utf-8')
    if 'issues=0' not in audit:
        raise RuntimeError('refusing to plot a dataset that failed its audit')

    configure_style()
    specifications = (
        ('traiectorii_pierdere_roata', trajectory_figure, True),
        ('erori_timp_pierdere_roata', combined_time_error_figure, True),
        ('finalizare_confirmatorie', completion_figure, False),
        ('metrici_urmarire_confirmatorie',
         tracking_metric_distribution_figure, False),
        ('iae_cte_confirmatorie', active_cte_iae_figure, False),
        ('cte_coada_confirmatorie', persistent_tail_figure, False),
    )
    inputs = [audit_path, root / 'protocol.txt']
    outputs = []
    for name, generator, needs_seed in specifications:
        output_base = output_directory / name
        if needs_seed:
            figure_inputs, figure_outputs = generator(
                root, output_base, arguments.representative_seed
            )
        else:
            figure_inputs, figure_outputs = generator(root, output_base)
        inputs.extend(figure_inputs)
        outputs.extend(figure_outputs)
        for output in figure_outputs:
            print(f'Wrote {output}')
    write_manifest(
        output_directory, root, arguments.representative_seed,
        inputs, outputs,
    )
    print(f'Wrote {output_directory / "figures_manifest.txt"}')


if __name__ == '__main__':
    main()
