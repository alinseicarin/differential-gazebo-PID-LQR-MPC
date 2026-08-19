#!/usr/bin/env python3
"""Generate dependency-free thesis tables and SVG figures from comparison CSVs."""

import csv
import hashlib
import html
import math
from pathlib import Path
import sys


CONTROLLERS = ('pid', 'lqr', 'mpc')
CONTROLLER_LABELS = {'pid': 'PID', 'lqr': 'TVLQR', 'mpc': 'MPC'}
CONTROLLER_COLORS = {
    'pid': '#0072B2',
    'lqr': '#D55E00',
    'mpc': '#009E73',
}

TRACK_LABELS = {
    'straight': 'Dreapta',
    'curve': 'Curba',
    'circle': 'Cerc',
    'figure_eight': 'Figura opt',
}

SCENARIO_LABELS = {
    'nominal': 'Nominal',
    'angular_pulse_train': 'Impulsuri unghiulare',
    'angular_constant': 'Bias unghiular',
    'left_wheel_loss': 'Pierdere roata temporara',
    'left_wheel_loss_persistent': 'Pierdere roata permanenta',
    'command_delay': 'Intarziere comanda',
    'localization_noise': 'Zgomot localizare',
    'localization_yaw_bias': 'Bias orientare',
}

FINITE_SCENARIOS = (
    'angular_pulse_train',
    'angular_constant',
    'left_wheel_loss',
    'command_delay',
    'localization_noise',
    'localization_yaw_bias',
)


# Load a headered analysis CSV into dictionaries.
def read_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


# Safely parse one finite numeric field, returning NaN for missing data.
def number(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


# Format numbers compactly and represent invalid values consistently.
def format_number(value):
    return '' if not math.isfinite(value) else f'{value:.9f}'


# Digest one artifact for the report provenance manifest.
def sha256_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            digest.update(block)
    return digest.hexdigest()


# Hash an ordered bundle including relative names and file contents.
def sha256_bundle(root, paths):
    """Hash relative names and bytes so input additions also change the digest."""
    digest = hashlib.sha256()
    for path in sorted(paths):
        relative = path.relative_to(root).as_posix().encode('utf-8')
        digest.update(len(relative).to_bytes(8, 'big'))
        digest.update(relative)
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(65536), b''):
                digest.update(block)
    return digest.hexdigest()


# Index descriptive rows by controller, track, and scenario.
def group_index(rows):
    indexed = {}
    for row in rows:
        key = (row['controller_family'], row['track'], row['scenario'])
        if key in indexed:
            raise RuntimeError(f'duplicate grouped condition: {key}')
        indexed[key] = row
    return indexed


# Return mean and confidence limits stored under one metric prefix.
def metric_triplet(row, metric):
    return (
        number(row, f'{metric}_mean'),
        number(row, f'{metric}_ci95_low'),
        number(row, f'{metric}_ci95_high'),
    )


# Choose a readable 1/2/5*10^n axis tick interval.
def nice_step(span, target_ticks=6):
    if not math.isfinite(span) or span <= 0.0:
        return 1.0
    raw = span / target_ticks
    power = 10.0 ** math.floor(math.log10(raw))
    fraction = raw / power
    if fraction <= 1.0:
        nice = 1.0
    elif fraction <= 2.0:
        nice = 2.0
    elif fraction <= 5.0:
        nice = 5.0
    else:
        nice = 10.0
    return nice * power


# Render a dependency-free SVG bar chart with intervals and labels.
def svg_bar_chart(path, title, y_label, categories, values, force_zero=True):
    """Write a grouped mean-and-95%-CI bar chart as plain SVG."""
    category_width = 150
    width = max(820, 155 + category_width * len(categories))
    height = 590
    left, right, top, bottom = 105, 35, 82, 155
    plot_width = width - left - right
    plot_height = height - top - bottom

    finite_bounds = []
    for controller in CONTROLLERS:
        for mean, low, high in values[controller]:
            finite_bounds.extend(
                value for value in (mean, low, high) if math.isfinite(value)
            )
    if not finite_bounds:
        raise RuntimeError(f'no finite values for figure {path.name}')
    minimum = min(finite_bounds)
    maximum = max(finite_bounds)
    if force_zero:
        minimum = min(0.0, minimum)
        maximum = max(0.0, maximum)
    span = maximum - minimum
    padding = 0.08 * span if span > 0.0 else max(0.01, abs(maximum) * 0.1)
    minimum -= padding
    maximum += padding
    if force_zero and min(finite_bounds) >= 0.0:
        minimum = 0.0

    step = nice_step(maximum - minimum)
    axis_minimum = math.floor(minimum / step) * step
    axis_maximum = math.ceil(maximum / step) * step
    if force_zero and min(finite_bounds) >= 0.0:
        axis_minimum = 0.0
    if axis_maximum <= axis_minimum:
        axis_maximum = axis_minimum + step

    # Place one controller bar within its grouped categorical slot.
    def x_position(category_index, controller_index):
        group_center = left + (category_index + 0.5) * plot_width / len(categories)
        bar_width = min(30.0, 0.18 * plot_width / len(categories))
        separation = bar_width + 5.0
        return group_center + (controller_index - 1) * separation, bar_width

    # Convert a bar value into SVG's downward-increasing y coordinate.
    def y_position(value):
        fraction = (value - axis_minimum) / (axis_maximum - axis_minimum)
        return top + plot_height * (1.0 - fraction)

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#222}'
        '.axis{stroke:#333;stroke-width:1.4}.grid{stroke:#ddd;stroke-width:1}'
        '.error{stroke:#222;stroke-width:1.4}</style>',
        f'<text x="{width / 2:.1f}" y="32" text-anchor="middle" '
        f'font-size="20" font-weight="bold">{html.escape(title)}</text>',
    ]

    tick = math.ceil(axis_minimum / step) * step
    while tick <= axis_maximum + step * 1.0e-9:
        y = y_position(tick)
        svg.append(
            f'<line class="grid" x1="{left}" y1="{y:.2f}" '
            f'x2="{width - right}" y2="{y:.2f}"/>'
        )
        svg.append(
            f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end" '
            f'font-size="12">{tick:.3g}</text>'
        )
        tick += step

    zero_y = y_position(0.0) if axis_minimum <= 0.0 <= axis_maximum else None
    if zero_y is not None:
        svg.append(
            f'<line class="axis" x1="{left}" y1="{zero_y:.2f}" '
            f'x2="{width - right}" y2="{zero_y:.2f}"/>'
        )
    svg.append(
        f'<line class="axis" x1="{left}" y1="{top}" '
        f'x2="{left}" y2="{top + plot_height}"/>'
    )

    for category_index, category in enumerate(categories):
        center = left + (category_index + 0.5) * plot_width / len(categories)
        svg.append(
            f'<text x="{center:.2f}" y="{top + plot_height + 25}" '
            f'text-anchor="end" font-size="12" '
            f'transform="rotate(-32 {center:.2f} {top + plot_height + 25})">'
            f'{html.escape(category)}</text>'
        )
        for controller_index, controller in enumerate(CONTROLLERS):
            mean, low, high = values[controller][category_index]
            if not math.isfinite(mean):
                continue
            center_x, bar_width = x_position(category_index, controller_index)
            base_value = 0.0 if zero_y is not None else axis_minimum
            y_mean = y_position(mean)
            y_base = y_position(base_value)
            rectangle_y = min(y_mean, y_base)
            rectangle_height = max(1.0, abs(y_base - y_mean))
            svg.append(
                f'<rect x="{center_x - bar_width / 2:.2f}" y="{rectangle_y:.2f}" '
                f'width="{bar_width:.2f}" height="{rectangle_height:.2f}" '
                f'fill="{CONTROLLER_COLORS[controller]}" opacity="0.88"/>'
            )
            if math.isfinite(low) and math.isfinite(high):
                y_low, y_high = y_position(low), y_position(high)
                svg.extend((
                    f'<line class="error" x1="{center_x:.2f}" y1="{y_low:.2f}" '
                    f'x2="{center_x:.2f}" y2="{y_high:.2f}"/>',
                    f'<line class="error" x1="{center_x - 5:.2f}" y1="{y_low:.2f}" '
                    f'x2="{center_x + 5:.2f}" y2="{y_low:.2f}"/>',
                    f'<line class="error" x1="{center_x - 5:.2f}" y1="{y_high:.2f}" '
                    f'x2="{center_x + 5:.2f}" y2="{y_high:.2f}"/>',
                ))

    svg.append(
        f'<text x="24" y="{top + plot_height / 2:.2f}" text-anchor="middle" '
        f'font-size="13" transform="rotate(-90 24 {top + plot_height / 2:.2f})">'
        f'{html.escape(y_label)}</text>'
    )
    legend_x = width - right - 235
    for index, controller in enumerate(CONTROLLERS):
        x = legend_x + index * 80
        svg.append(
            f'<rect x="{x}" y="50" width="14" height="14" '
            f'fill="{CONTROLLER_COLORS[controller]}"/>'
        )
        svg.append(
            f'<text x="{x + 20}" y="62" font-size="12">'
            f'{CONTROLLER_LABELS[controller]}</text>'
        )
    svg.append('</svg>')
    path.write_text('\n'.join(svg) + '\n', encoding='utf-8')


# Suppress meaningless plots when every selected value is missing.
def grouped_chart_has_finite_value(values):
    """Return whether at least one controller/category mean can be plotted."""
    return any(
        math.isfinite(mean)
        for controller in CONTROLLERS
        for mean, _low, _high in values[controller]
    )


# Check whether paired observations exist for requested plot conditions.
def paired_chart_has_finite_value(conditions, metric, raw_rows):
    """Return whether a requested paired-data figure has any observation."""
    requested = set(conditions)
    return any(
        (row['track'], row['scenario']) in requested
        and math.isfinite(number(row, metric))
        for row in raw_rows
    )


# Draw seed-matched controller observations as connected dots so pairing and
# between-seed variation remain visible.
def svg_paired_dot_chart(
        path, title, y_label, category_labels, conditions, metric, raw_rows):
    """Show every paired seed with gray links across the three controllers."""
    indexed = {}
    seeds = set()
    for row in raw_rows:
        key = (
            row['track'], row['scenario'], row['gazebo_seed'],
            row['controller_family'],
        )
        indexed[key] = number(row, metric)
        seeds.add(row['gazebo_seed'])
    ordered_seeds = sorted(seeds, key=int)

    finite_values = []
    for track, scenario in conditions:
        for seed in ordered_seeds:
            for controller in CONTROLLERS:
                value = indexed.get((track, scenario, seed, controller), math.nan)
                if math.isfinite(value):
                    finite_values.append(value)
    if not finite_values:
        raise RuntimeError(f'no paired values for figure {path.name}')

    category_width = 175
    width = max(820, 150 + category_width * len(conditions))
    height = 580
    left, right, top, bottom = 105, 35, 82, 145
    plot_width = width - left - right
    plot_height = height - top - bottom
    minimum = min(0.0, min(finite_values))
    maximum = max(finite_values)
    padding = 0.08 * (maximum - minimum) if maximum > minimum else 0.01
    axis_minimum = 0.0 if minimum >= 0.0 else minimum - padding
    axis_maximum = maximum + padding
    step = nice_step(axis_maximum - axis_minimum)
    axis_minimum = math.floor(axis_minimum / step) * step
    axis_maximum = math.ceil(axis_maximum / step) * step

    # Convert a paired observation into SVG's downward-increasing y coordinate.
    def y_position(value):
        fraction = (value - axis_minimum) / (axis_maximum - axis_minimum)
        return top + plot_height * (1.0 - fraction)

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#222}'
        '.axis{stroke:#333;stroke-width:1.4}.grid{stroke:#ddd;stroke-width:1}'
        '.pair{stroke:#999;stroke-width:1;fill:none;opacity:.6}</style>',
        f'<text x="{width / 2:.1f}" y="32" text-anchor="middle" '
        f'font-size="20" font-weight="bold">{html.escape(title)}</text>',
    ]
    tick = math.ceil(axis_minimum / step) * step
    while tick <= axis_maximum + step * 1.0e-9:
        y = y_position(tick)
        svg.append(
            f'<line class="grid" x1="{left}" y1="{y:.2f}" '
            f'x2="{width - right}" y2="{y:.2f}"/>'
        )
        svg.append(
            f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end" '
            f'font-size="12">{tick:.3g}</text>'
        )
        tick += step
    svg.append(
        f'<line class="axis" x1="{left}" y1="{top}" '
        f'x2="{left}" y2="{top + plot_height}"/>'
    )
    svg.append(
        f'<line class="axis" x1="{left}" y1="{top + plot_height}" '
        f'x2="{width - right}" y2="{top + plot_height}"/>'
    )

    for category_index, ((track, scenario), label) in enumerate(
            zip(conditions, category_labels)):
        center = left + (category_index + 0.5) * plot_width / len(conditions)
        controller_x = {
            controller: center + (index - 1) * 34.0
            for index, controller in enumerate(CONTROLLERS)
        }
        for seed_index, seed in enumerate(ordered_seeds):
            jitter = (seed_index - (len(ordered_seeds) - 1) / 2.0) * 0.9
            points = []
            for controller in CONTROLLERS:
                value = indexed.get((track, scenario, seed, controller), math.nan)
                if math.isfinite(value):
                    points.append((controller_x[controller] + jitter, y_position(value)))
            if len(points) >= 2:
                coordinates = ' '.join(f'{x:.2f},{y:.2f}' for x, y in points)
                svg.append(f'<polyline class="pair" points="{coordinates}"/>')
            for controller in CONTROLLERS:
                value = indexed.get((track, scenario, seed, controller), math.nan)
                if not math.isfinite(value):
                    continue
                x = controller_x[controller] + jitter
                svg.append(
                    f'<circle cx="{x:.2f}" cy="{y_position(value):.2f}" r="3.2" '
                    f'fill="{CONTROLLER_COLORS[controller]}" opacity="0.86"/>'
                )
        svg.append(
            f'<text x="{center:.2f}" y="{top + plot_height + 28}" '
            f'text-anchor="middle" font-size="12">{html.escape(label)}</text>'
        )

    svg.append(
        f'<text x="24" y="{top + plot_height / 2:.2f}" text-anchor="middle" '
        f'font-size="13" transform="rotate(-90 24 {top + plot_height / 2:.2f})">'
        f'{html.escape(y_label)}</text>'
    )
    legend_x = width - right - 235
    for index, controller in enumerate(CONTROLLERS):
        x = legend_x + index * 80
        svg.append(
            f'<circle cx="{x + 7}" cy="57" r="5" '
            f'fill="{CONTROLLER_COLORS[controller]}"/>'
        )
        svg.append(
            f'<text x="{x + 18}" y="62" font-size="12">'
            f'{CONTROLLER_LABELS[controller]}</text>'
        )
    svg.append('</svg>')
    path.write_text('\n'.join(svg) + '\n', encoding='utf-8')


# Gather ordered mean/confidence triplets for plotting.
def chart_values(indexed, conditions, metric):
    values = {controller: [] for controller in CONTROLLERS}
    for track, scenario in conditions:
        for controller in CONTROLLERS:
            row = indexed.get((controller, track, scenario))
            values[controller].append(
                metric_triplet(row, metric) if row is not None else
                (math.nan, math.nan, math.nan)
            )
    return values


# Convert wide descriptive results into a long/tidy metric table.
def write_tidy_descriptive(root, grouped_rows):
    selected = {
        'cte_rmse_m', 'heading_rmse_rad', 'temporal_position_rmse_m',
        'normalized_command_activity', 'window_recovery_fraction',
        'active_tail_mean_abs_cte_m', 'degradation_cte_rmse_m',
        'degradation_temporal_position_rmse_m',
    }
    fields = [
        'controller_family', 'track', 'scenario', 'metric', 'n', 'mean',
        'stddev', 'median', 'q1', 'q3', 'ci95_low', 'ci95_high',
    ]
    with (root / 'descriptive_results_tidy.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in grouped_rows:
            for metric in sorted(selected):
                count = number(row, f'{metric}_n')
                if not math.isfinite(count) or count <= 0:
                    continue
                writer.writerow({
                    'controller_family': row['controller_family'],
                    'track': row['track'],
                    'scenario': row['scenario'],
                    'metric': metric,
                    'n': str(int(count)),
                    'mean': format_number(number(row, f'{metric}_mean')),
                    'stddev': format_number(number(row, f'{metric}_stddev')),
                    'median': format_number(number(row, f'{metric}_median')),
                    'q1': format_number(number(row, f'{metric}_q1')),
                    'q3': format_number(number(row, f'{metric}_q3')),
                    'ci95_low': format_number(number(row, f'{metric}_ci95_low')),
                    'ci95_high': format_number(number(row, f'{metric}_ci95_high')),
                })


# Produce a concise label for one track/scenario combination.
def condition_label(track, scenario):
    if scenario == 'nominal':
        return f'{TRACK_LABELS.get(track, track)} - nominal'
    return SCENARIO_LABELS.get(scenario, scenario)


# Assemble tables, figures, inference, caveats, and audit into the generated report.
def write_markdown_report(
        root, figure_names, raw_rows, grouped_rows, confirmatory_rows,
        family_rows, completion_rows):
    significant = [
        row for row in confirmatory_rows
        if row['holm_significant_0p05'] == '1'
    ]
    practically_clear = [
        row for row in significant
        if 'better_beyond_practical_threshold' in
        row['practical_interpretation']
    ]
    completion_significant = [
        row for row in completion_rows
        if row['holm_significant_0p05'] == '1'
    ]
    primary_completion = [
        row for row in completion_rows
        if row.get('inference_role') == 'confirmatory_primary'
    ]
    primary_completion_significant = [
        row for row in primary_completion
        if row['holm_significant_0p05'] == '1'
    ]
    completion_is_confirmatory = bool(primary_completion)
    grouped = group_index(grouped_rows)
    conditions = sorted({
        (row['track'], row['scenario']) for row in raw_rows
    })
    lines = [
        '# Raport statistic automat',
        '',
        f'Setul de date contine {len(raw_rows)} de rulari si a trecut auditul '
        'de protocol.',
        'Rezultatele de mai jos sunt calculate automat si trebuie interpretate '
        'inginereste inainte de includerea in teza.',
        '',
        '## Metoda',
        '',
        '- Statisticile descriptive includ media, abaterea standard, mediana, '
        'IQR si intervalul Student-t de 95%.',
        '- Comparatiile folosesc diferente pereche pentru acelasi seed.',
        '- Valorile p provin din testul exact bilateral prin schimbarea semnelor.',
        '- Corectia Holm este aplicata separat in fiecare familie din planul de '
        'analiza.',
        '- Degradarea reprezinta valoarea perturbata minus nominalul aceluiasi '
        'regulator si seed.',
        '- Familiile au fost fixate inaintea interpretarii detaliate, dar analiza '
        'nu a fost preregistrata inaintea colectarii datelor.',
        '- Inferenta se refera la variatia observata in simulator, nu direct la o '
        'populatie de roboti fizici.',
        '',
        '## Rezumat inferential',
        '',
        f'- Familii statistice: {len(family_rows)}',
        f'- Comparatii confirmatorii Holm-semnificative: {len(significant)}',
        '- Comparatii primare de finalizare Holm-semnificative: '
        f'{len(primary_completion_significant)}',
        '- Comparatii semnificative cu CI complet dincolo de pragul practic: '
        f'{len(practically_clear)}',
        '',
    ]
    if significant:
        lines.extend((
            '| Familie | Comparatie | Conditie | Metrica | Diferenta medie | '
            'CI 95% | p Holm | Prag practic | Interpretare practica |',
            '|---|---|---|---|---:|---:|---:|---:|---|',
        ))
        for row in significant:
            comparison = f"{row['candidate'].upper()} - {row['baseline'].upper()}"
            condition = f"{row['track']} / {row['scenario']}"
            interval = f"[{row['ci95_low']}, {row['ci95_high']}]"
            lines.append(
                f"| {row['holm_family']} | {comparison} | {condition} | "
                f"{row['metric']} | {row['mean_candidate_minus_baseline']} | "
                f"{interval} | {row['holm_adjusted_p_value']} | "
                f"{row['practical_threshold']} | "
                f"{row['practical_interpretation']} |"
            )
        lines.append('')

    if completion_is_confirmatory:
        completion_title = (
            '## Finalizarea traiectoriei - analiza confirmatorie predeclarata'
        )
        completion_explanation = (
            'Finalizarea sub scenariile marcate confirmatorii a fost stabilita '
            'ca endpoint primar inaintea colectarii acestor date independente. '
            'Comparatiile pereche folosesc testul McNemar exact bilateral pe '
            'seed-urile discordante si corectia Holm in familia primara. '
            'Conditiile nominale raman diagnostice secundare.'
        )
        completion_file_description = (
            'analiza pereche predeclarata a ratei de finalizare.'
        )
    else:
        completion_title = (
            '## Finalizarea traiectoriei - analiza exploratorie post-hoc'
        )
        completion_explanation = (
            'Rata de finalizare nu a fost inclusa in planul confirmatoriu '
            'inaintea campaniei. Rezultatele din aceasta sectiune sunt, prin '
            'urmare, exploratorii. Comparatiile pereche folosesc testul '
            'McNemar exact bilateral pe seed-urile discordante si corectia '
            'Holm pentru toate comparatiile de finalizare.'
        )
        completion_file_description = (
            'analiza exploratorie pereche a ratei de finalizare.'
        )

    lines.extend((
        completion_title,
        '',
        completion_explanation,
        '',
        '| Conditie | PID | TVLQR | MPC |',
        '|---|---:|---:|---:|',
    ))
    for track, scenario in conditions:
        values = []
        for controller in CONTROLLERS:
            row = grouped.get((controller, track, scenario))
            values.append(
                f"{row['completed']}/{row['runs']}" if row is not None else '-'
            )
        lines.append(
            f'| {condition_label(track, scenario)} | '
            f'{values[0]} | {values[1]} | {values[2]} |'
        )
    lines.append('')
    if completion_significant:
        lines.extend((
            '| Comparatie | Conditie | Finalizari candidat | '
            'Finalizari referinta | p exact | p Holm |',
            '|---|---|---:|---:|---:|---:|',
        ))
        for row in completion_significant:
            comparison = (
                f"{CONTROLLER_LABELS[row['candidate']]} - "
                f"{CONTROLLER_LABELS[row['baseline']]}"
            )
            lines.append(
                f'| {comparison} | '
                f"{condition_label(row['track'], row['scenario'])} | "
                f"{row['candidate_completed']}/{row['paired_samples']} | "
                f"{row['baseline_completed']}/{row['paired_samples']} | "
                f"{row['exact_mcnemar_p_value']} | "
                f"{row['holm_adjusted_p_value']} |"
            )
        lines.append('')

    lines.extend(('## Figuri', ''))
    for name in figure_names:
        lines.append(f'![{name}](figures/{name})')
        lines.append('')
    lines.extend((
        '## Fisiere tabelare',
        '',
        '- `descriptive_results_tidy.csv`: tabel descriptiv in format lung.',
        '- `confirmatory_results.csv`: numai ipotezele confirmatorii.',
        '- `paired_differences.csv`: toate analizele confirmatorii si exploratorii.',
        '- `hypothesis_families.csv`: dimensiunea si rezultatul fiecarei familii.',
        '- `completion_comparison.csv`: ' + completion_file_description,
        '',
    ))
    (root / 'statistical_report.md').write_text(
        '\n'.join(lines), encoding='utf-8'
    )


# Record hashes of analysis inputs/outputs for complete traceability.
def write_analysis_manifest(root):
    protocol = {}
    for line in (root / 'protocol.txt').read_text(encoding='utf-8').splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            protocol[key] = value

    input_paths = list(root.glob('run_*_seed_*/*/track_*/summary.csv'))
    input_paths.extend(root.glob(
        'run_*_seed_*/*/track_*/*/scenario_metadata.csv'
    ))
    output_names = (
        'all_runs.csv', 'group_summary.csv', 'paired_differences.csv',
        'hypothesis_families.csv', 'confirmatory_results.csv',
        'completion_comparison.csv', 'descriptive_results_tidy.csv',
        'protocol_audit.txt',
        'statistical_report.md',
    )
    analysis_script = Path(__file__).resolve().parent / 'analyze_controller_comparison.py'
    report_script = Path(__file__).resolve()
    lines = [
        'CONTROLLER COMPARISON ANALYSIS MANIFEST',
        f'protocol_signature={protocol.get("protocol_signature", "")}',
        f'runtime_source_fingerprint={protocol.get("source_fingerprint", "")}',
        f'raw_summary_and_metadata_files={len(input_paths)}',
        f'raw_summary_and_metadata_sha256={sha256_bundle(root, input_paths)}',
        f'analyzer_sha256={sha256_file(analysis_script)}',
        f'report_generator_sha256={sha256_file(report_script)}',
    ]
    for name in output_names:
        path = root / name
        if not path.is_file():
            raise RuntimeError(f'missing analysis output for manifest: {path}')
        lines.append(f'{name}_sha256={sha256_file(path)}')
    figure_paths = sorted((root / 'figures').glob('*.svg'))
    lines.append(f'svg_figure_count={len(figure_paths)}')
    lines.append(f'svg_figures_sha256={sha256_bundle(root, figure_paths)}')
    (root / 'analysis_manifest.txt').write_text(
        '\n'.join(lines) + '\n', encoding='utf-8'
    )


# Validate inputs and generate SVG, tidy CSV, Markdown, and manifest artifacts.
def main():
    if len(sys.argv) != 2:
        raise SystemExit(
            'usage: generate_controller_comparison_report.py RESULT_DIRECTORY'
        )
    root = Path(sys.argv[1])
    grouped_rows = read_rows(root / 'group_summary.csv')
    raw_rows = read_rows(root / 'all_runs.csv')
    confirmatory_rows = read_rows(root / 'confirmatory_results.csv')
    completion_rows = read_rows(root / 'completion_comparison.csv')
    family_rows = read_rows(root / 'hypothesis_families.csv')
    indexed = group_index(grouped_rows)
    figures = root / 'figures'
    figures.mkdir(exist_ok=True)

    available_conditions = {
        (row['track'], row['scenario']) for row in grouped_rows
    }
    # A secondary campaign may intentionally contain only a subset of the
    # primary matrix. Do not render empty categories that could be mistaken
    # for missing or failed trials.
    nominal_conditions = tuple(
        condition for condition in (
            ('straight', 'nominal'), ('curve', 'nominal'),
            ('circle', 'nominal'), ('figure_eight', 'nominal')
        )
        if condition in available_conditions
    )
    perturbation_conditions = tuple(
        condition for condition in (
            ('figure_eight', scenario) for scenario in
            (*FINITE_SCENARIOS, 'left_wheel_loss_persistent')
        )
        if condition in available_conditions
    )
    finite_conditions = tuple(
        condition for condition in (
            ('figure_eight', scenario) for scenario in FINITE_SCENARIOS
        )
        if condition in available_conditions
    )
    effort_conditions = (*nominal_conditions, *perturbation_conditions)

    specifications = (
        (
            'nominal_temporal_rmse.svg',
            'Precizia temporala nominala', 'RMSE pozitie temporala [m]',
            [TRACK_LABELS[track] for track, _ in nominal_conditions],
            nominal_conditions, 'temporal_position_rmse_m', True,
        ),
        (
            'nominal_cte_rmse.svg',
            'Precizia spatiala nominala', 'RMSE CTE [m]',
            [TRACK_LABELS[track] for track, _ in nominal_conditions],
            nominal_conditions, 'cte_rmse_m', True,
        ),
        (
            'perturbation_cte_degradation.svg',
            'Degradarea CTE fata de nominal', 'Delta RMSE CTE [m]',
            [SCENARIO_LABELS[scenario] for _, scenario in perturbation_conditions],
            perturbation_conditions, 'degradation_cte_rmse_m', True,
        ),
        (
            'transient_recovery_fraction.svg',
            'Proportia ferestrelor cu recuperare', 'Ferestre recuperate / total',
            [SCENARIO_LABELS[scenario] for _, scenario in finite_conditions],
            finite_conditions,
            'window_recovery_fraction', True,
        ),
        (
            'persistent_tail_cte.svg',
            'Eroarea persistenta dupa pierderea rotii',
            'CTE absolut mediu in coada activa [m]',
            ['Pierdere roata permanenta'],
            (('figure_eight', 'left_wheel_loss_persistent'),),
            'active_tail_mean_abs_cte_m', True,
        ),
        (
            'normalized_command_activity.svg',
            'Activitatea normalizata a comenzilor', 'Activitate normalizata [-]',
            [
                TRACK_LABELS[track] if scenario == 'nominal' else
                SCENARIO_LABELS[scenario]
                for track, scenario in effort_conditions
            ],
            effort_conditions, 'normalized_command_activity', True,
        ),
    )

    names = []
    for name, title, y_label, labels, conditions, metric, force_zero in specifications:
        values = chart_values(indexed, conditions, metric)
        # A focused campaign may intentionally omit every finite-duration
        # disturbance.  In that case recovery-fraction fields are undefined,
        # so the report should omit that figure instead of aborting after all
        # simulations have completed successfully.
        if not labels or not grouped_chart_has_finite_value(values):
            continue
        svg_bar_chart(
            figures / name, title, y_label, labels,
            values, force_zero,
        )
        names.append(name)

    paired_specifications = (
        (
            'paired_nominal_temporal_rmse.svg',
            'Distributia pereche a preciziei temporale nominale',
            'RMSE pozitie temporala [m]',
            [TRACK_LABELS[track] for track, _ in nominal_conditions],
            nominal_conditions, 'temporal_position_rmse_m',
        ),
        (
            'paired_persistent_tail_cte.svg',
            'Distributia pereche pentru pierderea permanenta a rotii',
            'CTE absolut mediu in coada activa [m]',
            ['Pierdere roata permanenta'],
            (('figure_eight', 'left_wheel_loss_persistent'),),
            'active_tail_mean_abs_cte_m',
        ),
    )
    for name, title, y_label, labels, conditions, metric in paired_specifications:
        if not labels or not paired_chart_has_finite_value(
                conditions, metric, raw_rows):
            continue
        svg_paired_dot_chart(
            figures / name, title, y_label, labels, conditions, metric, raw_rows
        )
        names.append(name)

    write_tidy_descriptive(root, grouped_rows)
    write_markdown_report(
        root, names, raw_rows, grouped_rows, confirmatory_rows, family_rows,
        completion_rows
    )
    write_analysis_manifest(root)
    print(f'Wrote {len(names)} SVG figures below {figures}')
    print(f'Wrote {root / "descriptive_results_tidy.csv"}')
    print(f'Wrote {root / "statistical_report.md"}')
    print(f'Wrote {root / "analysis_manifest.txt"}')


if __name__ == '__main__':
    # Keep rendering helpers importable by unit tests.
    main()
