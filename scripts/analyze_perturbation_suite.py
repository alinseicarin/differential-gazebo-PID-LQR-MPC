#!/usr/bin/env python3
"""Aggregate controller-independent robustness metrics from suite CSV files."""

import bisect
import csv
import math
import sys
from pathlib import Path


def read_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def number(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate(times, values, query):
    if not times or query < times[0] or query > times[-1]:
        return math.nan
    upper = bisect.bisect_left(times, query)
    if upper == 0:
        return values[0]
    if upper == len(times):
        return values[-1]
    before = upper - 1
    interval = times[upper] - times[before]
    if interval <= 0.0:
        return math.nan
    fraction = (query - times[before]) / interval
    return values[before] + fraction * (values[upper] - values[before])


def time_rms(rows, key):
    integral = 0.0
    duration = 0.0
    previous_time = None
    previous_value = None
    for row in rows:
        current_time = number(row, 'time')
        current_value = number(row, key)
        if not math.isfinite(current_time) or not math.isfinite(current_value):
            continue
        if previous_time is not None and current_time > previous_time:
            dt = current_time - previous_time
            integral += 0.5 * (
                previous_value * previous_value + current_value * current_value
            ) * dt
            duration += dt
        previous_time = current_time
        previous_value = current_value
    return math.sqrt(integral / duration) if duration > 0.0 else math.nan


def active_window_metrics(rows, start, end):
    active = [
        row for row in rows
        if start <= number(row, 'time') < end
    ]
    tail_start = max(start, end - 2.0)
    tail = [
        abs(number(row, 'true_cross_track_error')) for row in active
        if number(row, 'time') >= tail_start and
        math.isfinite(number(row, 'true_cross_track_error'))
    ]

    iae = 0.0
    previous_time = None
    previous_value = None
    for row in active:
        current_time = number(row, 'time')
        current_value = abs(number(row, 'true_cross_track_error'))
        if not math.isfinite(current_time) or not math.isfinite(current_value):
            continue
        if previous_time is not None and current_time > previous_time:
            iae += 0.5 * (previous_value + current_value) * (
                current_time - previous_time
            )
        previous_time = current_time
        previous_value = current_value
    return iae, sum(tail) / len(tail) if tail else math.nan


def baseline_deviation(rows, baseline_rows, fault_start, fault_end):
    baseline_times = [number(row, 'time') for row in baseline_rows]
    baseline_cte = [number(row, 'true_cross_track_error') for row in baseline_rows]
    baseline_heading = [number(row, 'true_path_heading_error') for row in baseline_rows]

    deviations = []
    for row in rows:
        time = number(row, 'time')
        cte = number(row, 'true_cross_track_error')
        heading = number(row, 'true_path_heading_error')
        base_cte = interpolate(baseline_times, baseline_cte, time)
        base_heading = interpolate(baseline_times, baseline_heading, time)
        if all(math.isfinite(value) for value in (
                time, cte, heading, base_cte, base_heading)):
            deviations.append((
                time,
                abs(cte - base_cte),
                abs(wrap_angle(heading - base_heading)),
            ))

    # Incremental peaks must not include ordinary run-to-run differences that
    # occur before the disturbance has actually started.
    affected = [item for item in deviations if item[0] >= fault_start]
    peak_cte = max((item[1] for item in affected), default=math.nan)
    peak_heading = max((item[2] for item in affected), default=math.nan)

    # Recovery means one complete second continuously inside a band around the
    # controller's own nominal response. Absolute RMS metrics remain separate,
    # so a poor nominal controller cannot appear good merely by returning to it.
    stable_start = None
    recovery = math.nan
    for time, cte_delta, heading_delta in deviations:
        if time < fault_end:
            continue
        if cte_delta <= 0.01 and heading_delta <= 0.02:
            if stable_start is None:
                stable_start = time
            if time - stable_start >= 1.0:
                recovery = stable_start - fault_end
                break
        else:
            stable_start = None
    return peak_cte, peak_heading, recovery


def command_metrics(path):
    rows = read_rows(path)
    applied_angular = [
        abs(number(row, 'applied_angular_command')) for row in rows
        if math.isfinite(number(row, 'applied_angular_command'))
    ]
    saturation_samples = sum(value >= 1.5 - 1.0e-6 for value in applied_angular)
    activity = 0.0
    previous_time = None
    previous_value = None
    for row in rows:
        time = number(row, 'time')
        linear = number(row, 'applied_linear_command')
        angular = number(row, 'applied_angular_command')
        if not all(math.isfinite(value) for value in (time, linear, angular)):
            continue
        # The common limits are 1 m/s and 1.5 rad/s. This is the same
        # dimensionless command-activity integral used in the thesis method.
        value = linear * linear + (angular / 1.5) ** 2
        if previous_time is not None and time > previous_time:
            activity += 0.5 * (previous_value + value) * (
                time - previous_time
            )
        previous_time = time
        previous_value = value
    return (
        max(applied_angular, default=math.nan),
        saturation_samples / len(applied_angular) if applied_angular else math.nan,
        sum(int(float(row['fault_active'])) for row in rows),
        activity,
    )


def noise_metrics(path):
    rows = read_rows(path)
    position_squared = []
    yaw_squared = []
    for row in rows:
        if int(float(row['fault_active'])) != 1:
            continue
        x_error = number(row, 'x_perturbation')
        y_error = number(row, 'y_perturbation')
        yaw_error = number(row, 'yaw_perturbation')
        if math.isfinite(x_error) and math.isfinite(y_error):
            position_squared.append(x_error * x_error + y_error * y_error)
        if math.isfinite(yaw_error):
            yaw_squared.append(yaw_error * yaw_error)
    position_rms = math.sqrt(sum(position_squared) / len(position_squared)) \
        if position_squared else 0.0
    yaw_rms = math.sqrt(sum(yaw_squared) / len(yaw_squared)) \
        if yaw_squared else 0.0
    return position_rms, yaw_rms


def enabled(metadata, key):
    return metadata.get(key, '').strip().lower() == 'true'


def audit_fault_timing(
        scenario, metadata, truth_rows, command_rows, odometry_rows):
    """Verify that every injected fault uses the evaluator's time origin."""
    issues = []
    origins = [
        number(row, 'truth_stamp') - number(row, 'time')
        for row in truth_rows
        if math.isfinite(number(row, 'truth_stamp')) and
        math.isfinite(number(row, 'time'))
    ]
    if not origins:
        return [f'{scenario}: ground-truth experiment time origin is missing']
    origins.sort()
    experiment_origin = origins[len(origins) // 2]
    if origins[-1] - origins[0] > 1.0e-5:
        issues.append(
            f'{scenario}: ground-truth time origin is not constant '
            f'(spread {origins[-1] - origins[0]:.6g} s)'
        )

    expected_start = float(metadata['fault_start'])
    tolerance = 0.075  # More than two 30 Hz samples, but far below 1 s.
    command_active = [
        row for row in command_rows if number(row, 'fault_active') == 1.0
    ]
    command_expected = enabled(metadata, 'command_fault_enabled')
    if command_expected and not command_active:
        issues.append(f'{scenario}: command fault was enabled but never active')
    if not command_expected and command_active:
        issues.append(f'{scenario}: command fault was active although disabled')
    if command_active:
        logged_start = number(command_active[0], 'time')
        physical_start = (
            number(command_active[0], 'stamp') - experiment_origin
        )
        if not math.isfinite(logged_start) or abs(logged_start - expected_start) > tolerance:
            issues.append(
                f'{scenario}: command CSV starts fault at {logged_start:.6g} s, '
                f'expected {expected_start:.6g} s'
            )
        if not math.isfinite(physical_start) or abs(physical_start - expected_start) > tolerance:
            issues.append(
                f'{scenario}: command reaches Gazebo at experiment time '
                f'{physical_start:.6g} s, expected {expected_start:.6g} s'
            )

    odometry_active = [
        row for row in odometry_rows if number(row, 'fault_active') == 1.0
    ]
    feedback_expected = enabled(metadata, 'feedback_fault_enabled')
    if feedback_expected and not odometry_active:
        issues.append(f'{scenario}: feedback fault was enabled but never active')
    if not feedback_expected and odometry_active:
        issues.append(f'{scenario}: feedback fault was active although disabled')
    if odometry_active:
        logged_start = number(odometry_active[0], 'time')
        physical_start = (
            number(odometry_active[0], 'stamp') - experiment_origin
        )
        if not math.isfinite(logged_start) or abs(logged_start - expected_start) > tolerance:
            issues.append(
                f'{scenario}: odometry CSV starts fault at {logged_start:.6g} s, '
                f'expected {expected_start:.6g} s'
            )
        if not math.isfinite(physical_start) or abs(physical_start - expected_start) > tolerance:
            issues.append(
                f'{scenario}: feedback fault reaches the controller at '
                f'experiment time {physical_start:.6g} s, '
                f'expected {expected_start:.6g} s'
            )
    return issues


def audit_sampling_frequency(scenario, stream, rows, time_key, expected=30.0):
    times = [number(row, time_key) for row in rows]
    times = [value for value in times if math.isfinite(value)]
    if len(times) < 2:
        return [f'{scenario}: {stream} has fewer than two finite timestamps']
    if any(current < previous for previous, current in zip(times, times[1:])):
        return [f'{scenario}: {stream} timestamps move backwards']
    # A safety stop and the first control action can legitimately be published
    # during the same simulation tick. Count that tick once for rate auditing.
    distinct_times = [times[0]]
    for value in times[1:]:
        if value > distinct_times[-1]:
            distinct_times.append(value)
    if len(distinct_times) < 2:
        return [f'{scenario}: {stream} has fewer than two distinct timestamps']
    duration = distinct_times[-1] - distinct_times[0]
    measured = (len(distinct_times) - 1) / duration if duration > 0.0 else math.nan
    # Gazebo plugins quantize their requested 30 Hz period to the 1 ms physics
    # grid (ground truth is normally about 29.41 Hz), hence a 5% tolerance.
    if not math.isfinite(measured) or abs(measured - expected) > 0.05 * expected:
        return [
            f'{scenario}: {stream} effective frequency is {measured:.6g} Hz, '
            f'expected approximately {expected:.6g} Hz'
        ]
    return []


def format_value(value):
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    return '' if not math.isfinite(value) else f'{value:.9f}'


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: analyze_perturbation_suite.py RESULT_DIRECTORY')
    root = Path(sys.argv[1])
    nominal_path = root / 'nominal' / 'ground_truth.csv'
    if not nominal_path.is_file():
        raise SystemExit(f'nominal baseline missing: {nominal_path}')
    baseline_rows = read_rows(nominal_path)

    output_rows = []
    audit_issues = []
    for scenario_dir in sorted(path for path in root.iterdir() if path.is_dir()):
        metadata_path = scenario_dir / 'scenario_metadata.csv'
        controller_path = scenario_dir / 'controller.csv'
        truth_path = scenario_dir / 'ground_truth.csv'
        command_path = scenario_dir / 'applied_commands.csv'
        odometry_path = scenario_dir / 'disturbed_odometry.csv'
        if not all(path.is_file() for path in (
                metadata_path, controller_path, truth_path, command_path,
                odometry_path)):
            continue

        metadata = read_rows(metadata_path)[0]
        controller_rows = read_rows(controller_path)
        rows = read_rows(truth_path)
        command_rows = read_rows(command_path)
        odometry_rows = read_rows(odometry_path)
        start = float(metadata['fault_start'])
        duration = float(metadata['fault_duration'])
        end = start + duration
        is_nominal = metadata['scenario'] == 'nominal'
        audit_issues.extend(audit_fault_timing(
            scenario_dir.name, metadata, rows, command_rows, odometry_rows
        ))
        for stream, samples, time_key in (
                ('controller', controller_rows, 'time'),
                ('ground truth', rows, 'truth_stamp'),
                ('applied command', command_rows, 'time'),
                ('feedback path', odometry_rows, 'time')):
            audit_issues.extend(audit_sampling_frequency(
                scenario_dir.name, stream, samples, time_key
            ))

        cte_values = [
            abs(number(row, 'true_cross_track_error')) for row in rows
            if math.isfinite(number(row, 'true_cross_track_error'))
        ]
        heading_values = [
            abs(number(row, 'true_path_heading_error')) for row in rows
            if math.isfinite(number(row, 'true_path_heading_error'))
        ]
        active_iae, active_tail_mean = active_window_metrics(rows, start, end)
        if is_nominal:
            peak_delta_cte = math.nan
            peak_delta_heading = math.nan
            recovery = math.nan
            active_iae = math.nan
            active_tail_mean = math.nan
        else:
            peak_delta_cte, peak_delta_heading, recovery = baseline_deviation(
                rows, baseline_rows, start, end
            )
        (
            max_angular, saturation_fraction, active_samples,
            normalized_command_activity,
        ) = command_metrics(command_path)
        position_noise_rms, yaw_noise_rms = noise_metrics(odometry_path)

        output_rows.append({
            'case': scenario_dir.name,
            'controller_family': metadata.get('controller_family', 'pid'),
            'scenario': metadata['scenario'],
            'fault_domain': metadata['fault_domain'],
            'track_complete': int(metadata['track_complete']),
            'cte_rmse_m': time_rms(rows, 'true_cross_track_error'),
            'heading_rmse_rad': time_rms(rows, 'true_path_heading_error'),
            'temporal_position_rmse_m': time_rms(rows, 'true_position_error'),
            'peak_abs_cte_m': max(cte_values, default=math.nan),
            'peak_abs_heading_rad': max(heading_values, default=math.nan),
            'active_cte_iae_m_s': active_iae,
            'active_tail_mean_abs_cte_m': active_tail_mean,
            'peak_incremental_cte_vs_nominal_m': peak_delta_cte,
            'peak_incremental_heading_vs_nominal_rad': peak_delta_heading,
            'baseline_relative_recovery_s': recovery,
            'max_abs_applied_angular_command_rad_s': max_angular,
            'angular_saturation_fraction': saturation_fraction,
            'normalized_command_activity': normalized_command_activity,
            'command_fault_active_samples': active_samples,
            'injected_position_noise_rms_m': position_noise_rms,
            'injected_yaw_noise_rms_rad': yaw_noise_rms,
        })

    if not output_rows:
        raise SystemExit('no complete scenario datasets found')

    audit_path = root / 'dataset_audit.txt'
    with audit_path.open('w', encoding='utf-8') as stream:
        stream.write('PERTURBATION DATASET AUDIT\n')
        stream.write(f'issues={len(audit_issues)}\n')
        if audit_issues:
            stream.write('\n'.join(audit_issues) + '\n')
        else:
            stream.write(
                'PASS: command, feedback, evaluator, and declared fault '
                'windows use the same experiment time origin; logged streams '
                'run at approximately 30 Hz.\n'
            )
    if audit_issues:
        raise SystemExit(
            f'dataset audit failed with {len(audit_issues)} issue(s); '
            f'see {audit_path}'
        )

    summary_path = root / 'summary.csv'
    with summary_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]))
        writer.writeheader()
        for row in output_rows:
            writer.writerow({key: format_value(value) for key, value in row.items()})

    text_path = root / 'summary.txt'
    with text_path.open('w', encoding='utf-8') as stream:
        controller_families = sorted({
            row['controller_family'].upper() for row in output_rows
        })
        stream.write(
            f"{'/'.join(controller_families)} VALIDATION AND PERTURBATION SUITE\n"
        )
        stream.write(
            'Perturbed cases use the nominal directory as their time-aligned '
            'baseline; nominal-only directories may contain other tracks.\n'
        )
        stream.write(
            'Recovery: first continuous 1 s interval after fault end with '
            '|delta CTE| <= 0.01 m and |delta heading| <= 0.02 rad relative '
            'to the time-aligned nominal response of the same controller.\n'
        )
        stream.write(
            'Absolute RMS and peak values use Gazebo ground truth and remain '
            'the primary cross-controller performance metrics.\n\n'
        )
        for row in output_rows:
            recovery = format_value(row['baseline_relative_recovery_s'])
            stream.write(
                f"{row['case']} ({row['scenario']}): "
                f"complete={row['track_complete']}, "
                f"CTE_RMSE={row['cte_rmse_m']:.6f} m, "
                f"peak_CTE={row['peak_abs_cte_m']:.6f} m, "
                f"recovery={recovery or 'not_recovered/not_applicable'} s\n"
            )

    print(f'Wrote {summary_path}')
    print(f'Wrote {text_path}')


if __name__ == '__main__':
    main()
