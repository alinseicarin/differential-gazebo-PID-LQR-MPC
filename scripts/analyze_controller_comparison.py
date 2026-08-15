#!/usr/bin/env python3
"""Aggregate paired PID--TVLQR--MPC trials and audit protocol equality."""

import csv
import hashlib
import math
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path


METRICS = (
    'cte_rmse_m',
    'heading_rmse_rad',
    'temporal_position_rmse_m',
    'peak_abs_cte_m',
    'peak_abs_heading_rad',
    'active_cte_iae_m_s',
    'baseline_relative_recovery_s',
    'max_abs_applied_angular_command_rad_s',
    'angular_saturation_fraction',
    'normalized_command_activity',
)

COMMON_CONTROLLER_PARAMETERS = (
    'nominal_control_frequency',
    'max_control_dt',
    'goal_tolerance',
    'goal_heading_tolerance',
    'max_linear_velocity',
    'max_angular_velocity',
    'translation_stop_lateral_error',
    'translation_stop_heading_error',
    'odom_timeout',
    'startup_settling_time',
)

COMMON_REFERENCE_PARAMETERS = (
    'search_window',
    'reference_linear_velocity',
    'curvature_speed_gain',
    'endpoint_slowdown_distance',
    'maximum_reference_curvature',
    'trajectory_spatial_step',
    'maximum_reference_linear_acceleration',
    'maximum_reference_linear_deceleration',
    'maximum_reference_angular_velocity',
)

# Two-sided 95% Student-t critical values indexed by degrees of freedom.
T_975 = (
    math.nan, 12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365,
    2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145, 2.131,
    2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069,
    2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042,
)


def read_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def read_protocol(root):
    values = {}
    path = root / 'protocol.txt'
    if not path.is_file():
        raise RuntimeError(f'missing comparison protocol: {path}')
    for line in path.read_text(encoding='utf-8').splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            values[key] = value
    required = (
        'controllers', 'nominal_tracks', 'robustness_track',
        'robustness_scenarios', 'repetitions', 'base_gazebo_seed',
        'base_noise_seed', 'ros_domain_id', 'source_fingerprint',
        'protocol_signature',
    )
    missing = [key for key in required if key not in values]
    if missing:
        raise RuntimeError(f'incomplete comparison protocol: {missing}')
    return values


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            digest.update(block)
    return digest.hexdigest()


def read_yaml_scalars(path, selected_keys=None):
    """Read the simple scalar key/value lines used by controller YAML files."""
    if selected_keys is None:
        selected_keys = (
            set(COMMON_CONTROLLER_PARAMETERS) |
            set(COMMON_REFERENCE_PARAMETERS)
        )
    values = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        content = line.split('#', 1)[0].strip()
        if ':' not in content:
            continue
        key, value = content.split(':', 1)
        if key in selected_keys and value.strip():
            values[key] = value.strip()
    return values


def finite(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def format_value(value):
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    return '' if not math.isfinite(value) else f'{value:.9f}'


def confidence_interval(values):
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return math.nan, math.nan, math.nan, math.nan
    mean = statistics.fmean(values)
    if len(values) == 1:
        return mean, math.nan, math.nan, math.nan
    deviation = statistics.stdev(values)
    degrees = len(values) - 1
    critical = T_975[degrees] if degrees < len(T_975) else 1.96
    half_width = critical * deviation / math.sqrt(len(values))
    return mean, deviation, mean - half_width, mean + half_width


def parse_run_directory(summary_path):
    # Expected layout: run_01_seed_500/pid/track_circle/summary.csv
    track_dir = summary_path.parent
    controller_dir = track_dir.parent
    run_dir = controller_dir.parent
    match = re.fullmatch(r'run_(\d+)_seed_(\d+)', run_dir.name)
    if match is None or not track_dir.name.startswith('track_'):
        return None
    return {
        'repetition': int(match.group(1)),
        'gazebo_seed': int(match.group(2)),
        'controller_family': controller_dir.name,
        'track': track_dir.name.removeprefix('track_'),
    }


def load_trials(root):
    trials = []
    for summary_path in sorted(root.glob('run_*_seed_*/*/track_*/summary.csv')):
        identity = parse_run_directory(summary_path)
        if identity is None:
            continue
        for summary in read_rows(summary_path):
            case_dir = summary_path.parent / summary['case']
            metadata_path = case_dir / 'scenario_metadata.csv'
            if not metadata_path.is_file():
                raise RuntimeError(f'missing metadata: {metadata_path}')
            metadata = read_rows(metadata_path)[0]
            trial = dict(identity)
            trial['directory_controller'] = identity['controller_family']
            trial.update(summary)
            trial['metadata_gazebo_seed'] = metadata.get('gazebo_seed', '')
            trial['noise_seed'] = metadata.get('noise_seed', '')
            trial['controller_config_sha256'] = metadata.get(
                'controller_config_sha256', ''
            )
            trial['reference_config_sha256'] = metadata.get(
                'reference_config_sha256', ''
            )
            trial['track_sha256'] = metadata.get('track_sha256', '')
            trial['fault_signature'] = '|'.join(metadata.get(key, '') for key in (
                'fault_domain', 'fault_start', 'fault_duration',
                'command_fault_enabled', 'feedback_fault_enabled',
                'angular_bias', 'left_wheel_effectiveness',
                'right_wheel_effectiveness', 'command_delay',
                'position_noise_stddev', 'yaw_noise_stddev', 'noise_seed',
            ))
            trials.append(trial)
    if not trials:
        raise RuntimeError(f'no comparison summaries found below {root}')
    return trials


def write_all_trials(root, trials):
    stable_fields = [
        'repetition', 'gazebo_seed', 'controller_family', 'track', 'scenario',
        'fault_domain', 'track_complete', *METRICS, 'noise_seed',
        'controller_config_sha256', 'reference_config_sha256', 'track_sha256',
    ]
    with (root / 'all_runs.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=stable_fields)
        writer.writeheader()
        for trial in trials:
            writer.writerow({key: trial.get(key, '') for key in stable_fields})


def write_group_summary(root, trials):
    groups = defaultdict(list)
    for trial in trials:
        groups[(
            trial['controller_family'], trial['track'], trial['scenario']
        )].append(trial)

    fields = [
        'controller_family', 'track', 'scenario', 'runs', 'completed',
        'completion_rate',
    ]
    for metric in METRICS:
        fields.extend((
            f'{metric}_n', f'{metric}_mean', f'{metric}_stddev',
            f'{metric}_ci95_low', f'{metric}_ci95_high',
        ))

    output = []
    for key, rows in sorted(groups.items()):
        completed = sum(int(float(row['track_complete'])) for row in rows)
        result = {
            'controller_family': key[0],
            'track': key[1],
            'scenario': key[2],
            'runs': len(rows),
            'completed': completed,
            'completion_rate': completed / len(rows),
        }
        for metric in METRICS:
            values = [finite(row, metric) for row in rows]
            valid = [value for value in values if math.isfinite(value)]
            mean, deviation, low, high = confidence_interval(valid)
            result.update({
                f'{metric}_n': len(valid),
                f'{metric}_mean': mean,
                f'{metric}_stddev': deviation,
                f'{metric}_ci95_low': low,
                f'{metric}_ci95_high': high,
            })
        output.append(result)

    with (root / 'group_summary.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in output:
            writer.writerow({key: format_value(value) for key, value in row.items()})
    return output


def write_paired_differences(root, trials):
    indexed = {}
    controllers = set()
    for trial in trials:
        controllers.add(trial['controller_family'])
        key = (
            trial['controller_family'], trial['gazebo_seed'],
            trial['track'], trial['scenario'],
        )
        if key in indexed:
            raise RuntimeError(f'duplicate trial identity: {key}')
        indexed[key] = trial

    preferred_pairs = (
        ('lqr', 'pid'), ('mpc', 'pid'), ('mpc', 'lqr'),
    )
    output = []
    conditions = sorted({
        (trial['gazebo_seed'], trial['track'], trial['scenario'])
        for trial in trials
    })
    for candidate, baseline in preferred_pairs:
        if candidate not in controllers or baseline not in controllers:
            continue
        for track, scenario in sorted({(item[1], item[2]) for item in conditions}):
            matched = []
            for seed, condition_track, condition_scenario in conditions:
                if (condition_track, condition_scenario) != (track, scenario):
                    continue
                candidate_row = indexed.get((candidate, seed, track, scenario))
                baseline_row = indexed.get((baseline, seed, track, scenario))
                if candidate_row is not None and baseline_row is not None:
                    matched.append((candidate_row, baseline_row))
            for metric in METRICS:
                differences = []
                for candidate_row, baseline_row in matched:
                    candidate_value = finite(candidate_row, metric)
                    baseline_value = finite(baseline_row, metric)
                    if math.isfinite(candidate_value) and math.isfinite(baseline_value):
                        differences.append(candidate_value - baseline_value)
                mean, deviation, low, high = confidence_interval(differences)
                output.append({
                    'candidate': candidate,
                    'baseline': baseline,
                    'track': track,
                    'scenario': scenario,
                    'metric': metric,
                    'paired_samples': len(differences),
                    'mean_candidate_minus_baseline': mean,
                    'stddev_difference': deviation,
                    'ci95_low': low,
                    'ci95_high': high,
                })

    fields = list(output[0]) if output else [
        'candidate', 'baseline', 'track', 'scenario', 'metric',
        'paired_samples', 'mean_candidate_minus_baseline',
        'stddev_difference', 'ci95_low', 'ci95_high',
    ]
    with (root / 'paired_differences.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in output:
            writer.writerow({key: format_value(value) for key, value in row.items()})


def audit_protocol(root, trials, protocol):
    issues = []
    if not (root / 'protocol_source.tar.gz').is_file():
        issues.append('archived runtime source is missing')
    expected_controllers = set(protocol['controllers'].split())
    repetition_count = int(protocol['repetitions'])
    base_seed = int(protocol['base_gazebo_seed'])
    expected_conditions = set()
    for offset in range(repetition_count):
        seed = base_seed + offset
        for track in protocol['nominal_tracks'].split():
            expected_conditions.add((seed, track, 'nominal'))
        if protocol['robustness_scenarios']:
            for scenario in protocol['robustness_scenarios'].split():
                expected_conditions.add((
                    seed, protocol['robustness_track'], scenario
                ))
    conditions = defaultdict(list)
    for trial in trials:
        conditions[(
            trial['gazebo_seed'], trial['track'], trial['scenario']
        )].append(trial)

    actual_conditions = set(conditions)
    for condition in sorted(expected_conditions - actual_conditions):
        issues.append(f'{condition}: condition is completely missing')
    for condition in sorted(actual_conditions - expected_conditions):
        issues.append(f'{condition}: unexpected condition')

    for condition, rows in sorted(conditions.items()):
        condition_controllers = {row['directory_controller'] for row in rows}
        if condition_controllers != expected_controllers:
            issues.append(
                f'{condition}: controllers {sorted(condition_controllers)}; '
                f'expected {sorted(expected_controllers)}'
            )
        if any(
                row['controller_family'] != row['directory_controller']
                for row in rows):
            issues.append(f'{condition}: controller metadata/path mismatch')
        for field in (
            'track_sha256', 'reference_config_sha256', 'fault_signature',
            'metadata_gazebo_seed',
        ):
            values = {row[field] for row in rows}
            if len(values) != 1 or '' in values:
                issues.append(
                    f'{condition}: inconsistent or missing {field}: {sorted(values)}'
                )

    configs = defaultdict(set)
    for trial in trials:
        configs[trial['controller_family']].add(
            trial['controller_config_sha256']
        )
    for controller, hashes in sorted(configs.items()):
        if len(hashes) != 1 or '' in hashes:
            issues.append(
                f'{controller}: controller configuration changed: {sorted(hashes)}'
            )

    archived_values = {}
    for controller in sorted(expected_controllers):
        path = root / 'protocol_configs' / f'{controller}.yaml'
        if not path.is_file():
            issues.append(f'{controller}: archived controller config is missing')
            continue
        archived_values[controller] = read_yaml_scalars(path)
        runtime_hashes = configs.get(controller, set())
        if runtime_hashes != {sha256(path)}:
            issues.append(
                f'{controller}: archived config does not match runtime hash'
            )

    for parameter in COMMON_CONTROLLER_PARAMETERS:
        values = {
            controller: archived_values.get(controller, {}).get(parameter, '')
            for controller in expected_controllers
        }
        if len(set(values.values())) != 1 or '' in values.values():
            issues.append(f'common parameter {parameter} differs: {values}')

    reference_path = root / 'protocol_configs' / 'trajectory_reference.yaml'
    if not reference_path.is_file():
        issues.append('archived timed-reference configuration is missing')
    else:
        archived_reference_hash = sha256(reference_path)
        runtime_reference_hashes = {
            trial['reference_config_sha256'] for trial in trials
        }
        if runtime_reference_hashes != {archived_reference_hash}:
            issues.append(
                'archived timed-reference config does not match runtime hash'
            )
        reference_values = read_yaml_scalars(
            reference_path, set(COMMON_REFERENCE_PARAMETERS)
        )
        for parameter in COMMON_REFERENCE_PARAMETERS:
            expected = reference_values.get(parameter, '')
            if not expected:
                issues.append(
                    f'timed-reference parameter {parameter} is missing'
                )
                continue
            values = {
                controller: archived_values.get(controller, {}).get(
                    parameter, ''
                )
                for controller in expected_controllers
            }
            if any(value != expected for value in values.values()):
                issues.append(
                    f'timed-reference parameter {parameter} differs from '
                    f'authoritative value {expected}: {values}'
                )

    path = root / 'protocol_audit.txt'
    with path.open('w', encoding='utf-8') as stream:
        stream.write('PAIRED COMPARISON PROTOCOL AUDIT\n')
        stream.write(f'trials={len(trials)}\n')
        stream.write(f'conditions={len(conditions)}\n')
        stream.write(f'issues={len(issues)}\n')
        if issues:
            stream.write('\n'.join(issues) + '\n')
        else:
            stream.write(
                'PASS: track, timed-reference configuration, disturbance '
                'schedule, paired seeds, actuator limits, timing, completion '
                'criteria, and safety thresholds agree as required.\n'
            )
    return issues


def write_text_summary(root, trials, grouped, issues):
    with (root / 'summary.txt').open('w', encoding='utf-8') as stream:
        stream.write('PID--TVLQR--MPC PAIRED COMPARISON\n')
        stream.write(f'Individual scenario runs: {len(trials)}\n')
        stream.write(f'Aggregated conditions: {len(grouped)}\n')
        stream.write(f'Protocol audit issues: {len(issues)}\n')
        stream.write(
            'Confidence intervals use paired differences and the two-sided '
            'Student-t 95% critical value. A negative candidate-minus-baseline '
            'difference means that the candidate produced a smaller metric.\n'
        )


def main():
    if len(sys.argv) != 2:
        raise SystemExit(
            'usage: analyze_controller_comparison.py RESULT_DIRECTORY'
        )
    root = Path(sys.argv[1])
    trials = load_trials(root)
    write_all_trials(root, trials)
    grouped = write_group_summary(root, trials)
    write_paired_differences(root, trials)
    protocol = read_protocol(root)
    issues = audit_protocol(root, trials, protocol)
    write_text_summary(root, trials, grouped, issues)
    print(f'Wrote {root / "all_runs.csv"}')
    print(f'Wrote {root / "group_summary.csv"}')
    print(f'Wrote {root / "paired_differences.csv"}')
    print(f'Wrote {root / "protocol_audit.txt"}')
    if issues:
        raise SystemExit(f'protocol audit failed with {len(issues)} issue(s)')


if __name__ == '__main__':
    main()
