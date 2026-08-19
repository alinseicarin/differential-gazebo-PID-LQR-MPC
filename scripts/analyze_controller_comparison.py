#!/usr/bin/env python3
"""Aggregate paired PID--TVLQR--MPC trials and audit protocol equality."""

import argparse
import csv
import hashlib
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path


BASE_METRICS = (
    'cte_rmse_m',
    'heading_rmse_rad',
    'temporal_position_rmse_m',
    'peak_abs_cte_m',
    'peak_abs_heading_rad',
    'active_cte_iae_m_s',
    'active_tail_mean_abs_cte_m',
    'peak_incremental_cte_vs_nominal_m',
    'peak_incremental_heading_vs_nominal_rad',
    'baseline_relative_recovery_s',
    'window_recovery_fraction',
    'max_abs_applied_angular_command_rad_s',
    'angular_saturation_fraction',
    'normalized_command_activity',
)

# Robustness is not only the absolute response under a fault.  A controller
# that is less accurate nominally can otherwise appear artificially robust.
# These paired-with-nominal metrics quantify how much each perturbation changes
# the same controller under the same Gazebo seed.
DEGRADATION_SOURCES = {
    'degradation_cte_rmse_m': 'cte_rmse_m',
    'degradation_heading_rmse_rad': 'heading_rmse_rad',
    'degradation_temporal_position_rmse_m': 'temporal_position_rmse_m',
    'degradation_peak_abs_cte_m': 'peak_abs_cte_m',
    'degradation_normalized_command_activity': 'normalized_command_activity',
}

METRICS = BASE_METRICS + tuple(DEGRADATION_SOURCES)

FINITE_PERTURBATION_SCENARIOS = {
    'angular_pulse_train',
    'angular_constant',
    'left_wheel_loss',
    'command_delay',
    'localization_noise',
    'localization_yaw_bias',
}

# A positive difference is favorable only for recovery fraction.  Every other
# metric is an error, duration, peak, saturation, or command-effort quantity
# for which a smaller value is preferable.
HIGHER_IS_BETTER = {'window_recovery_fraction'}

# Engineering-relevance bands reuse the already documented recovery tolerances
# where units agree.  Empty entries are deliberately left uninterpreted rather
# than inventing an arbitrary practical threshold.
PRACTICAL_THRESHOLDS = {
    'cte_rmse_m': 0.01,
    'temporal_position_rmse_m': 0.01,
    'peak_abs_cte_m': 0.01,
    'active_tail_mean_abs_cte_m': 0.01,
    'peak_incremental_cte_vs_nominal_m': 0.01,
    'degradation_cte_rmse_m': 0.01,
    'degradation_temporal_position_rmse_m': 0.01,
    'degradation_peak_abs_cte_m': 0.01,
    'heading_rmse_rad': 0.02,
    'peak_abs_heading_rad': 0.02,
    'peak_incremental_heading_vs_nominal_rad': 0.02,
    'degradation_heading_rmse_rad': 0.02,
}

HEADING_PRACTICAL_METRICS = {
    'heading_rmse_rad',
    'peak_abs_heading_rad',
    'peak_incremental_heading_vs_nominal_rad',
    'degradation_heading_rmse_rad',
}


# Expand the two engineering tolerances into metric-specific thresholds while
# preserving whether a metric is measured in metres or radians.
def practical_threshold_map(position_threshold_m, heading_threshold_rad):
    """Apply one predeclared band consistently within each physical unit."""
    return {
        metric: (
            heading_threshold_rad
            if metric in HEADING_PRACTICAL_METRICS
            else position_threshold_m
        )
        for metric in PRACTICAL_THRESHOLDS
    }


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


# Load one headered CSV as dictionaries.
def read_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


# Read and validate the frozen text protocol from the result root.
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


# Hash a file to detect any configuration/reference changes between runs.
def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            digest.update(block)
    return digest.hexdigest()


# Extract the simple scalar subset needed for cross-controller parity checks.
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


# Parse a named field as a finite float, otherwise retain an explicit NaN.
def finite(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


# Serialize mixed numeric/text fields deterministically for output CSV files.
def format_value(value):
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    return '' if not math.isfinite(value) else f'{value:.9f}'


# Compute an interpolated empirical quantile after discarding missing values.
def quantile(values, probability):
    """Return a linearly interpolated sample quantile (Hyndman--Fan type 7)."""
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        return math.nan
    if not 0.0 <= probability <= 1.0:
        raise ValueError('quantile probability must lie in [0, 1]')
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


# Calculate mean, standard deviation, and two-sided 95% Student-t interval.
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


# Exact paired randomization test obtained by enumerating signs of nonzero
# controller differences under the null hypothesis of exchangeability.
def exact_sign_flip_test(values):
    """Two-sided paired randomization test for a zero centered difference.

    All 2^n sign assignments are enumerated for the thesis protocol (n=10).
    This avoids an additional statistics dependency and does not require a
    Gaussian approximation.  Larger future campaigns use a deterministic
    normal approximation only after the exact enumeration becomes impractical.
    """
    differences = [value for value in values if math.isfinite(value)]
    count = len(differences)
    if count == 0:
        return math.nan, 'not_available'
    observed = abs(statistics.fmean(differences))
    tolerance = 1.0e-15 * max(1.0, observed)

    if count <= 20:
        extreme = 0
        assignment_count = 1 << count
        for mask in range(assignment_count):
            signed_sum = 0.0
            for index, value in enumerate(differences):
                signed_sum += value if mask & (1 << index) else -value
            if abs(signed_sum / count) + tolerance >= observed:
                extreme += 1
        return extreme / assignment_count, 'exact_sign_flip'

    # For an unexpectedly large future campaign, the standardized mean tends
    # toward a standard normal distribution under random sign assignment.
    sum_of_squares = sum(value * value for value in differences)
    if sum_of_squares <= 0.0:
        return 1.0, 'normal_sign_flip_approximation'
    z_value = abs(sum(differences)) / math.sqrt(sum_of_squares)
    p_value = math.erfc(z_value / math.sqrt(2.0))
    return min(1.0, max(0.0, p_value)), 'normal_sign_flip_approximation'


# Exact two-sided McNemar/binomial test for paired completion outcomes.
def exact_mcnemar_test(candidate_only, baseline_only):
    """Return the exact two-sided test for paired binary outcomes."""
    discordant = candidate_only + baseline_only
    if discordant == 0:
        return 1.0
    smaller = min(candidate_only, baseline_only)
    tail_count = sum(
        math.comb(discordant, successes)
        for successes in range(smaller + 1)
    )
    return min(1.0, 2.0 * tail_count / (2 ** discordant))


# Standardized paired effect: mean difference divided by its sample deviation.
def paired_effect_dz(values):
    """Return Cohen's dz based on the standard deviation of paired differences."""
    differences = [value for value in values if math.isfinite(value)]
    if len(differences) < 2:
        return math.nan
    deviation = statistics.stdev(differences)
    if deviation <= 0.0:
        return math.nan
    return statistics.fmean(differences) / deviation


# Bias-adjusted sample skewness used to diagnose asymmetric difference data.
def adjusted_fisher_pearson_skewness(values):
    """Small-sample adjusted skewness used only as a distribution diagnostic."""
    samples = [value for value in values if math.isfinite(value)]
    count = len(samples)
    if count < 3:
        return math.nan
    mean = statistics.fmean(samples)
    second = sum((value - mean) ** 2 for value in samples) / count
    if second <= 0.0:
        return 0.0
    third = sum((value - mean) ** 3 for value in samples) / count
    unadjusted = third / (second ** 1.5)
    return math.sqrt(count * (count - 1)) / (count - 2) * unadjusted


# Count observations outside Tukey 1.5-IQR fences as a robustness diagnostic.
def iqr_outlier_count(values):
    samples = [value for value in values if math.isfinite(value)]
    if len(samples) < 4:
        return 0
    first = quantile(samples, 0.25)
    third = quantile(samples, 0.75)
    spread = third - first
    lower = first - 1.5 * spread
    upper = third + 1.5 * spread
    return sum(value < lower or value > upper for value in samples)


# Map a result to its predeclared confirmatory/exploratory hypothesis family.
def inference_metadata(track, scenario, metric):
    """Analysis-plan primary families; all other results are exploratory."""
    if scenario == 'nominal' and metric == 'temporal_position_rmse_m':
        return 'confirmatory', 'nominal_temporal_accuracy'
    if scenario == 'nominal' and metric == 'cte_rmse_m':
        return 'confirmatory', 'nominal_spatial_accuracy'
    if (
            scenario in FINITE_PERTURBATION_SCENARIOS and
            metric == 'degradation_temporal_position_rmse_m'):
        return 'confirmatory', 'transient_temporal_robustness'
    if (
            scenario in FINITE_PERTURBATION_SCENARIOS and
            metric == 'degradation_cte_rmse_m'):
        return 'confirmatory', 'transient_spatial_robustness'
    if (
            scenario in FINITE_PERTURBATION_SCENARIOS and
            metric == 'window_recovery_fraction'):
        return 'confirmatory', 'transient_recovery'
    if (
            scenario == 'left_wheel_loss_persistent' and
            metric == 'active_tail_mean_abs_cte_m'):
        return 'confirmatory', 'persistent_spatial_robustness'
    if metric == 'normalized_command_activity':
        return 'confirmatory', 'control_effort'
    return 'exploratory', f'exploratory_{metric}'


# Apply Holm's step-down family-wise error correction within each family.
def apply_holm_correction(rows):
    """Add strong family-wise error control within each declared family."""
    families = defaultdict(list)
    for index, row in enumerate(rows):
        p_value = row['sign_flip_p_value']
        if math.isfinite(p_value):
            families[row['holm_family']].append((index, p_value))

    for members in families.values():
        ordered = sorted(members, key=lambda item: item[1])
        running_maximum = 0.0
        family_size = len(ordered)
        for rank, (row_index, p_value) in enumerate(ordered):
            adjusted = min(1.0, (family_size - rank) * p_value)
            running_maximum = max(running_maximum, adjusted)
            rows[row_index]['holm_adjusted_p_value'] = running_maximum
            rows[row_index]['holm_family_size'] = family_size
            rows[row_index]['holm_significant_0p05'] = (
                1 if running_maximum < 0.05 else 0
            )


# Classify a confidence interval against the engineering relevance band.
def practical_interpretation(low, high, threshold):
    if not all(math.isfinite(value) for value in (low, high, threshold)):
        return 'not_predeclared'
    if high < -threshold:
        return 'candidate_better_beyond_practical_threshold'
    if low > threshold:
        return 'baseline_better_beyond_practical_threshold'
    if low >= -threshold and high <= threshold:
        return 'confidence_interval_inside_practical_band'
    return 'confidence_interval_overlaps_practical_threshold'


# Decode controller, track, scenario, and repetition from a standardized path.
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


# Traverse all run summaries and construct the common unaggregated trial table.
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
            trial['fixed_observation_duration'] = metadata.get(
                'fixed_observation_duration', '0.0'
            )
            trial['observation_horizon_reached'] = metadata.get(
                'observation_horizon_reached', '0'
            )
            trial['fault_signature'] = '|'.join(metadata.get(key, '') for key in (
                'fault_domain', 'fault_start', 'fault_duration',
                'fault_start_delays', 'fault_persistent',
                'command_fault_enabled', 'feedback_fault_enabled',
                'angular_bias', 'left_wheel_effectiveness',
                'right_wheel_effectiveness', 'command_delay',
                'odometry_x_bias', 'odometry_y_bias', 'odometry_yaw_bias',
                'position_noise_stddev', 'yaw_noise_stddev', 'noise_seed',
                'fixed_observation_duration',
            ))
            trials.append(trial)
    if not trials:
        raise RuntimeError(f'no comparison summaries found below {root}')
    return trials


# Subtract each controller's seed-matched nominal outcome from its disturbed
# outcome so robustness is not confused with different nominal accuracy.
def add_nominal_degradation_metrics(trials):
    """Attach within-controller, same-seed degradation relative to nominal."""
    nominal = {}
    for trial in trials:
        if trial['scenario'] != 'nominal':
            continue
        key = (
            trial['controller_family'], trial['gazebo_seed'], trial['track'],
        )
        if key in nominal:
            raise RuntimeError(f'duplicate nominal trial identity: {key}')
        nominal[key] = trial

    for trial in trials:
        for output_metric in DEGRADATION_SOURCES:
            trial[output_metric] = math.nan
        if trial['scenario'] == 'nominal':
            continue
        key = (
            trial['controller_family'], trial['gazebo_seed'], trial['track'],
        )
        baseline = nominal.get(key)
        if baseline is None:
            # A robustness track without a matched nominal run cannot support a
            # degradation claim; absolute metrics remain available.
            continue
        for output_metric, source_metric in DEGRADATION_SOURCES.items():
            perturbed_value = finite(trial, source_metric)
            nominal_value = finite(baseline, source_metric)
            if math.isfinite(perturbed_value) and math.isfinite(nominal_value):
                trial[output_metric] = perturbed_value - nominal_value


# Write tidy raw trial values for independent reanalysis.
def write_all_trials(root, trials):
    stable_fields = [
        'repetition', 'gazebo_seed', 'controller_family', 'track', 'scenario',
        'fault_domain', 'fault_window_count', 'recovered_window_count',
        'persistent_fault', 'track_complete',
        'analysis_observation_duration_s', *METRICS, 'noise_seed',
        'controller_config_sha256', 'reference_config_sha256', 'track_sha256',
    ]
    with (root / 'all_runs.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=stable_fields)
        writer.writeheader()
        for trial in trials:
            writer.writerow({key: trial.get(key, '') for key in stable_fields})


# Compute descriptive statistics for every controller/track/scenario group.
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
            f'{metric}_median', f'{metric}_q1', f'{metric}_q3',
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
                f'{metric}_median': quantile(valid, 0.5),
                f'{metric}_q1': quantile(valid, 0.25),
                f'{metric}_q3': quantile(valid, 0.75),
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


# Form seed-matched controller contrasts, perform paired inference, attach
# practical/effect diagnostics, then apply family-wise Holm correction.
def write_paired_differences(root, trials, practical_thresholds=None):
    if practical_thresholds is None:
        practical_thresholds = PRACTICAL_THRESHOLDS
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
                candidate_values = []
                baseline_values = []
                for candidate_row, baseline_row in matched:
                    candidate_value = finite(candidate_row, metric)
                    baseline_value = finite(baseline_row, metric)
                    if math.isfinite(candidate_value) and math.isfinite(baseline_value):
                        candidate_values.append(candidate_value)
                        baseline_values.append(baseline_value)
                        differences.append(candidate_value - baseline_value)
                mean, deviation, low, high = confidence_interval(differences)
                p_value, test_method = exact_sign_flip_test(differences)
                role, family = inference_metadata(track, scenario, metric)
                higher_is_better = metric in HIGHER_IS_BETTER
                threshold = practical_thresholds.get(metric, math.nan)
                candidate_mean = (
                    statistics.fmean(candidate_values)
                    if candidate_values else math.nan
                )
                baseline_mean = (
                    statistics.fmean(baseline_values)
                    if baseline_values else math.nan
                )
                relative_difference = (
                    100.0 * mean / abs(baseline_mean)
                    if math.isfinite(mean) and math.isfinite(baseline_mean) and
                    abs(baseline_mean) > 1.0e-15 else math.nan
                )
                if not math.isfinite(mean) or abs(mean) <= 1.0e-15:
                    mean_favors = 'tie_or_unavailable'
                elif (mean > 0.0) == higher_is_better:
                    mean_favors = 'candidate'
                else:
                    mean_favors = 'baseline'
                practical_low, practical_high = low, high
                if higher_is_better:
                    practical_low, practical_high = -high, -low
                output.append({
                    'candidate': candidate,
                    'baseline': baseline,
                    'track': track,
                    'scenario': scenario,
                    'metric': metric,
                    'inference_role': role,
                    'holm_family': family,
                    'higher_is_better': 1 if higher_is_better else 0,
                    'paired_samples': len(differences),
                    'candidate_mean': candidate_mean,
                    'baseline_mean': baseline_mean,
                    'mean_candidate_minus_baseline': mean,
                    'relative_difference_percent': relative_difference,
                    'stddev_difference': deviation,
                    'median_difference': quantile(differences, 0.5),
                    'difference_q1': quantile(differences, 0.25),
                    'difference_q3': quantile(differences, 0.75),
                    'ci95_low': low,
                    'ci95_high': high,
                    'ci95_excludes_zero': (
                        1 if math.isfinite(low) and math.isfinite(high) and
                        (high < 0.0 or low > 0.0) else 0
                    ),
                    'cohen_dz': paired_effect_dz(differences),
                    'difference_skewness': adjusted_fisher_pearson_skewness(
                        differences
                    ),
                    'iqr_outlier_count': iqr_outlier_count(differences),
                    'test_method': test_method,
                    'sign_flip_p_value': p_value,
                    'holm_family_size': 0,
                    'holm_adjusted_p_value': math.nan,
                    'holm_significant_0p05': 0,
                    'mean_favors': mean_favors,
                    'practical_threshold': threshold,
                    'practical_interpretation': practical_interpretation(
                        practical_low, practical_high, threshold
                    ),
                })

    apply_holm_correction(output)
    fields = list(output[0]) if output else [
        'candidate', 'baseline', 'track', 'scenario', 'metric',
        'inference_role', 'holm_family', 'higher_is_better',
        'paired_samples', 'candidate_mean', 'baseline_mean',
        'mean_candidate_minus_baseline', 'relative_difference_percent',
        'stddev_difference', 'median_difference', 'difference_q1',
        'difference_q3', 'ci95_low', 'ci95_high', 'ci95_excludes_zero',
        'cohen_dz', 'difference_skewness', 'iqr_outlier_count',
        'test_method', 'sign_flip_p_value', 'holm_family_size',
        'holm_adjusted_p_value', 'holm_significant_0p05', 'mean_favors',
        'practical_threshold', 'practical_interpretation',
    ]
    with (root / 'paired_differences.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in output:
            writer.writerow({key: format_value(value) for key, value in row.items()})
    return output


# Look up the predeclared family and role for binary completion inference.
def completion_inference_metadata(protocol, scenario):
    """Return the predeclared or legacy role of a completion comparison."""
    role = protocol.get('completion_analysis_role', 'exploratory_post_hoc')
    confirmatory_scenarios = set(
        protocol.get('completion_confirmatory_scenarios', '').split()
    )
    if role == 'predeclared_confirmatory':
        if scenario in confirmatory_scenarios:
            return 'confirmatory_primary', 'confirmatory_completion'
        return 'exploratory_secondary', 'exploratory_completion_secondary'
    return 'exploratory_post_hoc', 'exploratory_completion'


# Compare paired completion outcomes using discordant run pairs.
def write_completion_comparison(root, trials, protocol):
    """Write a paired analysis of the binary trajectory endpoint."""
    indexed = {
        (
            trial['controller_family'], trial['gazebo_seed'],
            trial['track'], trial['scenario'],
        ): int(float(trial['track_complete']))
        for trial in trials
    }
    controllers = {trial['controller_family'] for trial in trials}
    conditions = sorted({
        (trial['gazebo_seed'], trial['track'], trial['scenario'])
        for trial in trials
    })
    output = []
    for candidate, baseline in (
            ('lqr', 'pid'), ('mpc', 'pid'), ('mpc', 'lqr')):
        if candidate not in controllers or baseline not in controllers:
            continue
        condition_names = sorted({
            (track, scenario) for _, track, scenario in conditions
        })
        for track, scenario in condition_names:
            paired = []
            for seed, condition_track, condition_scenario in conditions:
                if (condition_track, condition_scenario) != (track, scenario):
                    continue
                candidate_value = indexed.get(
                    (candidate, seed, track, scenario)
                )
                baseline_value = indexed.get(
                    (baseline, seed, track, scenario)
                )
                if candidate_value is not None and baseline_value is not None:
                    paired.append((candidate_value, baseline_value))

            candidate_only = sum(a == 1 and b == 0 for a, b in paired)
            baseline_only = sum(a == 0 and b == 1 for a, b in paired)
            both_complete = sum(a == 1 and b == 1 for a, b in paired)
            both_incomplete = sum(a == 0 and b == 0 for a, b in paired)
            candidate_completed = sum(a for a, _ in paired)
            baseline_completed = sum(b for _, b in paired)
            count = len(paired)
            p_value = exact_mcnemar_test(candidate_only, baseline_only)
            inference_role, holm_family = completion_inference_metadata(
                protocol, scenario
            )
            output.append({
                'inference_role': inference_role,
                'holm_family': holm_family,
                'candidate': candidate,
                'baseline': baseline,
                'track': track,
                'scenario': scenario,
                'paired_samples': count,
                'candidate_completed': candidate_completed,
                'baseline_completed': baseline_completed,
                'candidate_completion_rate': (
                    candidate_completed / count if count else math.nan
                ),
                'baseline_completion_rate': (
                    baseline_completed / count if count else math.nan
                ),
                'completion_rate_difference': (
                    (candidate_completed - baseline_completed) / count
                    if count else math.nan
                ),
                'candidate_only_completed': candidate_only,
                'baseline_only_completed': baseline_only,
                'both_completed': both_complete,
                'both_incomplete': both_incomplete,
                'discordant_pairs': candidate_only + baseline_only,
                'test_method': 'exact_mcnemar',
                'sign_flip_p_value': p_value,
                'holm_family_size': 0,
                'holm_adjusted_p_value': math.nan,
                'holm_significant_0p05': 0,
            })

    # Each inference family is corrected independently. Legacy datasets retain
    # one post-hoc family, while a predeclared validation keeps its primary
    # comparisons separate from secondary nominal diagnostics.
    apply_holm_correction(output)
    fields = [
        'inference_role', 'holm_family', 'candidate', 'baseline', 'track',
        'scenario', 'paired_samples', 'candidate_completed',
        'baseline_completed', 'candidate_completion_rate',
        'baseline_completion_rate', 'completion_rate_difference',
        'candidate_only_completed', 'baseline_only_completed',
        'both_completed', 'both_incomplete', 'discordant_pairs',
        'test_method', 'exact_mcnemar_p_value', 'holm_family_size',
        'holm_adjusted_p_value', 'holm_significant_0p05',
    ]
    with (root / 'completion_comparison.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in output:
            exported = dict(row)
            exported['exact_mcnemar_p_value'] = exported.pop(
                'sign_flip_p_value'
            )
            writer.writerow({
                key: format_value(exported[key]) for key in fields
            })
    return output


# Verify expected files, seeds, repetitions, immutable configurations, timing,
# and pairing; return every protocol deviation rather than silently filtering.
def audit_protocol(root, trials, protocol):
    issues = []
    if not (root / 'protocol_source.tar.gz').is_file():
        issues.append('archived runtime source is missing')
    expected_controllers = set(protocol['controllers'].split())
    expected_observation_duration = float(
        protocol.get('fixed_observation_duration_s', '0.0')
    )
    completion_role = protocol.get(
        'completion_analysis_role', 'exploratory_post_hoc'
    )
    confirmatory_completion_scenarios = set(
        protocol.get('completion_confirmatory_scenarios', '').split()
    )
    if completion_role not in {
            'exploratory_post_hoc', 'predeclared_confirmatory'}:
        issues.append(f'unknown completion analysis role: {completion_role}')
    if completion_role == 'predeclared_confirmatory':
        if not confirmatory_completion_scenarios:
            issues.append('confirmatory completion scenario set is empty')
        declared_scenarios = set(protocol['robustness_scenarios'].split())
        if not confirmatory_completion_scenarios <= declared_scenarios:
            issues.append(
                'confirmatory completion scenarios are absent from the '
                'campaign matrix'
            )
        if expected_observation_duration <= 0.0:
            issues.append(
                'confirmatory completion lacks a fixed observation horizon'
            )
        if protocol.get('completion_test') != 'exact_two_sided_mcnemar':
            issues.append('confirmatory completion test is not exact McNemar')
        if protocol.get('completion_multiplicity') != (
                'holm_within_inference_family'):
            issues.append('confirmatory completion multiplicity rule changed')
        if protocol.get('completion_familywise_alpha') != '0.05':
            issues.append('confirmatory completion familywise alpha changed')
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
        runtime_duration = float(trial['fixed_observation_duration'])
        if not math.isclose(
                runtime_duration, expected_observation_duration,
                rel_tol=0.0, abs_tol=1.0e-9):
            issues.append(
                f"{trial['controller_family']}:{trial['track']}:"
                f"{trial['scenario']}: fixed observation duration "
                f'{runtime_duration} differs from protocol '
                f'{expected_observation_duration}'
            )
        if (expected_observation_duration > 0.0 and
                trial['observation_horizon_reached'] != '1'):
            issues.append(
                f"{trial['controller_family']}:{trial['track']}:"
                f"{trial['scenario']}: fixed observation horizon not reached"
            )

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
        analysis_durations = [
            finite(row, 'analysis_observation_duration_s') for row in rows
        ]
        if (any(not math.isfinite(value) for value in analysis_durations) or
                max(analysis_durations) - min(analysis_durations) > 0.05):
            issues.append(
                f'{condition}: controller analysis horizons differ: '
                f'{analysis_durations}'
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

    if completion_role == 'predeclared_confirmatory':
        completion_parameter_map = {
            'goal_tolerance': 'completion_position_tolerance_m',
            'goal_heading_tolerance': 'completion_heading_tolerance_rad',
        }
        for controller_parameter, protocol_parameter in (
                completion_parameter_map.items()):
            try:
                expected_value = float(protocol[protocol_parameter])
                controller_values = {
                    float(archived_values[controller][controller_parameter])
                    for controller in expected_controllers
                }
            except (KeyError, TypeError, ValueError):
                issues.append(
                    f'completion criterion {protocol_parameter} is missing '
                    'or invalid'
                )
                continue
            if controller_values != {expected_value}:
                issues.append(
                    f'completion criterion {protocol_parameter} differs '
                    f'from controller values {sorted(controller_values)}'
                )

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
        # Launch files load the reference YAML after the controller YAML.
        # Therefore this archived file, whose runtime hash must be common to
        # all trials above, is the effective source of truth. Controller YAML
        # values are merely standalone fallbacks and may legitimately differ
        # during an explicitly overridden stress campaign.

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


# Summarize the number of tested and Holm-rejected hypotheses per family.
def write_hypothesis_family_summary(root, paired_rows):
    families = defaultdict(list)
    for row in paired_rows:
        families[(row['inference_role'], row['holm_family'])].append(row)

    fields = [
        'inference_role', 'holm_family', 'tests', 'valid_tests',
        'holm_significant_0p05', 'metrics', 'conditions',
    ]
    output = []
    for (role, family), rows in sorted(families.items()):
        valid = [
            row for row in rows if math.isfinite(row['sign_flip_p_value'])
        ]
        output.append({
            'inference_role': role,
            'holm_family': family,
            'tests': len(rows),
            'valid_tests': len(valid),
            'holm_significant_0p05': sum(
                row['holm_significant_0p05'] for row in valid
            ),
            'metrics': ';'.join(sorted({row['metric'] for row in rows})),
            'conditions': ';'.join(sorted({
                f"{row['track']}:{row['scenario']}" for row in rows
            })),
        })

    with (root / 'hypothesis_families.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(output)
    return output


# Extract only predeclared confirmatory rows, keeping exploratory results distinct.
def write_confirmatory_results(root, paired_rows):
    fields = [
        'holm_family', 'candidate', 'baseline', 'track', 'scenario', 'metric',
        'higher_is_better', 'paired_samples', 'candidate_mean', 'baseline_mean',
        'mean_candidate_minus_baseline', 'relative_difference_percent',
        'ci95_low', 'ci95_high', 'cohen_dz', 'sign_flip_p_value',
        'holm_adjusted_p_value', 'holm_significant_0p05', 'mean_favors',
        'practical_threshold', 'practical_interpretation',
    ]
    selected = [
        row for row in paired_rows
        if row['inference_role'] == 'confirmatory' and
        math.isfinite(row['sign_flip_p_value'])
    ]
    with (root / 'confirmatory_results.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in selected:
            writer.writerow({key: format_value(row[key]) for key in fields})
    return selected


# Produce a human-readable synopsis beside the machine-readable tables.
def write_text_summary(root, trials, grouped, paired_rows, families, issues):
    confirmatory = [
        row for row in paired_rows
        if row['inference_role'] == 'confirmatory' and
        math.isfinite(row['sign_flip_p_value'])
    ]
    with (root / 'summary.txt').open('w', encoding='utf-8') as stream:
        stream.write('PID--TVLQR--MPC PAIRED COMPARISON\n')
        stream.write(f'Individual scenario runs: {len(trials)}\n')
        stream.write(f'Aggregated conditions: {len(grouped)}\n')
        stream.write(f'Protocol audit issues: {len(issues)}\n')
        stream.write(f'Hypothesis families: {len(families)}\n')
        stream.write(f'Valid confirmatory pairwise tests: {len(confirmatory)}\n')
        stream.write(
            'Holm-significant confirmatory tests at 0.05: '
            f'{sum(row["holm_significant_0p05"] for row in confirmatory)}\n'
        )
        stream.write(
            'Confidence intervals use paired differences and the two-sided '
            'Student-t 95% critical value. Hypothesis tests use the exact '
            'two-sided paired sign-flip distribution for n <= 20, with Holm '
            'family-wise correction inside each analysis-plan family.\n'
        )
        stream.write(
            'A negative candidate-minus-baseline difference favors the candidate '
            'for every metric except window_recovery_fraction. Statistical '
            'significance does not by itself establish engineering relevance.\n'
        )
        stream.write(
            'Perturbation degradation equals perturbed minus matched nominal '
            'performance for the same controller, track, and Gazebo seed.\n'
        )


# Orchestrate loading, auditing, descriptive analysis, paired inference,
# completion comparison, multiple-testing correction, and final exit status.
def main():
    parser = argparse.ArgumentParser(
        description='Analyze the paired PID--TVLQR--MPC campaign.'
    )
    parser.add_argument('result_directory', type=Path)
    parser.add_argument(
        '--position-practical-threshold-m', type=float, default=0.01,
        help='Smallest practically relevant difference for position errors.',
    )
    parser.add_argument(
        '--heading-practical-threshold-rad', type=float, default=0.02,
        help='Smallest practically relevant difference for heading errors.',
    )
    arguments = parser.parse_args()
    if (arguments.position_practical_threshold_m < 0.0 or
            arguments.heading_practical_threshold_rad < 0.0):
        parser.error('practical thresholds must be non-negative')

    practical_thresholds = practical_threshold_map(
        arguments.position_practical_threshold_m,
        arguments.heading_practical_threshold_rad,
    )

    root = arguments.result_directory
    protocol = read_protocol(root)
    trials = load_trials(root)
    add_nominal_degradation_metrics(trials)
    write_all_trials(root, trials)
    grouped = write_group_summary(root, trials)
    paired_rows = write_paired_differences(
        root, trials, practical_thresholds
    )
    write_completion_comparison(root, trials, protocol)
    families = write_hypothesis_family_summary(root, paired_rows)
    write_confirmatory_results(root, paired_rows)
    issues = audit_protocol(root, trials, protocol)
    write_text_summary(root, trials, grouped, paired_rows, families, issues)
    print(f'Wrote {root / "all_runs.csv"}')
    print(f'Wrote {root / "group_summary.csv"}')
    print(f'Wrote {root / "paired_differences.csv"}')
    print(f'Wrote {root / "hypothesis_families.csv"}')
    print(f'Wrote {root / "confirmatory_results.csv"}')
    print(f'Wrote {root / "completion_comparison.csv"}')
    print(f'Wrote {root / "protocol_audit.txt"}')
    if issues:
        raise SystemExit(f'protocol audit failed with {len(issues)} issue(s)')


if __name__ == '__main__':
    # Unit tests may import helpers without triggering filesystem analysis.
    main()
