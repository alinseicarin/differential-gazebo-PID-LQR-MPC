#!/usr/bin/env python3
"""Summarize wheel/body slip measurements from a speed-limit pilot."""

import argparse
import csv
import math
import re
import statistics
from pathlib import Path


def finite(row, key):
    """Return a finite float from a CSV row, or None when unavailable."""
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def rms(values):
    """Calculate root-mean-square for a non-empty numeric sequence."""
    return math.sqrt(sum(value * value for value in values) / len(values))


def repetition_from_path(path):
    """Extract the benchmark repetition from a result filename."""
    match = re.search(r'_run([0-9]+)_ground_truth[.]csv$', path.name)
    if match is None:
        raise RuntimeError(f'{path}: cannot determine repetition')
    return int(match.group(1))


def summarize_file(path, requested_speed, maximum_linear, maximum_angular):
    """Summarize one ground-truth trajectory CSV."""
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))

    controller_path = path.with_name(path.name.replace('_ground_truth', ''))
    with controller_path.open(newline='', encoding='utf-8') as stream:
        controller_rows = list(csv.DictReader(stream))

    required = {
        'reference_linear_velocity',
        'truth_linear_velocity',
        'true_cross_track_error',
        'center_longitudinal_slip_ratio',
        'yaw_velocity_discrepancy_ratio',
        'sideslip_angle_rad',
        'wheel_slip_sample_valid',
    }
    if not rows or not required.issubset(rows[0]):
        missing = sorted(required.difference(rows[0] if rows else {}))
        raise RuntimeError(f'{path}: missing slip columns: {missing}')

    moving = []
    valid = []
    plateau = []
    for row in rows:
        reference_speed = finite(row, 'reference_linear_velocity')
        if reference_speed is None or reference_speed < 0.1:
            continue
        moving.append(row)
        if row.get('wheel_slip_sample_valid') != '1':
            continue
        valid.append(row)
        if reference_speed >= 0.8 * requested_speed:
            plateau.append(row)

    if not moving or not valid:
        raise RuntimeError(f'{path}: no moving samples with valid wheel state')

    def values(source, key, absolute=False):
        result = []
        for row in source:
            value = finite(row, key)
            if value is not None:
                result.append(abs(value) if absolute else value)
        return result

    center_slip = values(valid, 'center_longitudinal_slip_ratio', True)
    yaw_slip = values(valid, 'yaw_velocity_discrepancy_ratio', True)
    sideslip = values(valid, 'sideslip_angle_rad', True)
    cte = values(moving, 'true_cross_track_error')
    reference = values(moving, 'reference_linear_velocity')
    truth = values(moving, 'truth_linear_velocity')
    ages = values(valid, 'joint_state_age', True)
    plateau_center = values(plateau, 'center_longitudinal_slip_ratio', True)
    plateau_sideslip = values(plateau, 'sideslip_angle_rad', True)
    active_controller = [
        row for row in controller_rows
        if (finite(row, 'reference_linear_velocity') or 0.0) >= 0.1]
    linear_commands = values(active_controller, 'linear_command', True)
    angular_commands = values(active_controller, 'angular_command', True)
    safety_stops = [
        row.get('translation_safety_stop') == '1'
        for row in active_controller]

    return {
        'requested_speed_m_s': requested_speed,
        'repetition': repetition_from_path(path),
        'source_file': str(path),
        'moving_samples': len(moving),
        'valid_slip_samples': len(valid),
        'valid_slip_fraction': len(valid) / len(moving),
        'plateau_samples': len(plateau),
        'max_reference_speed_m_s': max(reference),
        'max_truth_speed_m_s': max(truth),
        'max_abs_linear_command_m_s': max(linear_commands),
        'max_abs_angular_command_rad_s': max(angular_commands),
        'linear_saturation_fraction': sum(
            value >= maximum_linear - 1.0e-6
            for value in linear_commands) / len(linear_commands),
        'angular_saturation_fraction': sum(
            value >= maximum_angular - 1.0e-6
            for value in angular_commands) / len(angular_commands),
        'translation_safety_stop_fraction': (
            sum(safety_stops) / len(safety_stops)),
        'cte_rmse_m': rms(cte),
        'peak_abs_cte_m': max(abs(value) for value in cte),
        'center_slip_rms': rms(center_slip),
        'peak_abs_center_slip': max(center_slip),
        'center_slip_over_5pct_fraction': sum(
            value > 0.05 for value in center_slip) / len(center_slip),
        'yaw_discrepancy_rms': rms(yaw_slip),
        'peak_abs_yaw_discrepancy': max(yaw_slip),
        'sideslip_rms_rad': rms(sideslip),
        'peak_abs_sideslip_rad': max(sideslip),
        'sideslip_over_3deg_fraction': sum(
            value > math.radians(3.0) for value in sideslip) / len(sideslip),
        'plateau_center_slip_rms': (
            rms(plateau_center) if plateau_center else math.nan),
        'plateau_peak_abs_center_slip': (
            max(plateau_center) if plateau_center else math.nan),
        'plateau_sideslip_rms_rad': (
            rms(plateau_sideslip) if plateau_sideslip else math.nan),
        'plateau_peak_abs_sideslip_rad': (
            max(plateau_sideslip) if plateau_sideslip else math.nan),
        'max_abs_joint_state_age_s': max(ages),
    }


def speed_from_directory(path):
    """Decode speed_0p8-style result-directory names."""
    name = path.parent.name
    if not name.startswith('speed_'):
        raise RuntimeError(f'{path}: parent directory must start with speed_')
    return float(name[len('speed_'):].replace('p', '.'))


def main():
    """Find pilot CSV files and write one deterministic summary."""
    parser = argparse.ArgumentParser()
    parser.add_argument('result_directory', type=Path)
    parser.add_argument('--maximum-linear', type=float, default=1.0)
    parser.add_argument('--maximum-angular', type=float, default=1.5)
    arguments = parser.parse_args()

    paths = sorted(arguments.result_directory.glob(
        'speed_*/cascade_circle_run*_ground_truth.csv'))
    if not paths:
        raise SystemExit('No pilot ground-truth CSV files found')
    summaries = [
        summarize_file(
            path, speed_from_directory(path), arguments.maximum_linear,
            arguments.maximum_angular
        )
        for path in paths
    ]
    summaries.sort(key=lambda row: row['requested_speed_m_s'])

    destination = arguments.result_directory / 'speed_limit_per_run.csv'
    with destination.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    grouped = []
    for speed in sorted({row['requested_speed_m_s'] for row in summaries}):
        group = [row for row in summaries
                 if row['requested_speed_m_s'] == speed]

        def mean(key):
            return statistics.fmean(row[key] for row in group)

        def sample_sd(key):
            values = [row[key] for row in group]
            return statistics.stdev(values) if len(values) > 1 else math.nan

        grouped.append({
            'requested_speed_m_s': speed,
            'repetitions': len(group),
            'minimum_valid_slip_fraction': min(
                row['valid_slip_fraction'] for row in group),
            'mean_max_truth_speed_m_s': mean('max_truth_speed_m_s'),
            'mean_linear_saturation_fraction': mean(
                'linear_saturation_fraction'),
            'mean_angular_saturation_fraction': mean(
                'angular_saturation_fraction'),
            'maximum_translation_safety_stop_fraction': max(
                row['translation_safety_stop_fraction'] for row in group),
            'mean_cte_rmse_m': mean('cte_rmse_m'),
            'sd_cte_rmse_m': sample_sd('cte_rmse_m'),
            'maximum_peak_abs_cte_m': max(
                row['peak_abs_cte_m'] for row in group),
            'mean_plateau_center_slip_rms': mean(
                'plateau_center_slip_rms'),
            'sd_plateau_center_slip_rms': sample_sd(
                'plateau_center_slip_rms'),
            'maximum_plateau_peak_abs_center_slip': max(
                row['plateau_peak_abs_center_slip'] for row in group),
            'mean_plateau_sideslip_rms_rad': mean(
                'plateau_sideslip_rms_rad'),
            'sd_plateau_sideslip_rms_rad': sample_sd(
                'plateau_sideslip_rms_rad'),
            'maximum_plateau_peak_abs_sideslip_rad': max(
                row['plateau_peak_abs_sideslip_rad'] for row in group),
            'maximum_abs_joint_state_age_s': max(
                row['max_abs_joint_state_age_s'] for row in group),
        })

    group_destination = arguments.result_directory / 'speed_limit_summary.csv'
    with group_destination.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(grouped[0]))
        writer.writeheader()
        writer.writerows(grouped)

    print(destination)
    print(group_destination)
    for row in grouped:
        print(
            f"v={row['requested_speed_m_s']:.3f} m/s "
            f"n={row['repetitions']} "
            f"truth_max_mean={row['mean_max_truth_speed_m_s']:.3f} m/s "
            f"CTE_RMS_mean={1000.0 * row['mean_cte_rmse_m']:.1f} mm "
            f"plateau_slip_RMS_mean="
            f"{100.0 * row['mean_plateau_center_slip_rms']:.2f}% "
            f"plateau_sideslip_RMS_mean="
            f"{math.degrees(row['mean_plateau_sideslip_rms_rad']):.2f} deg "
            f"linear_sat={100.0 * row['mean_linear_saturation_fraction']:.1f}% "
            f"angular_sat="
            f"{100.0 * row['mean_angular_saturation_fraction']:.1f}%")


if __name__ == '__main__':
    main()
