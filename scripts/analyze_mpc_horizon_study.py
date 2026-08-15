#!/usr/bin/env python3
"""Summarize tracking, command activity, and QP timing by MPC horizon."""

import csv
import math
import sys
from pathlib import Path


def read_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def finite_number(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def percentile(values, probability):
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    index = max(0, math.ceil(probability * len(finite)) - 1)
    return finite[index]


def normalized_command_activity(rows):
    integral = 0.0
    previous_time = None
    previous_value = None
    for row in rows:
        time = finite_number(row, 'time')
        linear = finite_number(row, 'applied_linear_command')
        angular = finite_number(row, 'applied_angular_command')
        if not all(math.isfinite(value) for value in (time, linear, angular)):
            continue
        value = linear * linear + (angular / 1.5) ** 2
        if previous_time is not None and time > previous_time:
            integral += 0.5 * (previous_value + value) * (
                time - previous_time
            )
        previous_time = time
        previous_value = value
    return integral


def summarize(study_dir):
    nominal = study_dir / 'nominal'
    controller_rows = read_rows(nominal / 'controller.csv')
    command_rows = read_rows(nominal / 'applied_commands.csv')
    suite_rows = read_rows(study_dir / 'summary.csv')
    suite_row = next(row for row in suite_rows if row['scenario'] == 'nominal')
    if not controller_rows:
        raise RuntimeError(f'empty controller log: {study_dir}')

    solve_times = [
        finite_number(row, 'mpc_solve_time_seconds')
        for row in controller_rows
    ]
    iterations = [
        finite_number(row, 'mpc_iterations') for row in controller_rows
    ]
    solved_count = sum(
        int(finite_number(row, 'mpc_solved')) for row in controller_rows
    )
    horizon = int(finite_number(controller_rows[0], 'mpc_horizon_steps'))
    finite_solve_times = [
        value for value in solve_times if math.isfinite(value)
    ]

    return {
        'horizon_steps': horizon,
        'horizon_seconds': horizon / 30.0,
        'track_complete': int(suite_row['track_complete']),
        'cte_rmse_m': float(suite_row['cte_rmse_m']),
        'heading_rmse_rad': float(suite_row['heading_rmse_rad']),
        'temporal_position_rmse_m': float(
            suite_row['temporal_position_rmse_m']
        ),
        'peak_abs_cte_m': float(suite_row['peak_abs_cte_m']),
        'normalized_command_activity': normalized_command_activity(
            command_rows
        ),
        'qp_samples': len(controller_rows),
        'qp_failures': len(controller_rows) - solved_count,
        'mean_qp_time_s': sum(finite_solve_times) / len(finite_solve_times),
        'p95_qp_time_s': percentile(solve_times, 0.95),
        'max_qp_time_s': max(finite_solve_times),
        'max_iterations': int(max(iterations)),
    }


def format_value(value):
    if isinstance(value, int):
        return str(value)
    return f'{value:.9f}'


def main():
    if len(sys.argv) != 2:
        raise SystemExit(
            'usage: analyze_mpc_horizon_study.py RESULT_DIRECTORY'
        )
    root = Path(sys.argv[1])
    study_dirs = sorted(
        path for path in root.glob('horizon_*') if path.is_dir()
    )
    if not study_dirs:
        raise SystemExit(f'no horizon result directories found in {root}')

    rows = sorted((summarize(path) for path in study_dirs),
                  key=lambda row: row['horizon_steps'])
    summary_path = root / 'horizon_summary.csv'
    with summary_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            writer.writerow({key: format_value(value)
                             for key, value in row.items()})

    text_path = root / 'horizon_summary.txt'
    with text_path.open('w', encoding='utf-8') as stream:
        stream.write('MPC HORIZON COMMISSIONING STUDY\n')
        stream.write(
            'Only the prediction horizon changes; Q, R, actuator limits, '
            'reference, track, and Gazebo seed remain fixed.\n\n'
        )
        for row in rows:
            stream.write(
                f"N={row['horizon_steps']:d} "
                f"({row['horizon_seconds']:.2f} s): "
                f"complete={row['track_complete']:d}, "
                f"CTE_RMS={row['cte_rmse_m']:.6f} m, "
                f"heading_RMS={row['heading_rmse_rad']:.6f} rad, "
                f"command_activity={row['normalized_command_activity']:.6f}, "
                f"QP_mean={1000.0 * row['mean_qp_time_s']:.3f} ms, "
                f"QP_p95={1000.0 * row['p95_qp_time_s']:.3f} ms, "
                f"QP_max={1000.0 * row['max_qp_time_s']:.3f} ms, "
                f"failures={row['qp_failures']:d}\n"
            )

    print(f'Wrote {summary_path}')
    print(f'Wrote {text_path}')


if __name__ == '__main__':
    main()
