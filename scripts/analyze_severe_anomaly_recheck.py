#!/usr/bin/env python3
"""Summarize post-hoc repeats of selected severe-campaign anomalies."""

import csv
import math
import statistics
import sys
from collections import defaultdict
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


def main():
    if len(sys.argv) != 2:
        raise SystemExit(
            'usage: analyze_severe_anomaly_recheck.py RESULT_DIRECTORY'
        )
    root = Path(sys.argv[1])
    details = []
    for path in sorted(root.glob('repeat_*/*_seed_*/summary.csv')):
        repeat = int(path.parent.parent.name.removeprefix('repeat_'))
        family, seed = path.parent.name.split('_seed_', 1)
        for row in read_rows(path):
            details.append({
                'repeat': repeat,
                'gazebo_seed': int(seed),
                'controller_family': family,
                'scenario': row['scenario'],
                'track_complete': int(row['track_complete']),
                'cte_rmse_m': number(row, 'cte_rmse_m'),
                'temporal_position_rmse_m': number(
                    row, 'temporal_position_rmse_m'
                ),
                'active_tail_mean_abs_cte_m': number(
                    row, 'active_tail_mean_abs_cte_m'
                ),
            })
    if not details:
        raise SystemExit(f'no anomaly recheck summaries found below {root}')

    detail_fields = list(details[0])
    with (root / 'recheck_runs.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=detail_fields)
        writer.writeheader()
        writer.writerows(details)

    groups = defaultdict(list)
    for row in details:
        groups[(
            row['gazebo_seed'], row['controller_family'], row['scenario']
        )].append(row)
    summary_fields = [
        'gazebo_seed', 'controller_family', 'scenario', 'repetitions',
        'completed', 'completion_rate', 'cte_rmse_m_median',
        'temporal_position_rmse_m_median',
        'active_tail_mean_abs_cte_m_median',
    ]
    summary = []
    for (seed, family, scenario), rows in sorted(groups.items()):
        completed = sum(row['track_complete'] for row in rows)

        def median(metric):
            values = [row[metric] for row in rows]
            values = [value for value in values if math.isfinite(value)]
            return statistics.median(values) if values else math.nan

        summary.append({
            'gazebo_seed': seed,
            'controller_family': family,
            'scenario': scenario,
            'repetitions': len(rows),
            'completed': completed,
            'completion_rate': completed / len(rows),
            'cte_rmse_m_median': median('cte_rmse_m'),
            'temporal_position_rmse_m_median': median(
                'temporal_position_rmse_m'
            ),
            'active_tail_mean_abs_cte_m_median': median(
                'active_tail_mean_abs_cte_m'
            ),
        })

    with (root / 'recheck_summary.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(summary)

    for row in summary:
        print(
            f"seed={row['gazebo_seed']} {row['controller_family']} "
            f"{row['scenario']}: completed={row['completed']}/"
            f"{row['repetitions']}, CTE median="
            f"{row['cte_rmse_m_median']:.6f} m"
        )


if __name__ == '__main__':
    main()
