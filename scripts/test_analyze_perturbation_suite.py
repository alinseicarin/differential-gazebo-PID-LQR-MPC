#!/usr/bin/env python3
"""Focused tests for experiment-horizon handling in robustness analysis."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_perturbation_suite as analysis  # noqa: E402


class PerturbationAnalysisTest(unittest.TestCase):

    def test_fixed_horizon_removes_polling_overshoot(self):
        rows = [
            {'time': '29.98'},
            {'time': '30.00'},
            {'time': '30.42'},
        ]
        cropped = analysis.crop_to_observation_horizon(rows, 'time', 30.0)
        self.assertEqual([row['time'] for row in cropped], ['29.98', '30.00'])

    def test_zero_horizon_preserves_legacy_variable_duration(self):
        rows = [{'time': '1.0'}, {'time': '2.0'}]
        self.assertIs(
            analysis.crop_to_observation_horizon(rows, 'time', 0.0),
            rows,
        )

    def test_legacy_horizon_uses_reference_end_plus_settling_time(self):
        rows = [
            {'time': '0.0', 'reference_linear_velocity': '0.0',
             'reference_angular_velocity': '0.0'},
            {'time': '1.0', 'reference_linear_velocity': '0.4',
             'reference_angular_velocity': '0.1'},
            {'time': '2.0', 'reference_linear_velocity': '0.2',
             'reference_angular_velocity': '0.0'},
            {'time': '3.0', 'reference_linear_velocity': '0.0',
             'reference_angular_velocity': '0.0'},
        ]
        self.assertAlmostEqual(
            analysis.inferred_reference_observation_horizon(rows), 4.0
        )


if __name__ == '__main__':
    unittest.main()
