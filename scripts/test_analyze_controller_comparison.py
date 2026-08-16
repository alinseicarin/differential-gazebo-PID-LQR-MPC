#!/usr/bin/env python3
"""Focused unit tests for dependency-free comparison statistics."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_controller_comparison as analysis  # noqa: E402


class ComparisonStatisticsTest(unittest.TestCase):

    def test_quantile_uses_linear_interpolation(self):
        values = [0.0, 10.0, 20.0, 30.0]
        self.assertAlmostEqual(analysis.quantile(values, 0.25), 7.5)
        self.assertAlmostEqual(analysis.quantile(values, 0.5), 15.0)
        self.assertAlmostEqual(analysis.quantile(values, 0.75), 22.5)

    def test_exact_sign_flip_enumerates_all_assignments(self):
        p_value, method = analysis.exact_sign_flip_test([1.0, 1.0])
        self.assertEqual(method, 'exact_sign_flip')
        self.assertAlmostEqual(p_value, 0.5)
        self.assertEqual(analysis.exact_sign_flip_test([0.0, 0.0])[0], 1.0)

    def test_exact_mcnemar_uses_only_discordant_completion_pairs(self):
        self.assertAlmostEqual(analysis.exact_mcnemar_test(9, 0), 0.00390625)
        self.assertEqual(analysis.exact_mcnemar_test(0, 0), 1.0)

    def test_completion_role_comes_from_predeclared_protocol(self):
        protocol = {
            'completion_analysis_role': 'predeclared_confirmatory',
            'completion_confirmatory_scenarios': (
                'left_wheel_loss_persistent'
            ),
        }
        self.assertEqual(
            analysis.completion_inference_metadata(
                protocol, 'left_wheel_loss_persistent'
            ),
            ('confirmatory_primary', 'confirmatory_completion'),
        )
        self.assertEqual(
            analysis.completion_inference_metadata(protocol, 'nominal'),
            ('exploratory_secondary', 'exploratory_completion_secondary'),
        )
        self.assertEqual(
            analysis.completion_inference_metadata({}, 'nominal'),
            ('exploratory_post_hoc', 'exploratory_completion'),
        )

    def test_paired_effect_uses_difference_standard_deviation(self):
        expected = 2.0 / 1.0
        self.assertAlmostEqual(
            analysis.paired_effect_dz([1.0, 2.0, 3.0]), expected
        )
        self.assertTrue(math.isnan(analysis.paired_effect_dz([1.0, 1.0])))

    def test_holm_adjustment_is_monotone_in_sorted_p_values(self):
        rows = [
            {'holm_family': 'a', 'sign_flip_p_value': 0.01},
            {'holm_family': 'a', 'sign_flip_p_value': 0.04},
            {'holm_family': 'a', 'sign_flip_p_value': 0.03},
        ]
        for row in rows:
            row.update({
                'holm_adjusted_p_value': math.nan,
                'holm_family_size': 0,
                'holm_significant_0p05': 0,
            })
        analysis.apply_holm_correction(rows)
        self.assertAlmostEqual(rows[0]['holm_adjusted_p_value'], 0.03)
        self.assertAlmostEqual(rows[1]['holm_adjusted_p_value'], 0.06)
        self.assertAlmostEqual(rows[2]['holm_adjusted_p_value'], 0.06)
        self.assertEqual({row['holm_family_size'] for row in rows}, {3})

    def test_degradation_uses_same_controller_seed_track_nominal(self):
        nominal = {
            'controller_family': 'pid', 'gazebo_seed': 500,
            'track': 'figure_eight', 'scenario': 'nominal',
            'cte_rmse_m': '0.02', 'heading_rmse_rad': '0.03',
            'temporal_position_rmse_m': '0.04',
            'peak_abs_cte_m': '0.05',
            'normalized_command_activity': '2.0',
        }
        perturbed = {
            'controller_family': 'pid', 'gazebo_seed': 500,
            'track': 'figure_eight', 'scenario': 'angular_pulse_train',
            'cte_rmse_m': '0.05', 'heading_rmse_rad': '0.07',
            'temporal_position_rmse_m': '0.10',
            'peak_abs_cte_m': '0.12',
            'normalized_command_activity': '2.5',
        }
        analysis.add_nominal_degradation_metrics([nominal, perturbed])
        self.assertAlmostEqual(perturbed['degradation_cte_rmse_m'], 0.03)
        self.assertAlmostEqual(
            perturbed['degradation_temporal_position_rmse_m'], 0.06
        )
        self.assertTrue(math.isnan(nominal['degradation_cte_rmse_m']))

    def test_confirmatory_family_selection_is_predeclared(self):
        self.assertEqual(
            analysis.inference_metadata(
                'circle', 'nominal', 'temporal_position_rmse_m'
            ),
            ('confirmatory', 'nominal_temporal_accuracy'),
        )
        self.assertEqual(
            analysis.inference_metadata(
                'figure_eight', 'left_wheel_loss_persistent',
                'active_tail_mean_abs_cte_m'
            ),
            ('confirmatory', 'persistent_spatial_robustness'),
        )
        self.assertEqual(
            analysis.inference_metadata(
                'circle', 'nominal', 'heading_rmse_rad'
            )[0],
            'exploratory',
        )

    def test_custom_practical_thresholds_are_accepted(self):
        thresholds = analysis.practical_threshold_map(0.00875, 0.02)
        self.assertEqual(thresholds['cte_rmse_m'], 0.00875)
        self.assertEqual(thresholds['heading_rmse_rad'], 0.02)
        self.assertEqual(
            analysis.practical_interpretation(-0.001, 0.001, 0.00875),
            'confidence_interval_inside_practical_band',
        )


if __name__ == '__main__':
    unittest.main()
