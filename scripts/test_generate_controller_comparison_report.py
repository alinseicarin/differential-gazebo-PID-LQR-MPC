#!/usr/bin/env python3
"""Unit tests for dependency-free SVG comparison figures."""

from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
import generate_controller_comparison_report as report  # noqa: E402


# Synthetic inputs protect axis selection, SVG validity, optional-data handling,
# and confirmatory-versus-post-hoc wording without a full Gazebo campaign.
class ComparisonReportTest(unittest.TestCase):

    def test_nice_step_uses_readable_one_two_five_sequence(self):
        self.assertEqual(report.nice_step(0.006), 0.001)
        self.assertEqual(report.nice_step(0.08), 0.02)
        self.assertEqual(report.nice_step(8.0), 2.0)

    def test_grouped_svg_is_valid_xml(self):
        values = {
            'pid': [(0.02, 0.01, 0.03)],
            'lqr': [(0.015, 0.012, 0.018)],
            'mpc': [(0.018, 0.014, 0.022)],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'figure.svg'
            report.svg_bar_chart(
                path, 'Test figure', 'RMSE [m]', ['Circle'], values
            )
            tree = ElementTree.parse(path)
            self.assertTrue(tree.getroot().tag.endswith('svg'))
            self.assertIn('TVLQR', path.read_text(encoding='utf-8'))

    def test_missing_optional_metrics_are_detected_before_plotting(self):
        missing = {
            controller: [(float('nan'), float('nan'), float('nan'))]
            for controller in report.CONTROLLERS
        }
        self.assertFalse(report.grouped_chart_has_finite_value(missing))
        rows = [{
            'controller_family': 'pid', 'track': 'figure_eight',
            'scenario': 'left_wheel_loss_persistent',
            'window_recovery_fraction': '',
        }]
        self.assertFalse(report.paired_chart_has_finite_value(
            (('figure_eight', 'left_wheel_loss_persistent'),),
            'window_recovery_fraction', rows,
        ))

    def test_markdown_uses_actual_run_count_and_labels_completion_post_hoc(self):
        raw = [
            {'controller_family': 'pid', 'track': 'figure_eight',
             'scenario': 'nominal'},
            {'controller_family': 'lqr', 'track': 'figure_eight',
             'scenario': 'nominal'},
        ]
        grouped = [
            {'controller_family': 'pid', 'track': 'figure_eight',
             'scenario': 'nominal', 'completed': '1', 'runs': '1'},
            {'controller_family': 'lqr', 'track': 'figure_eight',
             'scenario': 'nominal', 'completed': '1', 'runs': '1'},
        ]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report.write_markdown_report(
                root, [], raw, grouped, [], [], []
            )
            text = (root / 'statistical_report.md').read_text(
                encoding='utf-8'
            )
            self.assertIn('contine 2 de rulari', text)
            self.assertIn('analiza exploratorie post-hoc', text)
            self.assertNotIn('330 de rulari', text)

    def test_markdown_labels_predeclared_completion_as_confirmatory(self):
        raw = [
            {'controller_family': 'pid', 'track': 'figure_eight',
             'scenario': 'left_wheel_loss_persistent'},
        ]
        grouped = [
            {'controller_family': 'pid', 'track': 'figure_eight',
             'scenario': 'left_wheel_loss_persistent',
             'completed': '0', 'runs': '1'},
        ]
        completion = [{
            'inference_role': 'confirmatory_primary',
            'holm_significant_0p05': '0',
        }]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report.write_markdown_report(
                root, [], raw, grouped, [], [], completion
            )
            text = (root / 'statistical_report.md').read_text(
                encoding='utf-8'
            )
            self.assertIn('analiza confirmatorie predeclarata', text)
            self.assertIn('endpoint primar', text)


if __name__ == '__main__':
    unittest.main()
