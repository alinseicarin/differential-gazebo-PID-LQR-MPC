#!/usr/bin/env python3
"""Unit tests for dependency-free SVG comparison figures."""

from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
import generate_controller_comparison_report as report  # noqa: E402


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


if __name__ == '__main__':
    unittest.main()
