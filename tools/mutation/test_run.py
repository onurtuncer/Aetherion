# SPDX-License-Identifier: MIT
"""Regression checks for the reporting and execution safeguards (no Mull needed)."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import run


def report(*statuses):
    return {'files': {'TimeBase.h': {'mutants': [{'status': s} for s in statuses]}}}


class SummaryTests(unittest.TestCase):
    def test_survivors_are_reported_and_uncovered_count_in_denominator(self):
        result = run.summarize(report('Killed', 'Survived', 'NoCoverage', 'Killed'))
        self.assertIn('50.0%', result)
        self.assertIn('survived: 1', result)

    def test_empty_and_uncovered_only_reports_fail(self):
        for data in (report(), report('NoCoverage')):
            with self.subTest(data=data), self.assertRaises(ValueError):
                run.summarize(data)

    def test_unknown_status_fails(self):
        with self.assertRaises(ValueError):
            run.summarize(report('Killed', 'Unexpected'))

    def test_failed_baseline_stops_runner_and_removes_stale_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            (output / 'estimation.json').write_text(json.dumps(report('Killed')))
            (output / 'summary.md').write_text('old success')
            args = ['run.py', '--binary', 'test_estimation', '--output', tmp]
            with patch('sys.argv', args), patch.object(run.subprocess, 'run') as execute:
                execute.side_effect = subprocess.CalledProcessError(1, 'baseline')
                with self.assertRaises(subprocess.CalledProcessError):
                    run.main()
                self.assertEqual(execute.call_count, 1)
            self.assertFalse((output / 'estimation.json').exists())
            self.assertFalse((output / 'summary.md').exists())

    def test_success_uses_serial_mutants_and_explicit_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            args = ['run.py', '--binary', 'test_estimation', '--output', tmp]
            def execute(command, **kwargs):
                self.assertTrue(Path(kwargs['env']['MULL_CONFIG']).is_file())
                if '--workers=1' in command:
                    (output / 'estimation.json').write_text(json.dumps(report('Killed', 'Survived')))
            with patch('sys.argv', args), patch.dict('os.environ', {}, clear=True), \
                    patch.object(run.subprocess, 'run', side_effect=execute) as mocked:
                run.main()
                self.assertEqual(mocked.call_count, 2)
                self.assertIn('--allow-surviving', mocked.call_args.args[0])
            self.assertTrue((output / 'summary.md').is_file())


if __name__ == '__main__':
    unittest.main()
