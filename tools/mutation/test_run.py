# SPDX-License-Identifier: MIT
"""Regression checks for the reporting and execution safeguards (no Mull needed)."""
import json
import io
from contextlib import redirect_stderr
from pathlib import Path
import subprocess
import sys
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
                    # Mull's warm-up must accept every duration allowed by our
                    # external baseline gate, including a healthy 90-second run.
                    timeout_ms = int(next(arg.split('=')[1] for arg in command if arg.startswith('--timeout=')))
                    self.assertGreaterEqual(timeout_ms, 90000)
                    self.assertEqual(timeout_ms, run.BASELINE_TIMEOUT_SECONDS * 1000)
                    self.assertEqual(kwargs['timeout'], run.CAMPAIGN_TIMEOUT_SECONDS)
                    (output / 'estimation.json').write_text(json.dumps(report('Killed', 'Survived')))
                else:
                    self.assertEqual(kwargs['timeout'], run.BASELINE_TIMEOUT_SECONDS)
            with patch('sys.argv', args), patch.dict('os.environ', {}, clear=True), \
                    patch.object(run.subprocess, 'run', side_effect=execute) as mocked:
                run.main()
                self.assertEqual(mocked.call_count, 2)
                self.assertIn('--allow-surviving', mocked.call_args.args[0])
            self.assertTrue((output / 'summary.md').is_file())

    def test_runner_failure_still_fails_after_a_passing_baseline(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = ['run.py', '--binary', 'test_estimation', '--output', tmp]
            failure = subprocess.CalledProcessError(1, 'mull-runner-19')
            with patch('sys.argv', args), redirect_stderr(io.StringIO()), \
                    patch.object(run.subprocess, 'run', side_effect=[None, failure]) as execute:
                with self.assertRaises(subprocess.CalledProcessError):
                    run.main()
                self.assertEqual(execute.call_count, 2)
            self.assertFalse((Path(tmp) / 'summary.md').exists())

    def test_real_process_failure_exposes_log_and_preserves_exit_code(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'mull.log'
            stderr = io.StringIO()
            command = [sys.executable, '-c', "print('warm-up diagnostic', flush=True); raise SystemExit(7)"]
            with redirect_stderr(stderr), self.assertRaises(subprocess.CalledProcessError) as caught:
                run.run_logged(command, None, log, 5)
            self.assertEqual(caught.exception.returncode, 7)
            self.assertIn('warm-up diagnostic', log.read_text())
            self.assertIn('warm-up diagnostic', stderr.getvalue())

    def test_real_process_timeout_remains_a_failure_and_exposes_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'baseline.log'
            stderr = io.StringIO()
            command = [sys.executable, '-c', "print('before timeout', flush=True); import time; time.sleep(30)"]
            with redirect_stderr(stderr), self.assertRaises(subprocess.TimeoutExpired):
                run.run_logged(command, None, log, 1)
            self.assertIn('before timeout', stderr.getvalue())


if __name__ == '__main__':
    unittest.main()
