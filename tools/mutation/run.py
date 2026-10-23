#!/usr/bin/env python3
"""Run the baseline and Mull pilot; reject absent/empty reports, allow survivors."""
# SPDX-License-Identifier: MIT
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import subprocess
import sys


def summarize(report):
    counts = Counter(m['status'] for f in report['files'].values() for m in f['mutants'])
    if not counts or not (counts['Killed'] + counts['Survived']):
        raise ValueError('No executed mutants: check instrumentation and includePaths')
    if counts.keys() - {'Killed', 'Survived', 'NoCoverage'}:
        raise ValueError(f'Unexpected Mull statuses: {dict(counts)}')
    total = sum(counts.values())
    return (
        '# Aetherion mutation pilot\n\n'
        f'Mutants: {total}; killed: {counts["Killed"]}; '
        f'survived: {counts["Survived"]}; no coverage: {counts["NoCoverage"]}.\n\n'
        f'Detection score (including uncovered mutants): {100 * counts["Killed"] / total:.1f}%.\n\n'
        'Scope: Replay/Csv.h, Logs.h and TimeBase.h; this is not a repository-wide score.\n\n'
        'Mull 0.28 Elements groups crashes/timeouts under Killed. Inspect the SQLite report '
        'before treating these as meaningful test detections. Survivors are advisory.\n'
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--runner', default='mull-runner-19')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    binary = args.binary.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # A failed run must never reuse a previous successful result.
    for name in ('estimation.json', 'summary.md'):
        (output / name).unlink(missing_ok=True)
    env = dict(os.environ, MULL_CONFIG=str(Path(__file__).with_name('mull.yml').resolve()))
    with (output / 'baseline.log').open('w') as log:
        subprocess.run([str(binary)], env=env, stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=300)
    # Tests use shared temporary filenames. Parallel mutant processes would race.
    command = [args.runner, str(binary), '--workers=1', '--timeout=30000',
               '--allow-surviving', '--reporters=IDE', '--reporters=Elements', '--reporters=SQLite',
               f'--report-dir={output}', '--report-name=estimation']
    with (output / 'mull.log').open('w') as log:
        subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=5400)
    summary = summarize(json.loads((output / 'estimation.json').read_text()))
    (output / 'summary.md').write_text(summary)
    print(summary)
    if os.environ.get('GITHUB_STEP_SUMMARY'):
        with open(os.environ['GITHUB_STEP_SUMMARY'], 'a') as stream:
            stream.write(summary)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print(f'Mutation run failed: {error}. Inspect baseline.log and mull.log.', file=sys.stderr)
        sys.exit(1)
