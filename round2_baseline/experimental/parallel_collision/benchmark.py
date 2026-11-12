#!/usr/bin/env python3
"""Sequential, isolated first-corner comparison; never a full-course verdict."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('output', type=Path)
p.add_argument('--library', type=Path, default=Path.home()/'round2_engine_experiment/build/libround2_parallel_collision.so')
p.add_argument('--variants', nargs='+', default=['stock', '1', '4', '8', 'stock'])
args = p.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
scripts = Path(__file__).resolve().parents[2]/'scripts'
folders = []
for index, variant in enumerate(args.variants):
    folder = args.output/f'{index}_{variant}'
    command = [sys.executable, str(scripts/'run_trial.py'), str(folder),
        '--profile-performance', '--low-com', '--cones', '--seed', '42',
        '--speed-profile', 'fast', '--debug-pose', '-0.75', '1.75', '1.57079632679',
        '--debug-goal', '-0.70', '2.67', '0', '--debug-control', '--debug-stage', 'bend1', '--seconds', '110']
    if variant != 'stock':
        command += ['--experimental-collision-library', str(args.library), '--collision-threads', str(int(variant))]
    print(f'Starting {folder.name}', flush=True)
    with (args.output/f'{index}_{variant}.log').open('w') as log:
        completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    print(f'Completed {folder.name}: exit={completed.returncode}', flush=True)
    if variant != 'stock':
        log = (folder/'simulation.log').read_text()
        hits = re.findall(r'cache_hits=(\d+)', log)
        if not hits or int(hits[-1]) == 0 or 'contact mismatch' in log:
            raise RuntimeError(f'{folder}: no valid active parallel collision result')
    folders.append(str(folder))
with (args.output/'comparison.json').open('w') as output:
    subprocess.run([sys.executable, str(scripts/'summarize_performance.py'), *folders], stdout=output, check=True)
print((args.output/'comparison.json').read_text())
physics = [json.loads((Path(f)/'physics_properties.json').read_text()) for f in folders
           if (Path(f)/'physics_properties.json').exists()]
if physics and any(p != physics[0] for p in physics[1:]):
    raise RuntimeError('Runtime physics properties differ between runs')
