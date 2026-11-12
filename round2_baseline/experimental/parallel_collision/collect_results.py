#!/usr/bin/env python3
"""Keep compact, auditable results without publishing ROS debug logs."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

p = argparse.ArgumentParser()
p.add_argument('output', type=Path)
p.add_argument('folders', nargs='+', type=Path)
args = p.parse_args()
scripts = Path(__file__).resolve().parents[2]/'scripts'
metrics = json.loads(subprocess.check_output([sys.executable, str(scripts/'summarize_performance.py'), *map(str, args.folders)]))
rows = []
for folder, metric in zip(args.folders, metrics):
    log = (folder/'simulation.log').read_text()
    records = re.findall(r'\[parallel-collision\] steps=.*', log)
    result = json.loads((folder/'result.json').read_text())
    trace = [json.loads(line) for line in (folder/'snapshots/trace.jsonl').read_text().splitlines()]
    guards = [sample['guard'] for sample in trace if sample.get('guard')]
    row = dict(folder=folder.as_posix(), metrics=metric, result=result,
               engine_active='experimental adapter active' in log,
               last_engine_stats=records[-1] if records else None,
               contact_mismatch='contact mismatch' in log,
               guard_trips=max((g['trips'] for g in guards), default=None),
               source_hashes=json.loads((folder/'source_hashes.json').read_text()))
    for filename in ('physics_properties.json', 'analysis.json', 'cone_audit.json'):
        if (folder/filename).exists():
            row[filename[:-5]] = json.loads((folder/filename).read_text())
    rows.append(row)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(rows, indent=2))
print(args.output)
