#!/usr/bin/env python3
"""Regression check for the first-corner stop/start failure, not full-route approval."""
import argparse
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('trial', type=Path)
p.add_argument('--max-wall', type=float, default=60.)
args = p.parse_args()
result = json.loads((args.trial / 'result.json').read_text())
events = result['events']
start = next((e for e in events if e['state']=='MOVING' and e['detail']=='bend1'), None)
end = next((e for e in events if e['state']=='ARRIVED' and e['detail']=='bend1'), None)
wall = end['wall']-start['wall'] if start and end else None
warnings = (args.trial / 'control.log').read_text().count('trajectory is not feasible')
trace = [json.loads(line) for line in (args.trial / 'snapshots/trace.jsonl').read_text().splitlines()]
trips = max((row.get('guard', {}).get('trips', 0) for row in trace), default=0)
passed = wall is not None and wall <= args.max_wall and warnings == 0 and trips == 0
print(json.dumps(dict(first_corner_pass=passed, wall_seconds=wall,
    infeasible_warnings=warnings, guard_trips=trips,
    scope='First corner only; 60s is a development regression budget, not an official rule'), indent=2))
raise SystemExit(0 if passed else 1)
