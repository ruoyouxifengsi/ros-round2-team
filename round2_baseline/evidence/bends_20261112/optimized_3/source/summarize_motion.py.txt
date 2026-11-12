#!/usr/bin/env python3
"""Separate vehicle speed per simulation second from host simulation throughput."""
import argparse
import json
import math
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('output',type=Path)
p.add_argument('trials',nargs='+',type=Path)
args=p.parse_args();rows=[]
for folder in args.trials:
    result=json.loads((folder/'result.json').read_text())
    events=result['events'];start=next(e for e in events if e['state']=='MOVING')
    end=next(e for e in reversed(events) if e['state'] in ('CONTROL_ROUTE_DONE','FAILED'))
    poses=[p for p in json.loads((folder/'poses.json').read_text()) if start['stamp']<=p['t']<=end['stamp']]
    speeds=sorted(math.hypot(*p['v'][:2]) for p in poses)
    trace=[json.loads(line) for line in (folder/'snapshots/trace.jsonl').read_text().splitlines()]
    log=(folder/'simulation.log').read_text(errors='replace')
    rows.append(dict(trial=folder.name,passed=result['passed'],scope=result.get('scope'),
        timer_used=result['trigger']=='unchanged official Draw widget',timeout_clock=result.get('timeout_clock','wall'),
        moving_sim_seconds=end['stamp']-start['stamp'],moving_wall_seconds=end['wall']-start['wall'],
        peak_measured_speed_mps=max(speeds),p95_measured_speed_mps=speeds[int(.95*(len(speeds)-1))],
        max_tilt_deg=max(math.degrees(math.acos(max(-1.,min(1.,math.cos(p['rpy'][0])*math.cos(p['rpy'][1]))))) for p in poses),
        guard_trips=max(t.get('guard',{}).get('trips',0) for t in trace),
        infeasible_retries=(folder/'control.log').read_text(errors='replace').count('trajectory is not feasible'),
        engine=result['simulation_engine'],
        parallel_adapter_active='experimental adapter active' in log,
        cleanup=result.get('cleanup'),
        physics=json.loads((folder/'physics_properties.json').read_text()),
        source_hashes=json.loads((folder/'source_hashes.json').read_text())))
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(rows,indent=2));print(json.dumps([{k:v for k,v in r.items() if k not in ('physics','source_hashes')} for r in rows],indent=2))
