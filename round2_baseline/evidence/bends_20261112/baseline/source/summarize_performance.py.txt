#!/usr/bin/env python3
"""Compare development trials without confusing simulation time with wall time."""
import json
from pathlib import Path
import statistics
import sys

rows=[]
for folder in map(Path,sys.argv[1:]):
    result=json.loads((folder/'result.json').read_text())
    events=result.get('events',[])
    start=next((e for e in events if e['state']=='MOVING'),None)
    end=next((e for e in reversed(events) if e['state'] in ('CONTROL_ROUTE_DONE','FAILED')),None)
    samples=[json.loads(line) for line in (folder/'performance.jsonl').read_text().splitlines()]
    if start and end:samples=[s for s in samples if start['wall']<=s['wall']<=end['wall']]
    loads=[p for s in samples for p in s['processes'] if p['name']=='gzserver' and p['threads']>1]
    wall=end['wall']-start['wall'] if start and end else None
    sim=end['stamp']-start['stamp'] if start and end else None
    rows.append(dict(run=folder.name,passed=result['passed'],cpus=samples[0]['cpus'] if samples else None,
        wall=wall,sim=sim,rtf=sim/wall if wall else None,samples=len(samples),
        gzserver_cpu_median=statistics.median(p['cpu_percent'] for p in loads) if loads else None,
        busiest_thread_median=statistics.median(p['busiest_thread_percent'] for p in loads) if loads else None,
        infeasible_warnings=(folder/'control.log').read_text(errors='replace').count('trajectory is not feasible')))
print(json.dumps(rows,indent=2))
