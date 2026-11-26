#!/usr/bin/env python3
"""Summarize retries by route stage without treating warnings as contacts."""
import argparse
import json
from pathlib import Path
import re

p=argparse.ArgumentParser()
p.add_argument('trial',type=Path)
p.add_argument('--max-retries',type=int)
args=p.parse_args()
r=json.loads((args.trial/'result.json').read_text())
events=r.get('events',[])
starts=[e for e in events if e['state']=='MOVING']
counts={e['detail']:0 for e in starts}
warnings=[]
for line in (args.trial/'control.log').read_text(errors='replace').splitlines():
    if 'trajectory is not feasible' not in line:continue
    match=re.search(r'\[\w+\] \[[0-9.]+, ([0-9.]+)\]',line)
    if not match:continue
    stamp=float(match[1]);before=[e for e in starts if e['stamp']<=stamp]
    stage=before[-1]['detail'] if before else 'startup'
    counts[stage]=counts.get(stage,0)+1
    warnings.append(dict(stamp=stamp,stage=stage))
trace=[json.loads(line) for line in (args.trial/'snapshots/trace.jsonl').read_text().splitlines()]
stages=[]
for e in starts:
    end=next((a for a in events if a['state'] in ('ARRIVED','PASSED') and a['detail']==e['detail']),None)
    stamps=[w['stamp'] for w in warnings if w['stage']==e['detail']]
    nearby=[min(trace,key=lambda t:abs(t['t']-stamp))['position'] for stamp in stamps] if trace else []
    stages.append(dict(stage=e['detail'],wall=end['wall']-e['wall'] if end else None,sim=end['stamp']-e['stamp'] if end else None,
        retries=counts[e['detail']],retry_positions=nearby))
out=dict(total_retries=len(warnings),stages=stages,final_status=r.get('final_status'),
         guard_trips=max((t.get('guard',{}).get('trips',0) for t in trace),default=None))
(args.trial/'health.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
if args.max_retries is not None and len(warnings)>args.max_retries:raise SystemExit(1)
