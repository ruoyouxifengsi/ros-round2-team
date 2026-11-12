#!/usr/bin/env python3
"""Summarize all requested runs, including failures; never feeds navigation."""
import json
import math
import sys
from pathlib import Path

rows=[]
for folder in map(Path,sys.argv[1:]):
    r=json.loads((folder/'result.json').read_text())
    a=json.loads((folder/'analysis.json').read_text()) if (folder/'analysis.json').exists() else {}
    poses=json.loads((folder/'poses.json').read_text()) if (folder/'poses.json').exists() else []
    audit=json.loads((folder/'cone_audit.json').read_text()) if (folder/'cone_audit.json').exists() else {}
    trace=[json.loads(line) for line in (folder/'snapshots/trace.jsonl').read_text().splitlines()] if (folder/'snapshots/trace.jsonl').exists() else []
    moving=[e for e in r['events'] if e['state']=='MOVING']
    end=r.get('final_status',{})
    row=dict(run=folder.name,seed=r.get('seed'),control_pass=a.get('control_baseline_pass',False),arrivals=len(a.get('arrivals',{})),last_stage=moving[-1]['detail'] if moving else None,final_status=end,last_position=[poses[-1]['x'],poses[-1]['y']] if poses else None,wall_seconds=r.get('elapsed_wall'),sim_seconds=end.get('stamp',moving[0]['stamp'])-moving[0]['stamp'] if moving else None,max_tilt_deg=a.get('max_tilt_deg'),guard_trips=max((t.get('guard',{}).get('trips',0) for t in trace),default=0),cone_audit_complete=len(audit.get('baseline',{}))==5 and audit.get('samples',0)>10,cones_moved_over_5mm=audit.get('moved_over_5mm'),max_cone_displacement_m=max((p['displacement_m'] for p in audit.get('peaks',{}).values()),default=None),error=r.get('error'))
    row['clean_pass']=row['control_pass'] and row['cone_audit_complete'] and not row['cones_moved_over_5mm']
    rows.append(row)
print(json.dumps(dict(runs=rows,control_passes=sum(r['control_pass'] for r in rows),clean_passes=sum(r['clean_pass'] for r in rows),total=len(rows),note='5mm displacement screening is not a zero-contact certificate or official scoring threshold'),indent=2))
