#!/usr/bin/env python3
"""Compare matched seeds using both wall-clock and simulation durations."""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('stable_prefix');p.add_argument('fast_prefix');p.add_argument('seeds',type=int,nargs='+');args=p.parse_args()
rows=[]
for seed in args.seeds:
    pair=[]
    for prefix in (args.stable_prefix,args.fast_prefix):
        folder=Path(prefix+'_s'+str(seed));r=json.loads((folder/'result.json').read_text());a=json.loads((folder/'analysis.json').read_text())
        start=next(e['stamp'] for e in r['events'] if e['state']=='MOVING')
        pair.append(dict(wall=r['elapsed_wall'],sim=r['final_status']['stamp']-start,passed=a['control_baseline_pass']))
    stable,fast=pair
    rows.append(dict(seed=seed,stable=stable,fast=fast,wall_reduction_pct=100*(1-fast['wall']/stable['wall']),sim_reduction_pct=100*(1-fast['sim']/stable['sim'])))
print(json.dumps(dict(runs=rows,aggregate_wall_reduction_pct=100*(1-sum(r['fast']['wall'] for r in rows)/sum(r['stable']['wall'] for r in rows)),aggregate_sim_reduction_pct=100*(1-sum(r['fast']['sim'] for r in rows)/sum(r['stable']['sim'] for r in rows)),note='One trial per seed/profile; not a statistical performance guarantee'),indent=2))
