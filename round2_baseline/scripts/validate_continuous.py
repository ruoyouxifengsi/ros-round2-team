"""Validate exactly three official random layouts, stopping at the first failure."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

p=argparse.ArgumentParser()
p.add_argument('output',type=Path)
p.add_argument('--observer',type=Path,required=True)
p.add_argument('--profile',choices=['fast','sprint'],default='sprint')
p.add_argument('--seeds',nargs=3,type=int,default=[42,1,2])
args=p.parse_args()
if not args.observer.is_file() or not os.access(args.observer,os.X_OK):p.error('Build the observer in a persistent WSL directory first')
if len(set(args.seeds))!=3:p.error('Use three distinct seeds')
args.output.mkdir(parents=True,exist_ok=False)
scripts=Path(__file__).parent;rows=[]
for seed in args.seeds:
    folder=args.output/('seed_'+str(seed))
    command=[sys.executable,str(scripts/'run_trial.py'),str(folder),'--low-com','--cones','--seed',str(seed),
        '--speed-profile',args.profile,'--continuous-bends','--timeout-clock','sim','--audit-cones',
        '--contact-observer',str(args.observer),'--profile-performance','--seconds','350']
    print('START seed '+str(seed),flush=True)
    with (args.output/('seed_'+str(seed)+'.log')).open('w') as log:
        run=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        assessment=subprocess.run([sys.executable,str(scripts/'assess_practice.py'),str(folder)],stdout=log,stderr=subprocess.STDOUT) if (folder/'poses.json').exists() else None
    row=json.loads((folder/'practice.json').read_text()) if (folder/'practice.json').exists() else dict(seed=seed,accepted=False,result=json.loads((folder/'result.json').read_text()))
    row['run_exit']=run.returncode;row['assessment_exit']=assessment.returncode if assessment else None
    rows.append(row)
    (args.output/'summary.json').write_text(json.dumps(dict(attempts=rows,passed=sum(r['accepted'] for r in rows),requested_seeds=args.seeds),indent=2))
    print(json.dumps({k:row.get(k) for k in ('seed','accepted','moving_sim_seconds','guard_trips','retries','contact_summary','final_status')}),flush=True)
    if run.returncode or assessment is None or assessment.returncode or not row['accepted']:raise SystemExit(1)
