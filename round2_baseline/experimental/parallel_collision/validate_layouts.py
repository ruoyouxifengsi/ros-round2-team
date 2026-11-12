#!/usr/bin/env python3
"""Sequential official-layout trials; every attempt stays in the denominator."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

p=argparse.ArgumentParser()
p.add_argument('output',type=Path)
p.add_argument('--startup-repeats',type=int,default=8)
p.add_argument('--seeds',nargs='*',type=int,default=[42,1,2,3,42,3])
p.add_argument('--library',type=Path,default=Path.home()/'round2_engine_experiment/build/libround2_parallel_collision.so')
args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
scripts=Path(__file__).resolve().parents[2]/'scripts'
library_hash=hashlib.sha256(args.library.read_bytes()).hexdigest()
attempts=[]
tasks=[('startup',None)]*args.startup_repeats+[('full',seed) for seed in args.seeds]
for index,(scope,seed) in enumerate(tasks):
    if hashlib.sha256(args.library.read_bytes()).hexdigest()!=library_hash:raise RuntimeError('Engine changed during validation')
    folder=args.output/f'{index:02d}_{scope}_{seed if seed is not None else "probe"}'
    command=[sys.executable,str(scripts/'run_trial.py'),str(folder),'--low-com','--speed-profile','fast',
        '--profile-performance','--experimental-collision-library',str(args.library),'--collision-threads','4',
        '--verify-collisions','--collision-verify-stride','100']
    if scope=='startup':command+=['--startup-only']
    else:command+=['--cones','--seed',str(seed),'--timer','--audit-cones','--seconds','600']
    print(f'START {folder.name}',flush=True)
    with (args.output/f'{folder.name}.log').open('w') as log:
        run=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
    result=json.loads((folder/'result.json').read_text())
    log=(folder/'simulation.log').read_text(errors='replace') if (folder/'simulation.log').exists() else ''
    row=dict(run=folder.name,scope=scope,seed=seed,exit_code=run.returncode,
        startup_pass='startup_seconds' in result,startup_seconds=result.get('startup_seconds'),
        cleanup=result.get('cleanup'),result=result,
        engine_active='experimental adapter active' in log,
        engine_stats=re.findall(r'\[parallel-collision\] steps=.*',log)[-1:])
    if (folder/'physics_properties.json').exists():row['physics']=json.loads((folder/'physics_properties.json').read_text())
    if scope=='full' and (folder/'poses.json').exists():
        with (folder/'analysis.log').open('w') as output:
            subprocess.run([sys.executable,str(scripts/'analyze_trial.py'),str(folder)],stdout=output,stderr=subprocess.STDOUT,check=True)
            subprocess.run([sys.executable,str(scripts/'trial_health.py'),str(folder)],stdout=output,stderr=subprocess.STDOUT,check=True)
        row['analysis']=json.loads((folder/'analysis.json').read_text())
        row['health']=json.loads((folder/'health.json').read_text())
        row['cone_audit']=json.loads((folder/'cone_audit.json').read_text())
        contact_file=folder/'cone_contacts.jsonl'
        contacts=[json.loads(line) for line in contact_file.read_text().splitlines()] if contact_file.exists() else []
        summaries=[r for r in contacts if r['type']=='summary']
        row['contact_summary']=summaries[-1] if summaries else None
        row['contacts']=[r for r in contacts if r['type']=='contact_begin']
        row['contact_audit_complete']=bool(summaries and summaries[-1]['unsupported']==0 and len(row['cone_audit']['baseline'])==5)
        row['accepted']=bool(run.returncode==0 and row['analysis']['control_baseline_pass'] and
            row['health']['guard_trips']==0 and row['contact_audit_complete'] and not row['contacts'] and
            not row['cone_audit']['moved_over_5mm'] and row['health']['total_retries']<=10 and
            row['engine_active'] and 'contact mismatch' not in log and not result.get('cleanup',{}).get('remaining',[1]))
    else:row['accepted']=bool(scope=='startup' and run.returncode==0 and row['startup_pass'] and row['engine_active'] and not result.get('cleanup',{}).get('remaining',[1]))
    attempts.append(row)
    summary=dict(engine_sha256=library_hash,attempts=attempts,
        startup_successes=sum(r['startup_pass'] for r in attempts),startup_attempts=len(attempts),
        full_successes=sum(r['accepted'] for r in attempts if r['scope']=='full'),
        full_attempts=sum(r['scope']=='full' for r in attempts))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(dict(run=folder.name,accepted=row['accepted'],elapsed=result.get('elapsed_wall'),
        retries=row.get('health',{}).get('total_retries'),contacts=len(row.get('contacts',[])),
        error=result.get('error'),status=result.get('final_status'))),flush=True)
    if not row['accepted']:
        print('Stopping at the first failure; fix before starting a fresh validation batch.',flush=True)
        raise SystemExit(1)
