#!/usr/bin/env python3
"""Export verified metrics and retain failed development attempts separately."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

p=argparse.ArgumentParser()
p.add_argument('batch',type=Path)
p.add_argument('development_batch',type=Path)
p.add_argument('output',type=Path)
args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
batch=json.loads((args.batch/'summary.json').read_text())
development=json.loads((args.development_batch/'summary.json').read_text())
full=[r for r in batch['attempts'] if r['scope']=='full']
if not full or not all(r['accepted'] for r in full):raise RuntimeError('Final batch is not accepted')
reference_sources=None;reference_assets=None;reference_physics=None
rows=[]
for run in full:
    folder=args.batch/run['run']
    sources=json.loads((folder/'source_hashes.json').read_text())
    assets={str(path.relative_to(folder)):hashlib.sha256(path.read_bytes()).hexdigest()
        for sub in ('config','launch') for path in (folder/sub).glob('*') if path.is_file()}
    if reference_sources is None:reference_sources=sources;reference_assets=assets;reference_physics=run['physics']
    if sources!=reference_sources or assets!=reference_assets:raise RuntimeError('Source/configuration changed during final batch')
    if run['physics']!=reference_physics:raise RuntimeError('Physics configuration changed')
    events=run['result']['events']
    first=next(e for e in events if e['state']=='MOVING')
    last=next(e for e in reversed(events) if e['state']=='CONTROL_ROUTE_DONE')
    observed={}
    for item in run['engine_stats'][-1].split():
        if '=' in item:
            key,value=item.split('=',1)
            observed[key]=float(value)
    if observed.get('verified',0)<=0 or observed.get('cache_hits',0)<=0 or observed.get('mismatches',1)!=0:
        raise RuntimeError('Missing engine activation/verification evidence')
    rows.append(dict(run=run['run'],seed=run['seed'],passed=run['accepted'],
        elapsed_wall=run['result']['elapsed_wall'],moving_wall=last['wall']-first['wall'],
        moving_sim=last['stamp']-first['stamp'],startup_seconds=run['startup_seconds'],
        retries=run['health']['total_retries'],contact_episodes=len(run['contacts']),
        max_cone_displacement_m=max(p['displacement_m'] for p in run['cone_audit']['peaks'].values()),
        guard_trips=run['health']['guard_trips'],max_tilt_deg=run['analysis']['max_tilt_deg'],
        qr_in_region=run['analysis']['arrivals']['qr']['body_in_region'],
        finish_in_region=run['analysis']['arrivals']['finish']['body_in_region'],
        finish_stopped=run['analysis']['arrivals']['finish']['stopped'],
        verified_pairs_at_least=int(observed['verified']),contact_mismatches=int(observed['mismatches']),
        contact_candidates=run['contact_summary']['candidates'],contact_unsupported=run['contact_summary']['unsupported'],
        cleanup_remaining=run['cleanup']['remaining']))
probes=[r for r in development['attempts'] if r['scope']=='startup']
failed=[]
for run in development['attempts']:
    if run['scope']=='full':
        poses=json.loads((args.development_batch/run['run']/'poses.json').read_text())
        failed.append(dict(run=run['run'],passed=run['accepted'],result=run['result'],
            last_pose=poses[-1],retries=run.get('health',{}).get('total_retries')))
summary=dict(scope='Headless WSL control-only tests; image recognition and speech skipped; no competition approval for custom engine implied',
    engine_sha256=batch['engine_sha256'],runs=rows,full_passes=sum(r['passed'] for r in rows),full_attempts=len(rows),
    distinct_seeds=sorted({r['seed'] for r in rows}),mean_elapsed_wall=sum(r['elapsed_wall'] for r in rows)/len(rows),
    startup_probes_passed=sum(r['accepted'] for r in probes),startup_probes=len(probes),
    final_run_startups_passed=sum(r['startup_pass'] for r in full),
    identical_sources=True,identical_configuration=True,identical_runtime_physics=True,
    runtime_physics=reference_physics,source_hashes=reference_sources,configuration_hashes=reference_assets)
(args.output/'summary.json').write_text(json.dumps(summary,indent=2))
(args.output/'detailed_trials.json').write_text(json.dumps(batch,indent=2))
(args.output/'development_attempts.json').write_text(json.dumps(dict(startup_probes=probes,failed_full_trials=failed),indent=2))
print(json.dumps(summary,indent=2))
