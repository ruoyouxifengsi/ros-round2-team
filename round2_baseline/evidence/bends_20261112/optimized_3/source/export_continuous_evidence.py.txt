"""Export compact, auditable records after the three-layout validation passes."""
import argparse
import json
import math
from pathlib import Path
import shutil
import hashlib

p=argparse.ArgumentParser()
p.add_argument('output',type=Path)
p.add_argument('baseline',type=Path)
p.add_argument('prefix',type=Path)
p.add_argument('validation',type=Path)
p.add_argument('--fixture',type=Path,required=True)
p.add_argument('--failed-attempt',type=Path,action='append',default=[])
args=p.parse_args();batch=json.loads((args.validation/'summary.json').read_text())
assert batch['passed']==3 and len(batch['attempts'])==3
args.output.mkdir(parents=True,exist_ok=False)
baseline_physics=json.loads((args.baseline/'physics_properties.json').read_text())
baseline_model_hash=hashlib.sha256((args.baseline/'low_com.urdf').read_bytes()).hexdigest()
navigation_hashes=[]

def export(folder,name):
    target=args.output/name;target.mkdir()
    for file in ('result.json','analysis.json','practice.json','health.json','cone_audit.json','physics_properties.json','source_hashes.json'):
        if (folder/file).exists():shutil.copyfile(folder/file,target/file)
    r=json.loads((folder/'result.json').read_text());poses=json.loads((folder/'poses.json').read_text())
    first=next(e for e in r['events'] if e['state']=='MOVING')
    last=next(e for e in reversed(r['events']) if e['state'] in ('CONTROL_ROUTE_DONE','FAILED'))
    selected=[v for v in poses if first['stamp']<=v['t']<=last['stamp']]
    stopped=sum(max(0.,b['t']-a['t']) for a,b in zip(selected,selected[1:]) if math.hypot(*a['v'][:2])<.02 and abs(a['v'][2])<.05)
    row=dict(seed=r['seed'],scope=r.get('scope','FULL CONTROL ROUTE'),passed=r['passed'],
        moving_sim_seconds=last['stamp']-first['stamp'],moving_wall_seconds=last['wall']-first['wall'],
        stopped_sim_seconds=stopped,peak_planar_speed_mps=max(math.hypot(*v['v'][:2]) for v in selected),
        max_tilt_deg=max(math.degrees(math.acos(max(-1.,min(1.,math.cos(v['rpy'][0])*math.cos(v['rpy'][1]))))) for v in selected),
        guard_max_trips=r.get('guard_max_trips'),
        retries=(folder/'control.log').read_text(errors='replace').count('trajectory is not feasible'))
    if (folder/'contacts.jsonl').exists():
        rows=[json.loads(line) for line in (folder/'contacts.jsonl').read_text().splitlines()]
        summaries=[s for s in rows if s['type']=='summary' and s['messages']>0]
        contact=dict(first=summaries[0],last=summaries[-1],events=[s for s in rows if s['type']=='contact_begin'])
        (target/'contacts.json').write_text(json.dumps(contact,indent=2))
    (target/'motion.json').write_text(json.dumps(row,indent=2))
    return row,r

baseline,br=export(args.baseline,'baseline_seed42')
prefix,pr=export(args.prefix,'continuous_fast_prefix')
final=[]
for row in batch['attempts']:
    trial=args.validation/row['trial']
    assert json.loads((trial/'physics_properties.json').read_text())==baseline_physics
    assert hashlib.sha256((trial/'low_com.urdf').read_bytes()).hexdigest()==baseline_model_hash
    analysis=json.loads((trial/'analysis.json').read_text())
    assert all(analysis['arrivals'][name]['stopped'] for name in analysis['expected_stages'] if not name.startswith('bend'))
    hashes=json.loads((trial/'source_hashes.json').read_text())
    navigation_hashes.append({name:hashes['scripts/'+name] for name in ('control.py','attitude_guard.py','cone_obstacles.py')})
    for name in ('navigation.yaml','speed_sprint.yaml','docking.yaml'):
        navigation_hashes[-1][name]=hashlib.sha256((trial/'config'/name).read_bytes()).hexdigest()
    navigation_hashes[-1]['control.launch']=hashlib.sha256((trial/'launch/control.launch.txt').read_bytes()).hexdigest()
    motion,_=export(trial,row['trial'])
    final.append(dict(row,motion=motion))
assert all(h==navigation_hashes[0] for h in navigation_hashes)
matched=next(row for row in final if row['seed']==baseline['seed'])
old_layout=json.loads((args.baseline/'cone_audit.json').read_text())['baseline']
new_layout=json.loads((args.validation/matched['trial']/'cone_audit.json').read_text())['baseline']
assert old_layout.keys()==new_layout.keys() and all(math.hypot(old_layout[n][0]-new_layout[n][0],old_layout[n][1]-new_layout[n][1])<.001 for n in old_layout)
prefix_before=next(e['stamp'] for e in br['events'] if e['state']=='ARRIVED' and e['detail']=='bend8')-next(e['stamp'] for e in br['events'] if e['state']=='MOVING')
summary=dict(scope='Official ODE, approved low COM; untimed control only. Images and speech omitted.',
    baseline_ref='fast-before-continuous-v1',baseline=baseline,continuous_only_prefix=prefix,
    prefix_before_sim_seconds=prefix_before,prefix_reduction_pct=100*(1-prefix['moving_sim_seconds']/prefix_before),
    matched_seed42_reduction_pct=100*(1-matched['moving_sim_seconds']/baseline['moving_sim_seconds']),
    validation=final,route_attempts=3,route_passes=batch['passed'],
    identical_navigation_across_three_layouts=True,physics_equal_to_baseline=True,navigation_hashes=navigation_hashes[0],
    matched_seed42_layout_verified=True,
    unchanged_low_com_model_sha256=baseline_model_hash,
    limitation='Three layouts, one final run each; no guarantee for arbitrary layouts. Baseline contact stream was not recorded; its displacement and odometry are available. Baseline guard trace ended early; no complete baseline guard count is claimed.')
(args.output/'summary.json').write_text(json.dumps(summary,indent=2))
shutil.copyfile(args.fixture/'result.json',args.output/'contact_fixture.json')
for i,folder in enumerate(args.failed_attempt):
    shutil.copyfile(folder/'result.json',args.output/f'development_failure_{i}.json')
print(json.dumps(dict(passed=batch['passed'],matched_reduction_pct=summary['matched_seed42_reduction_pct'],prefix_reduction_pct=summary['prefix_reduction_pct']),indent=2))
