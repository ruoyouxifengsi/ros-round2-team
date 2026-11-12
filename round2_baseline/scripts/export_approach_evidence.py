"""Export paired three-layout results without calling local practice a race score."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from check_entry_turn import entry_turn

p=argparse.ArgumentParser()
p.add_argument('output',type=Path)
p.add_argument('baseline',type=Path)
p.add_argument('validation',type=Path)
p.add_argument('--probe',type=Path,action='append',default=[])
p.add_argument('--failed-attempt',type=Path,action='append',default=[])
args=p.parse_args()
read=lambda path:json.loads(path.read_text())
digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
batch=read(args.validation/'summary.json')
assert batch['passed']==3 and len(batch['attempts'])==3
assert len({r['seed'] for r in batch['attempts']})==3
args.output.mkdir(parents=True,exist_ok=False)

def export(folder,name):
    target=args.output/name;target.mkdir()
    for file in ('result.json','analysis.json','practice.json','health.json','cone_audit.json','physics_properties.json','source_hashes.json','interventions.json','analysis_before_causal_fix.json','practice_before_causal_fix.json'):
        if (folder/file).exists():shutil.copyfile(folder/file,target/file)
    contacts=[json.loads(line) for line in (folder/'contacts.jsonl').read_text().splitlines()]
    live=[c for c in contacts if c['type']=='summary' and c['messages']>0]
    (target/'contacts.json').write_text(json.dumps(dict(first=live[0],last=live[-1],events=[c for c in contacts if c['type']=='contact_begin']),indent=2))
    result=read(folder/'result.json')
    starts=[e for e in result['events'] if e['state']=='MOVING']
    end=next(e for e in reversed(result['events']) if e['state'] in ('CONTROL_ROUTE_DONE','FAILED'))
    motion=dict(passed=result['passed'],scope=result.get('scope','FULL CONTROL ROUTE'),
        moving_sim_seconds=end['stamp']-starts[0]['stamp'],guard_trips=result.get('guard_max_trips'))
    if any(e['state']=='ARRIVED' and e['detail']=='cone_entry' for e in result['events']):
        motion['entry_turn']=entry_turn(result,read(folder/'poses.json'))
    (target/'motion.json').write_text(json.dumps(motion,indent=2))
    return motion

paired=[];hashes=[];models=[];physics=[]
for attempt in batch['attempts']:
    seed=attempt['seed'];name='seed_'+str(seed)
    old=args.baseline/name;new=args.validation/name
    before=read(old/'practice.json');after=read(new/'practice.json')
    assert before['accepted'] and after['accepted'] and after['efficient_approach']
    old_physics=read(old/'physics_properties.json');new_physics=read(new/'physics_properties.json')
    assert old_physics==new_physics
    assert digest(old/'low_com.urdf')==digest(new/'low_com.urdf')
    physics.append(new_physics);models.append(digest(new/'low_com.urdf'))
    old_layout=read(old/'cone_audit.json')['baseline'];new_layout=read(new/'cone_audit.json')['baseline']
    assert old_layout.keys()==new_layout.keys()
    assert all(sum((old_layout[n][i]-new_layout[n][i])**2 for i in (0,1))**.5<.001 for n in old_layout)
    source=read(new/'source_hashes.json')
    signature={name:source['scripts/'+name] for name in ('control.py','attitude_guard.py','cone_obstacles.py')}
    for directory in ('config','launch'):
        signature.update({str(path.relative_to(new)):digest(path) for path in (new/directory).glob('*') if path.is_file()})
    hashes.append(signature)
    old_motion=export(old,'before_'+name);new_motion=export(new,name)
    old_stages={s['stage']:s['sim'] for s in before['stages']}
    new_stages={s['stage']:s['sim'] for s in after['stages']}
    paired.append(dict(seed=seed,before_sim_seconds=before['moving_sim_seconds'],after_sim_seconds=after['moving_sim_seconds'],
        reduction_pct=100*(1-after['moving_sim_seconds']/before['moving_sim_seconds']),
        before_entry_turn=old_motion['entry_turn'],after_entry_turn=new_motion['entry_turn'],
        stage_comparison={stage:dict(before=old_stages[stage],after=new_stages[stage]) for stage in old_stages},
        accepted=after['accepted'],guard_trips=after['guard_trips'],contact_events=after['contact_summary']['cone_contact_events'],
        max_tilt_deg=after['max_tilt_deg'],retries=after['retries']))
assert all(h==hashes[0] for h in hashes)
assert len(set(models))==1 and all(v==physics[0] for v in physics)
probes={folder.name:export(folder,folder.name[6:] if folder.name.startswith('local_') else folder.name) for folder in args.probe}
failures=[]
for i,folder in enumerate(args.failed_attempt):
    motion=export(folder,'development_failure_'+str(i));failures.append(motion)
    snapshots=sorted((folder/'snapshots').glob('snapshot_*.json'))
    if snapshots:
        snapshot=read(snapshots[-1])
        fixture={key:snapshot[key] for key in ('t','position','rpy','local','status')}
        (args.output/('development_failure_'+str(i))/'entry_snapshot.json').write_text(json.dumps(fixture))
summary=dict(scope='Official ODE, approved low COM; untimed control only, images and speech omitted.',
    baseline_ref='continuous-before-docking-v2',route_attempts=3,route_passes=3,paired=paired,
    average_before_sim_seconds=sum(r['before_sim_seconds'] for r in paired)/3,
    average_after_sim_seconds=sum(r['after_sim_seconds'] for r in paired)/3,
    identical_navigation_across_three_layouts=True,navigation_hashes=hashes[0],
    physics_equal_to_baseline=True,matched_layouts_verified=True,unchanged_low_com_model_sha256=models[0],
    development_probes=probes,development_failures=failures,development_failure_count=len(failures),
    reassessment=batch.get('reassessment'),
    assessment_source_hashes={name:digest(Path(__file__).parent/name) for name in ('assess_practice.py','analyze_trial.py','arrival_evidence.py','check_entry_turn.py','trial_health.py')},
    limitation='Three layouts, one final run each; paired layout positions are equal but planner execution varies. Probes use debug starting poses and are not full-route passes. The first finish probe was rejected for slower motion. The first entry probe was superseded by move-before-turn after full-route boundary clearance failure.')
(args.output/'summary.json').write_text(json.dumps(summary,indent=2))
if (args.validation/'summary_before_causal_fix.json').exists():
    shutil.copyfile(args.validation/'summary_before_causal_fix.json',args.output/'summary_before_causal_fix.json')
print(json.dumps({k:summary[k] for k in ('route_passes','average_before_sim_seconds','average_after_sim_seconds')},indent=2))
