"""Archive accepted bend-only comparisons with matching runtime/configuration checks."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
from assess_bends import assess

p=argparse.ArgumentParser()
p.add_argument('baseline',type=Path)
p.add_argument('trials',nargs='+',type=Path)
p.add_argument('--output',required=True,type=Path)
a=p.parse_args()
read=lambda path:json.loads(path.read_text())
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
rows=[assess(folder) for folder in [a.baseline]+a.trials]
assert all(row['accepted'] for row in rows),'Only accepted bend prefixes may be exported as final results'
runtime=('scripts/control.py','scripts/attitude_guard.py','scripts/bend_path.py','scripts/cone_obstacles.py')
hashes=[read(folder/'source_hashes.json') for folder in a.trials]
assert all(all(h[k]==hashes[0][k] for k in runtime) for h in hashes),'Runtime changed between final repeats'
configs=[{str(p.relative_to(folder)):sha(p) for sub in ('config','launch') for p in (folder/sub).iterdir() if p.is_file()} for folder in a.trials]
assert all(c==configs[0] for c in configs),'Configuration changed between final repeats'
physics=read(a.baseline/'physics_properties.json');model_hash=sha(a.baseline/'low_com.urdf')
for folder in a.trials:
    assert read(folder/'physics_properties.json')==physics,'Physics parameters differ'
    assert sha(folder/'low_com.urdf')==model_hash,'Low COM vehicle differs'
    assert read(folder/'result.json')['attitude_protection'] is False
    assert assess(folder)['max_lateral_command']==0.,'A final bend run used sideways commands'
a.output.mkdir(parents=True,exist_ok=False)
for index,folder in enumerate([a.baseline]+a.trials):
    target=a.output/('baseline' if index==0 else 'optimized_'+str(index));target.mkdir()
    for name in ('result.json','bends.json','poses.json','contacts.jsonl','physics_properties.json','low_com.urdf','source_hashes.json','control.log'):
        shutil.copyfile(folder/name,target/name)
    for name in ('source','config','launch'):
        shutil.copytree(folder/name,target/name)
    (target/'snapshots').mkdir()
    shutil.copyfile(folder/'snapshots/trace.jsonl',target/'snapshots/trace.jsonl')
average=sum(r['moving_sim_seconds'] for r in rows[1:])/len(a.trials)
summary=dict(baseline=rows[0],optimized=rows[1:],optimized_mean_sim_seconds=average,
    reduction_percent=100*(1-average/rows[0]['moving_sim_seconds']),
    same_physics=True,same_low_com_model=True,same_optimized_runtime_and_configuration=True,
    low_com_sha256=model_hash,optimized_runtime_sha256={k:hashes[0][k] for k in runtime},
    scope='Only the fixed bend prefix was repeated. No cone layouts, image tasks, full round or official timer validation.',
    comparison='Same official start, ordered original waypoint gates, stopped bend8 within 3cm and 0.25rad. Movement clock starts at first MOVING; trigger elapsed is also retained.')
(a.output/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(dict(mean=average,reduction_percent=summary['reduction_percent'],repeats=len(a.trials))))
