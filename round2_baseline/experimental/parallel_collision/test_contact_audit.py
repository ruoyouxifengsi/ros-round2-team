#!/usr/bin/env python3
"""Positive/negative collision-observer fixtures, separate from competition maps."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from trial_processes import stop_processes,wait_for_ports
p=argparse.ArgumentParser()
p.add_argument('output',type=Path)
p.add_argument('--library',type=Path,default=Path.home()/'round2_engine_experiment/build/libround2_parallel_collision.so')
args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
results=[]
for label,x,expected in [('overlap',.1,True),('separated',1.,False)]:
    folder=args.output/label;folder.mkdir()
    def box(name,x):
        return f'<model name="{name}"><pose>{x} 0 1 0 0 0</pose><link name="body"><inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial><collision name="collision"><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision></link></model>'
    world=folder/'fixture.world'
    world.write_text('<sdf version="1.6"><world name="default"><physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>'+box('car3',0)+box('construction_cone_probe',x)+'</world></sdf>')
    env=dict(os.environ,GAZEBO_MASTER_URI='http://127.0.0.1:11359',ROUND2_COLLISION_THREADS='4',OMP_WAIT_POLICY='PASSIVE',ROUND2_COLLISION_VERIFY='1',ROUND2_COLLISION_VERIFY_STRIDE='1',ROUND2_CONTACT_AUDIT=str((folder/'contacts.jsonl').resolve()))
    wait_for_ports((11359,))
    with (folder/'simulation.log').open('w') as log:
        proc=subprocess.Popen(['gzserver','--verbose','--iters','1200','-e','round2_parallel_ode','-s',str(args.library),str(world)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+35
            while time.monotonic()<deadline and proc.poll() is None:
                path=folder/'contacts.jsonl'
                if path.exists():
                    snapshots=[json.loads(line) for line in path.read_text().splitlines()]
                    if any(r.get('type')=='summary' and r['steps']>=1000 for r in snapshots):break
                time.sleep(.1)
        finally:cleanup=stop_processes([proc])
    records=[json.loads(line) for line in (folder/'contacts.jsonl').read_text().splitlines()]
    contacts=[r for r in records if r['type']=='contact_begin']
    summaries=[r for r in records if r['type']=='summary']
    passed=bool(contacts)==expected and bool(summaries) and summaries[-1]['steps']>=1000 and summaries[-1]['unsupported']==0 and not cleanup['remaining']
    results.append(dict(fixture=label,passed=passed,contact_begins=len(contacts),summary=summaries[-1] if summaries else None))
(args.output/'result.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
if not all(r['passed'] for r in results):raise SystemExit(1)
