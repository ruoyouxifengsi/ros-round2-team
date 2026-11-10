"""Verify ordered route and conservative body containment from odometry evidence."""
import json
import math
import sys
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET

folder=Path(sys.argv[1]);result=json.loads((folder/'result.json').read_text());poses=json.loads((folder/'poses.json').read_text())
footprint=yaml.safe_load((folder/'config/navigation.yaml').read_text())['local_costmap']['footprint']
def bounds(p):
    c,s=math.cos(p['rpy'][2]),math.sin(p['rpy'][2]);xy=[(p['x']+c*x-s*y,p['y']+s*x+c*y) for x,y in footprint]
    return [min(x for x,y in xy),max(x for x,y in xy),min(y for x,y in xy),max(y for x,y in xy)]
def contained(b,box):return b[0]>=box[0] and b[1]<=box[1] and b[2]>=box[2] and b[3]<=box[3]
arrivals=[e for e in result['events'] if e.get('state')=='ARRIVED']
launch_file=folder/'launch/control.launch.txt'
if not launch_file.exists():launch_file=Path(__file__).parents[1]/'launch/control.launch'
expected=[row[0] for row in yaml.safe_load(ET.parse(launch_file).find('.//rosparam[@param="route"]').text)]
checks={}
for event in arrivals:
    p=min(poses,key=lambda v:abs(v['t']-event['stamp']));checks[event['detail']]=dict(pose=[p['x'],p['y'],p['rpy'][2]],body_bounds=bounds(p),stopped=math.hypot(*p['v'][:2])<.02 and abs(p['v'][2])<.05)
if 'qr' in checks:checks['qr']['body_in_region']=contained(checks['qr']['body_bounds'],[0,.5,0,.5])
if 'finish' in checks:checks['finish']['body_in_region']=contained(checks['finish']['body_bounds'],[-.5,0,2,2.5])
summary=dict(control_done=result['passed'],ordered_stages_pass=[e['detail'] for e in arrivals]==expected,expected_stages=expected,arrivals=checks,elapsed_wall=result.get('elapsed_wall'),max_tilt_deg=max(math.degrees(math.acos(max(-1,min(1,math.cos(p['rpy'][0])*math.cos(p['rpy'][1]))))) for p in poses),scope='Odometry and official URDF geometry; not Gazebo truth or a collision-free certificate')
timer=json.loads((folder/'timer/timer.json').read_text()) if (folder/'timer/timer.json').exists() else {}
summary['official_start']=result.get('trigger')=='unchanged official Draw widget' and timer.get('draw_invocations')==1
summary['random_scene_prepared']=all((folder/(name+'.log')).exists() for name in ('random_cones.py','random_place.py'))
summary['within_10_min_after_trigger']=result.get('elapsed_wall',float('inf'))<600
summary['control_baseline_pass']=bool(summary['control_done'] and summary['ordered_stages_pass'] and checks.get('qr',{}).get('body_in_region') and checks.get('finish',{}).get('body_in_region') and checks.get('finish',{}).get('stopped') and summary['official_start'] and summary['random_scene_prepared'] and summary['within_10_min_after_trigger'] and not result.get('scope','').startswith('DEBUG') and not (folder/'interventions.json').exists())
(folder/'analysis.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
