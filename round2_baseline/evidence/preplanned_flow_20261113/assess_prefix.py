import json,math,sys,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[2]
polygon=yaml.safe_load((root/'config/navigation.yaml').read_text())['local_costmap']['footprint']
expected=[p[0] for p in yaml.safe_load(ET.parse(root/'launch/control.launch').find('.//rosparam[@param="route"]').text)][:11]
for name in sys.argv[1:]:
    folder=Path(__file__).parent/name;r=json.loads((folder/'result.json').read_text());p=r.get('final_pose',{})
    events=r.get('events',[]);moving=[e for e in events if e['state'] in ('MOVING','CONTROL_ROUTE_DONE')]
    rows=[json.loads(s) for s in (folder/'contacts.jsonl').read_text().splitlines()]
    live=[r for r in rows if r['type']=='summary' and r['messages']>0]
    def contained():
        if not p:return False
        c,s=math.cos(p['yaw']),math.sin(p['yaw'])
        return all(0<=p['x']+c*x-s*y<=.5 and 0<=p['y']+s*x+c*y<=.5 for x,y in polygon)
    checks=dict(completed=r.get('route_done',False),contacts=r.get('passed',False),
        coverage=bool(moving and live and live[0]['sim']<=moving[0]['stamp'] and live[-1]['sim']>=moving[-1]['stamp']),
        qr_contained=contained(),qr_pose=bool(p and math.hypot(p['x']-.2,p['y']-.25)<.011 and abs(p['yaw'])<.08),
        stopped=bool(p and math.hypot(*p['v'][:2])<.02 and abs(p['v'][2])<.05),
        order=[e['detail'] for e in events if e['state'] in ('PASSED','ARRIVED')]==expected,
        lateral=r.get('max_lateral_command',99)<1e-6,cleanup=not r.get('cleanup',{}).get('remaining',[1]))
    result=dict(name=name,accepted=all(checks.values()),checks=checks,sim_seconds=r.get('sim_seconds'),wall_seconds=r.get('wall_seconds'),
                max_tilt_deg=r.get('max_tilt_deg'),contacts=r.get('contacts'),final_pose=p)
    (folder/'assessment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
