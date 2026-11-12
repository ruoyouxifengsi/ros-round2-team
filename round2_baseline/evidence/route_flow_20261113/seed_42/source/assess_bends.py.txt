"""Compare the same start-to-bend8 stopped prefix, never a full-race score."""
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml

def assess(folder):
    read=lambda name:json.loads((folder/name).read_text())
    result=read('result.json');poses=read('poses.json');events=result['events']
    route=yaml.safe_load(ET.parse(folder/'launch/control.launch.txt').find('.//rosparam[@param="route"]').text)
    route=route[:next(i for i,p in enumerate(route) if p[0]=='bend8')+1]
    start=next(e for e in events if e['state']=='MOVING')['stamp']
    end=events[-1]['stamp'];samples=[p for p in poses if start<=p['t']<=end]
    arrivals=[e for e in events if e['state'] in ('PASSED','ARRIVED')]
    final=min(poses,key=lambda p:abs(p['t']-end))
    position_error=math.hypot(final['x']-route[-1][1],final['y']-route[-1][2])
    wrap=lambda a:math.atan2(math.sin(a),math.cos(a))
    yaw_error=abs(wrap(final['rpy'][2]-route[-1][3]))
    contacts=[json.loads(line) for line in (folder/'contacts.jsonl').read_text().splitlines()]
    summaries=[c for c in contacts if c['type']=='summary' and c['messages']>0]
    covered=bool(summaries and summaries[0]['sim']<=start and summaries[-1]['sim']>=end)
    tilt=max(math.degrees(math.acos(max(-1.,min(1.,math.cos(p['rpy'][0])*math.cos(p['rpy'][1]))))) for p in samples)
    gates=[]
    for event,point in zip(arrivals,route):
        pose=min(poses,key=lambda p:abs(p['t']-event['stamp']))
        gates.append(dict(name=point[0],error=math.hypot(pose['x']-point[1],pose['y']-point[2])))
    contact=summaries[-1] if summaries else {}
    trace=[json.loads(line) for line in (folder/'snapshots/trace.jsonl').read_text().splitlines()]
    active_trace=[p for p in trace if start<=p['t']<=end]
    stopped=math.hypot(*final['v'][:2])<.02 and abs(final['v'][2])<.05
    ok=bool(result['passed'] and [e['detail'] for e in arrivals]==[p[0] for p in route]
        and all(g['error']<.115 for g in gates) and position_error<=.03 and yaw_error<=.25
        and stopped and covered and contact.get('wall_contact_events',-1)==0
        and contact.get('cone_contact_events',-1)==0 and tilt<3.
        and not result.get('cleanup',{}).get('remaining',[1]))
    stages=[]
    for event in events:
        if event['state']!='MOVING':continue
        arrived=next((e for e in arrivals if e['detail']==event['detail']),None)
        stages.append(dict(name=event['detail'],sim_seconds=arrived['stamp']-event['stamp'] if arrived else None))
    row=dict(accepted=ok,moving_sim_seconds=end-start,max_tilt_deg=tilt,
        trigger_elapsed_sim=result.get('elapsed_sim'),
        max_speed_mps=max(math.hypot(*p['v'][:2]) for p in samples),
        max_yaw_rate_rps=max(abs(p['v'][2]) for p in samples),
        max_lateral_command=max((abs(p.get('nav_cmd',[0,0,0])[1]) for p in active_trace),default=0.),
        path_m=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(samples,samples[1:])),
        rotation_deg=sum(abs(math.degrees(wrap(b['rpy'][2]-a['rpy'][2]))) for a,b in zip(samples,samples[1:])),
        stationary_sim_seconds=sum(b['t']-a['t'] for a,b in zip(samples,samples[1:]) if math.hypot(*a['v'][:2])<.02 and abs(a['v'][2])<.05),
        retries=(folder/'control.log').read_text(errors='replace').count('trajectory is not feasible'),
        contact_coverage=covered,wall_contacts=contact.get('wall_contact_events'),cone_contacts=contact.get('cone_contact_events'),
        final_position_error_m=position_error,final_yaw_error_rad=yaw_error,stopped=stopped,gates=gates,stages=stages,
        scope='BEND PREFIX ONLY: identical start, ordered 10cm waypoint gates, bend8 stopped within 3cm / 0.25rad; no race-time claim')
    (folder/'bends.json').write_text(json.dumps(row,indent=2))
    return row

if __name__=='__main__':
    results={str(p):assess(Path(p)) for p in sys.argv[1:]}
    print(json.dumps(results,indent=2))
    raise SystemExit(0 if all(r['accepted'] for r in results.values()) else 1)
