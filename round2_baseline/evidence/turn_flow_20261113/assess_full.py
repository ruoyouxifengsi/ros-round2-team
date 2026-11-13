import gzip,json,math,sys,yaml
from pathlib import Path
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'scripts'))
from finish_region import contained
polygon=yaml.safe_load((root/'config/navigation.yaml').read_text())['local_costmap']['footprint']
for name in sys.argv[1:]:
    folder=Path(__file__).parent/name
    r=json.loads((folder/'result.json').read_text())
    t=json.loads((folder/'trace.json').read_text()) if (folder/'trace.json').exists() else json.loads(gzip.decompress((folder/'trace.json.gz').read_bytes()))
    events=r.get('events',t['events']);moving=[e for e in events if e['state'] in ('MOVING','CONTROL_ROUTE_DONE','CANCELLED','FAILED')]
    contacts=[json.loads(s) for s in (folder/'contacts.jsonl').read_text().splitlines()]
    live=[s for s in contacts if s['type']=='summary' and s['messages']>0]
    p=r.get('final_pose');c=r.get('contacts',{})
    blocks=next((e for e in events if e['state']=='ARRIVED' and e['detail']=='blocks'),None)
    b=min(t['poses'],key=lambda p:abs(p['t']-blocks['stamp'])) if blocks else None
    stopped=lambda p:bool(p and math.hypot(*p['v'][:2])<.02 and abs(p['v'][2])<.05)
    delta=max((math.dist(r['cones_before'][n][:2],pos[:2]) for n,pos in r.get('cones_after',{}).items()),default=999.)
    checks=dict(completed=r.get('route_done',False),contact_coverage=bool(live and live[0]['sim']<=moving[0]['stamp'] and live[-1]['sim']>=moving[-1]['stamp']),
        zero_contacts=c.get('cone_contact_events',-1)==0 and c.get('wall_contact_events',-1)==0,
        no_tilt=r.get('max_tilt_deg',99)<3,zero_lateral=r.get('max_lateral_command',99)<1e-6,
        cones_unchanged=delta<.005,finish_contained=bool(p and contained(p['x'],p['y'],p['yaw'],polygon,margin=0.)),finish_stopped=stopped(p),
        blocks_stopped=stopped(b),blocks_pose=bool(b and math.hypot(b['x']+1.75,b['y']-2.25)<.0301 and abs(math.atan2(math.sin(b['yaw']-1.57),math.cos(b['yaw']-1.57)))<.25),
        cleanup=not r.get('cleanup',{}).get('remaining',[1]))
    qr=next((e for e in events if e['state']=='ARRIVED' and e['detail']=='qr'),None)
    q=min(t['poses'],key=lambda p:abs(p['t']-qr['stamp'])) if qr else None
    def qr_inside(p):
        if not p:return False
        c,s=math.cos(p['yaw']),math.sin(p['yaw'])
        return all(0<=p['x']+c*x-s*y<=.5 and 0<=p['y']+s*x+c*y<=.5 for x,y in polygon)
    checks.update(qr_contained=qr_inside(q),qr_stopped=stopped(q),under_600_wall=r.get('wall_seconds',999)<600)
    import xml.etree.ElementTree as ET
    expected=[p[0] for p in yaml.safe_load(ET.parse(root/'launch/control.launch').find('.//rosparam[@param="route"]').text)]
    checks['ordered_route']=[e['detail'] for e in events if e['state'] in ('PASSED','ARRIVED')]==expected
    summary=dict(name=name,accepted=all(checks.values()),checks=checks,sim_seconds=r.get('sim_seconds'),wall_seconds=r.get('wall_seconds'),
        stages={a['detail']:round(b['stamp']-a['stamp'],3) for a,b in zip(moving,moving[1:])},
        max_cone_displacement=delta,max_tilt_deg=r.get('max_tilt_deg'),contacts=c,final_pose=p)
    (folder/'assessment.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
