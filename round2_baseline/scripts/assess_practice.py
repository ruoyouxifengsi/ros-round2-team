"""Untimed development acceptance. Never an official race score."""
import json
import math
from pathlib import Path
import subprocess
import sys

folder=Path(sys.argv[1]);scripts=Path(__file__).parent
for script in ('analyze_trial.py','trial_health.py'):
    subprocess.run([sys.executable,str(scripts/script),str(folder)],stdout=subprocess.DEVNULL,check=True)
read=lambda name:json.loads((folder/name).read_text())
r=read('result.json');a=read('analysis.json');h=read('health.json');poses=read('poses.json')
audit=read('cone_audit.json')
contacts=[json.loads(line) for line in (folder/'contacts.jsonl').read_text().splitlines()] if (folder/'contacts.jsonl').exists() else []
summaries=[v for v in contacts if v['type']=='summary']
events=[e for e in r['events'] if e['state'] in ('MOVING','CONTROL_ROUTE_DONE','FAILED')]
start=next(e['stamp'] for e in events if e['state']=='MOVING');end=events[-1]['stamp']
live=[s for s in summaries if s['messages']>0]
coverage=bool(live and live[0]['sim']<=start and live[-1]['sim']>=end)
required=[n for n in a['expected_stages'] if not (r.get('continuous_bends',False) and n.startswith('bend'))]
stops=all(a['arrivals'].get(n,{}).get('stopped') for n in required)
ok=bool(r['passed'] and a['ordered_stages_pass'] and a['arrivals'].get('qr',{}).get('body_in_region') and
        a['arrivals'].get('finish',{}).get('body_in_region') and stops and coverage and
        summaries[-1]['cone_contact_events']==0 and len(audit['baseline'])==5 and not audit['moved_over_5mm'] and
        r.get('guard_max_trips')==0 and a['max_tilt_deg']<3 and not r.get('scope') and
        not r.get('cleanup',{}).get('remaining',[1]) and not (folder/'interventions.json').exists())
samples=[p for p in poses if start<=p['t']<=end]
stopped_sim=sum(max(0.,b['t']-p['t']) for p,b in zip(samples,samples[1:]) if math.hypot(*p['v'][:2])<.02 and abs(p['v'][2])<.05)
row=dict(trial=folder.name,accepted=ok,seed=r['seed'],speed_profile=r['speed_profile'],continuous=r.get('continuous_bends',False),
    moving_sim_seconds=end-start,stopped_sim_seconds=stopped_sim,max_tilt_deg=a['max_tilt_deg'],guard_trips=r.get('guard_max_trips'),
    retries=h['total_retries'],contact_coverage=coverage,contact_summary=summaries[-1] if summaries else None,
    moved_cones=audit['moved_over_5mm'],mandatory_stops=stops,stages=h['stages'],
    final_status=r.get('final_status'),cleanup=r.get('cleanup'),scope='UNTIMED CONTROL ONLY; images omitted; not official race acceptance')
(folder/'practice.json').write_text(json.dumps(row,indent=2));print(json.dumps(row,indent=2))
raise SystemExit(0 if ok else 1)
