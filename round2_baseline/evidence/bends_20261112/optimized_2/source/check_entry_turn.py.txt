"""Measure actual accumulated yaw, including turns across the +/-pi boundary."""
import argparse
import json
import math
from pathlib import Path

def entry_turn(result,poses):
    start=next(e['stamp'] for e in result['events'] if e['state']=='MOVING' and e['detail']=='cone_entry')
    end=next(e['stamp'] for e in result['events'] if e['state']=='ARRIVED' and e['detail']=='cone_entry')
    samples=[p for p in poses if start<=p['t']<=end]
    if len(samples)<2:raise ValueError('Entry odometry is missing')
    turn=sum(abs(math.atan2(math.sin(b['rpy'][2]-a['rpy'][2]),math.cos(b['rpy'][2]-a['rpy'][2]))) for a,b in zip(samples,samples[1:]))
    return dict(seconds=end-start,rotation_deg=math.degrees(turn))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--max-degrees',type=float,default=180)
    args=p.parse_args()
    row=entry_turn(json.loads((args.trial/'result.json').read_text()),json.loads((args.trial/'poses.json').read_text()))
    print(json.dumps(row,indent=2))
    raise SystemExit(0 if row['rotation_deg']<args.max_degrees else 1)
