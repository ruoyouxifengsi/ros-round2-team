"""Check map/scan consistency in official odometry coordinates, not independent ground truth."""
import argparse,json,math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
p=argparse.ArgumentParser();p.add_argument('survey',type=Path);p.add_argument('map',type=Path);a=p.parse_args()
cfg=yaml.safe_load(a.map.read_text());pixels=np.asarray(Image.open(a.map.parent/cfg['image']))
iy,ix=np.where(pixels<50);r=cfg['resolution'];ox,oy,_=cfg['origin']
occupied=np.column_stack((ox+(ix+.5)*r,oy+(pixels.shape[0]-iy-.5)*r));points=[]
for path in sorted((a.survey/'snapshots').glob('snapshot_*.json')):
    row=json.loads(path.read_text());s=row.get('scan')
    if not s or max(abs(v) for v in row['rpy'][:2])>math.radians(.7):continue
    q=s['rotation'];yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    rr=np.array(s['ranges']);ang=s['angle_min']+np.arange(len(rr))*s['angle_increment']+yaw
    valid=np.isfinite(rr)&(rr>.05)&(rr<5)
    points.extend(np.column_stack((s['position'][0]+rr*np.cos(ang),s['position'][1]+rr*np.sin(ang)))[valid][::5])
points=np.asarray(points)
distances=np.concatenate([np.sqrt(((occupied[i:i+128,None,:]-points[None,:,:])**2).sum(axis=2).min(axis=1)) for i in range(0,len(occupied),128)])
report=dict(wall_cell_distance_to_scan_p50=float(np.quantile(distances,.5)),wall_cell_distance_to_scan_p95=float(np.quantile(distances,.95)),fraction_wall_cells_within_5cm=float(np.mean(distances<.05)),scan_points=len(points),occupied_cells=len(occupied))
report['accepted']=report['fraction_wall_cells_within_5cm']>.95
(a.map.parent/'sensor_alignment.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if report['accepted'] else 1)
