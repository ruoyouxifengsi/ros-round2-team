"""Check the straight QR exit against the exact saved local map."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import yaml
from control import footprint_clear

path=Path(sys.argv[1]);d=json.loads(path.read_text());m=d['local']
cfg=yaml.safe_load((path.parent.parent/'config/navigation.yaml').read_text())['local_costmap']
grid=NS(info=NS(resolution=m['resolution'],width=m['width'],height=m['height'],origin=NS(position=NS(x=m['origin'][0],y=m['origin'][1]))),data=m['data'])
x,y=d['position'];yaw=d['rpy'][2];checks=[]
while x>=-.25:
    checks.append(dict(x=x,clear=footprint_clear(grid,x,y,yaw,cfg['footprint'],cfg['footprint_padding'])))
    x-=.01
print(json.dumps(dict(snapshot=str(path),samples=len(checks),all_clear=all(v['clear'] for v in checks),blocked=[v for v in checks if not v['clear']]),indent=2))
raise SystemExit(0 if all(v['clear'] for v in checks) else 1)
