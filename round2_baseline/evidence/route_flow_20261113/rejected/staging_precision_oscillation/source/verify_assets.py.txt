"""Read-only verification of official runtime assets and measured footprint."""
import hashlib
import json
from pathlib import Path
import yaml

root=Path(__file__).resolve().parents[1]
original=root.parent/'第二次分站赛下发/src'
import os
deployed=Path(os.environ.get('ROUND2_WORKSPACE',str(Path.home()/'round2_team_ws')))/'src'
names=['car3/urdf/car3.urdf','car3/world/round2.world','car3/scripts/random_cones.py','car3/scripts/random_place.py','ucar_accumtimer/scripts/AccumTimer.py','ucar_accumtimer/config/AccumTimer.yaml']
checks={}
for name in names:
    a=hashlib.sha256((original/name).read_bytes()).hexdigest()
    b=hashlib.sha256((deployed/name).read_bytes()).hexdigest()
    checks[name]=dict(original=a,deployed=b,unchanged=a==b)
measurement=json.loads((root/'evidence/footprint_measurement.json').read_text())
polygon=yaml.safe_load((root/'config/navigation.yaml').read_text())['local_costmap']['footprint']
contained=True
for a,b in zip(polygon,polygon[1:]+polygon[:1]):
    for x,y in measurement['footprint']:
        contained &= (b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])>=-1e-9
result=dict(official_assets=checks,all_collision_mesh_hull_contained=contained)
(root/'evidence/asset_verification.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
raise SystemExit(0 if contained and all(c['unchanged'] for c in checks.values()) else 1)
