"""Replay costmap footprint edge rasterization from saved sensor evidence."""
import json
import math
from pathlib import Path
import sys
import yaml

path=Path(sys.argv[1]);data=json.loads(path.read_text())
cfg=yaml.safe_load((path.parent.parent/'config/navigation.yaml').read_text())
poly=cfg['local_costmap']['footprint'];pad=cfg['local_costmap']['footprint_padding']
x,y=data['position'];yaw=data['rpy'][2];c,s=math.cos(yaw),math.sin(yaw)
poly=[(px+math.copysign(pad,px),py+math.copysign(pad,py)) for px,py in poly]
poly=[(x+c*px-s*py,y+s*px+c*py) for px,py in poly]
for name in ('local','global'):
    m=data[name];r=m['resolution'];ox,oy=m['origin'];hits=set()
    for a,b in zip(poly,poly[1:]+poly[:1]):
        x0,y0=int((a[0]-ox)/r),int((a[1]-oy)/r)
        x1,y1=int((b[0]-ox)/r),int((b[1]-oy)/r)
        dx,dy=abs(x1-x0),-abs(y1-y0);sx=1 if x0<x1 else -1;sy=1 if y0<y1 else -1;err=dx+dy
        while True:
            cost=m['data'][y0*m['width']+x0]
            if cost>=100 or cost<0:hits.add((round(ox+(x0+.5)*r,4),round(oy+(y0+.5)*r,4),cost))
            if x0==x1 and y0==y1:break
            e=2*err
            if e>=dy:err+=dy;x0+=sx
            if e<=dx:err+=dx;y0+=sy
    print(name,'pose',data['position'],'blocked edge cells',sorted(hits))
