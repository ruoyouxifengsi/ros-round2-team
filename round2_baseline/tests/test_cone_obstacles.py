#!/usr/bin/env python3
"""Synthetic laser regression: detect moved layouts without seed/spawn input."""
import math
from cone_obstacles import detect

def scan(circles,tx=0,ty=0,yaw=0):
    values=[];step=2*math.pi/1440
    for i in range(1440):
        a=-math.pi+i*step+yaw;dx,dy=math.cos(a),math.sin(a);nearest=float('inf')
        for cx,cy in circles:
            x,y=cx-tx,cy-ty;b=x*dx+y*dy;disc=b*b-(x*x+y*y-.08**2)
            if disc>=0 and b>0:nearest=min(nearest,b-math.sqrt(disc))
        values.append(nearest)
    return detect(values,-math.pi,step,.02,10,tx,ty,yaw)

assert set(scan([(-.75,.75)]))=={(-.75,.75)}
assert set(scan([(-1.75,.25),(-1.25,1.75)]))=={(-1.75,.25),(-1.25,1.75)}
assert set(scan([(-.75,.75)],-.3,.25,.6))=={(-.75,.75)}
assert not scan([(-.25,.25)])  # official entrance is never a cone cell
assert not scan([(.75,.75)])
assert not scan([(-.25,1.75)])
assert not detect([float('inf')]*100,0,.01,.02,10,0,0,0)
print('PASS: multiple sensed layouts, rotated sensor, excluded entrance/outside region, empty scan')
