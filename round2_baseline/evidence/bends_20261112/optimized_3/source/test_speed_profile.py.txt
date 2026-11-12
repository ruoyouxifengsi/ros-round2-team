#!/usr/bin/env python3
"""Check fast straight convergence and curvature-aware command scaling."""
import math
from control import straight_velocity
from attitude_guard import Guard

for direction in (-1,1):
    for heading in (0.,math.pi/2):
        x,y=-.02*math.sin(heading),.02*math.cos(heading);a=heading+.1
        gx,gy=direction*math.cos(heading),direction*math.sin(heading)
        for step in range(3000):
            if math.hypot(gx-x,gy-y)<.012:break
            v,w,remaining=straight_velocity(x,y,a,gx,gy,heading,direction,True)
            assert abs(v)<=.18 and abs(w)<=.15 and remaining>0
            assert v*v<=.5*remaining+1e-9
            x+=v*math.cos(a)*.02;y+=v*math.sin(a)*.02;a+=w*.02
        assert math.hypot(gx-x,gy-y)<.012

def settle(command):
    g=Guard(max_speed=.45,linear_accel=.5,curvature_accel=.12)
    for i in range(1,201):
        t=i*.05;g.observe(t,t,[0,0,0,1]);g.input(command,t,t);v=g.tick(t,t)
    return v
straight=settle([.4,0,0]);turn=settle([.4,0,.5])
assert abs(straight[0]-.4)<1e-9
assert turn[0]<straight[0] and math.hypot(*turn[:2])*abs(turn[2])<=.12+1e-9
assert abs(turn[2]/turn[0]-.5/.4)<1e-9
assert settle([0,0,0])==[0,0,0]
print('PASS: fast forward/reverse convergence, braking envelope, curvature scaling and stop')
