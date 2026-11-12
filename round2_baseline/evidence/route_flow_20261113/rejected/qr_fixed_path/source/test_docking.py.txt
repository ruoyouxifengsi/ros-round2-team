"""Geometry regression checks for collision-gated QR reverse exit."""
from types import SimpleNamespace as NS
import math
from control import footprint_clear,straight_velocity

grid=NS(info=NS(resolution=.025,width=80,height=80,origin=NS(position=NS(x=-1.,y=-1.))),data=[0]*6400)
poly=[(.2,.1),(-.2,.1),(-.2,-.1),(.2,-.1)]
def cell(x,y,value):grid.data[int((y+1)/.025)*80+int((x+1)/.025)]=value
assert footprint_clear(grid,0,0,0,poly,.005)
cell(0,0,100)
assert not footprint_clear(grid,0,0,0,poly,.005)
cell(0,0,0);cell(.21,0,100)
assert not footprint_clear(grid,0,0,0,poly,.005)
assert footprint_clear(grid,0,0,math.pi/2,poly,.005)
cell(.21,0,0);cell(0,0,-1)
assert not footprint_clear(grid,0,0,0,poly,.005)
cell(0,0,0)
assert not footprint_clear(grid,.9,0,0,poly,.005)
cell(.23,0,100)
assert footprint_clear(grid,0,0,0,poly,.021)
cell(.23,0,0)
for direction in (-1,1):
    for heading in (0.,math.pi/2):
        x,y=-.02*math.sin(heading),.02*math.cos(heading);a=heading+.1
        gx,gy=direction*math.cos(heading),direction*math.sin(heading)
        for _ in range(3000):
            if math.hypot(gx-x,gy-y)<.012:break
            v,w,remaining=straight_velocity(x,y,a,gx,gy,heading,direction)
            assert abs(v)<=.08 and abs(w)<=.15 and remaining>0
            x+=v*math.cos(a)*.02;y+=v*math.sin(a)*.02;a+=w*.02
        assert math.hypot(gx-x,gy-y)<.012
print('PASS: footprint collisions, cell edges, unknown/outside map, forward/reverse line convergence in two headings')
