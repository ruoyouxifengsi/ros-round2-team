"""Footprint checks and velocity calculations for the motion controllers."""

import math


def yaw_from_orientation(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def footprint_clear(grid, x, y, yaw, polygon, padding):
    r=grid.info.resolution;ox=grid.info.origin.position.x;oy=grid.info.origin.position.y
    c,s=math.cos(yaw),math.sin(yaw)
    vertices=[]
    for px,py in polygon:
        px+=math.copysign(padding,px);py+=math.copysign(padding,py)
        vertices.append((x+c*px-s*py,y+s*px+c*py))
    margin=r/math.sqrt(2)
    ix0=math.floor((min(p[0] for p in vertices)-margin-ox)/r)
    ix1=math.floor((max(p[0] for p in vertices)+margin-ox)/r)
    iy0=math.floor((min(p[1] for p in vertices)-margin-oy)/r)
    iy1=math.floor((max(p[1] for p in vertices)+margin-oy)/r)
    if ix0<0 or iy0<0 or ix1>=grid.info.width or iy1>=grid.info.height:return False
    edges=list(zip(vertices,vertices[1:]+vertices[:1]))
    xmin,xmax=min(p[0] for p in vertices),max(p[0] for p in vertices)
    ymin,ymax=min(p[1] for p in vertices),max(p[1] for p in vertices)
    half=r/2
    for iy in range(iy0,iy1+1):
        for ix in range(ix0,ix1+1):
            value=grid.data[iy*grid.info.width+ix]
            if 0<=value<100:continue
            px,py=ox+(ix+.5)*r,oy+(iy+.5)*r
            if px+half<xmin or px-half>xmax or py+half<ymin or py-half>ymax:continue
            separated=False
            for (ax,ay),(bx,by) in edges:
                dx,dy=bx-ax,by-ay
                if dx*(py-ay)-dy*(px-ax)+half*(abs(dx)+abs(dy))<0:
                    separated=True;break
            if not separated:return False
    return True


def short_turn_command(grid,x,y,angle,target,polygon,padding,angular_limit=.5,tolerance=.12):
    error=math.atan2(math.sin(target-angle),math.cos(target-angle))
    if abs(error)<tolerance:return 0.
    steps=max(1,math.ceil(abs(error)/.04))
    if not all(footprint_clear(grid,x,y,angle+error*i/steps,polygon,padding) for i in range(steps+1)):
        return None
    return max(-angular_limit,min(angular_limit,1.6*error))


def profiled_turn_velocity(error,previous,dt,limit,tolerance,acceleration=3.):
    speed=min(limit,math.sqrt(2*acceleration*max(0.,abs(error)-tolerance*.5)))
    target=math.copysign(speed,error)
    return previous+max(-acceleration*dt,min(acceleration*dt,target-previous))


def straight_velocity(px,py,angle,x,y,yaw,direction,fast=False,efficient=False):
    dx,dy=x-px,y-py
    cross=(py-y)*math.cos(yaw)-(px-x)*math.sin(yaw)
    target=yaw-direction*max(-.12,min(.12,(4. if efficient and fast else 2.)*cross))
    error=math.atan2(math.sin(target-angle),math.cos(target-angle))
    remaining=direction*(dx*math.cos(yaw)+dy*math.sin(yaw))
    distance = max(0., remaining)
    speed = min(.75 if direction > 0 else .45, 2.5 * distance, math.sqrt(1.6 * distance))
    v = direction * speed if abs(error) < (.15 if efficient and fast else .25) else 0.
    return v, max(-.8, min(.8, (3. if efficient and fast else 1.5) * error)), remaining
