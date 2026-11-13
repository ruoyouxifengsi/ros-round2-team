"""Finish acceptance uses a contained, stopped footprint rather than a point goal."""
import math

BOX=(-.5,0.,2.,2.5)

def contained(x,y,yaw,polygon,margin=.01,box=BOX):
    c,s=math.cos(yaw),math.sin(yaw)
    return bool(polygon) and all(box[0]+margin<=x+c*px-s*py<=box[1]-margin and
                                box[2]+margin<=y+s*px+c*py<=box[3]-margin for px,py in polygon)

def can_stop_inside(x,y,yaw,velocity,polygon,acceleration=.8,box=BOX):
    if acceleration<=0 or not all(math.isfinite(v) for v in (x,y,yaw,*velocity,acceleration)):
        return False
    vx,vy,omega=velocity
    for i in range(200):
        if not contained(x,y,yaw,polygon,box=box):return False
        if i>=5:
            speed=math.hypot(vx,vy)
            if speed<1e-5:return True
            factor=max(0.,1.-acceleration*.02/speed)
            vx*=factor;vy*=factor;omega=0.
        x+=(vx*math.cos(yaw)-vy*math.sin(yaw))*.02
        y+=(vx*math.sin(yaw)+vy*math.cos(yaw))*.02
        yaw+=omega*.02
    return False
