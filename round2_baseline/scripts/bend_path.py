"""Build a dense rounded reference from the public course waypoints, not simulator geometry."""
import math
import bisect

BEND_GATE_HALF_WIDTH = .25


def bend_passage_lines(route, start):
    """Place intermediate lines across the incoming/outgoing direction bisectors."""
    points = [start] + [tuple(row[1:3]) for row in route]
    lines = []
    for before, center, after in zip(points, points[1:], points[2:]):
        dx, dy = center[0]-before[0], center[1]-before[1]
        ex, ey = after[0]-center[0], after[1]-center[1]
        incoming, outgoing = math.hypot(dx,dy), math.hypot(ex,ey)
        nx = dx/incoming + ex/outgoing if incoming and outgoing else dx+ex
        ny = dy/incoming + ey/outgoing if incoming and outgoing else dy+ey
        norm = math.hypot(nx,ny)
        if norm < 1e-9:
            raise ValueError('Bend passage line requires a forward direction')
        lines.append((*center,nx/norm,ny/norm))
    return lines


def advance_bend_passages(lines, index, previous, current):
    """Count forward crossings through each finite line in route order."""
    last_fraction = -1.
    for x,y,nx,ny in lines[index:]:
        px,py = previous[0]-x,previous[1]-y
        cx,cy = current[0]-x,current[1]-y
        before,after = px*nx+py*ny,cx*nx+cy*ny
        if not before < 0. <= after:
            break
        fraction = -before/(after-before)
        crossing_x = px+(cx-px)*fraction
        crossing_y = py+(cy-py)*fraction
        if fraction <= last_fraction or abs(-ny*crossing_x+nx*crossing_y)>BEND_GATE_HALF_WIDTH:
            break
        last_fraction = fraction
        index += 1
    return index

def validate_settings(settings):
    keys=('max_speed','acceleration','max_yaw_rate','output_speed','output_acceleration','output_yaw_rate')
    if not all(isinstance(settings.get(k),(int,float)) and math.isfinite(settings[k]) and settings[k]>0 for k in keys):
        raise ValueError('Bend tracking limits must be positive finite numbers')
    if any(settings[a]>settings[b] for a,b in zip(keys[:3],keys[3:])):
        raise ValueError('Bend tracking demands exceed output limits')

def smooth_corner(entry, exit_point, incoming, outgoing, entry_trim, exit_trim, spacing):
    """Tangent-matched quintic Bézier with zero curvature at the straight joins."""
    controls = [entry]
    controls.extend((entry[0]+f*entry_trim*incoming[0], entry[1]+f*entry_trim*incoming[1])
                    for f in (.20,.55))
    controls.extend((exit_point[0]-f*exit_trim*outgoing[0], exit_point[1]-f*exit_trim*outgoing[1])
                    for f in (.55,.20))
    controls.append(exit_point)
    count = max(12, math.ceil((entry_trim+exit_trim)/spacing)*4)
    dense = []
    distance = [0.]
    for j in range(count+1):
        t = j/count
        weights = [math.comb(5,i)*t**i*(1-t)**(5-i) for i in range(6)]
        point = tuple(sum(w*p[k] for w,p in zip(weights,controls)) for k in (0,1))
        if dense:
            distance.append(distance[-1]+math.dist(dense[-1],point))
        dense.append(point)
    count = math.ceil(distance[-1]/spacing)
    for j in range(1,count):
        along = distance[-1]*j/count
        i = bisect.bisect_right(distance,along)-1
        f = (along-distance[i])/(distance[i+1]-distance[i])
        yield tuple(a+(b-a)*f for a,b in zip(dense[i],dense[i+1]))
    yield exit_point

def rounded_path(points, radius=.16, spacing=.025, corner_radii=None, joined_corners=None, smooth_corners=None):
    if len(points)<2 or not (0 < radius < math.inf and 0 < spacing < math.inf) or not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('Bend reference requires finite points and positive spacing/radius')
    if any(math.dist(a,b)<1e-6 for a,b in zip(points,points[1:])):
        raise ValueError('Duplicate bend vertices')
    def unit(a,b):
        d=math.dist(a,b)
        return ((b[0]-a[0])/d,(b[1]-a[1])/d),d
    corner_radii = corner_radii or {}
    smooth_corners = smooth_corners or {}
    if any(not 0 < value < math.inf for value in corner_radii.values()):
        raise ValueError('Individual corner radii must be positive and finite')
    vertices=[0]
    for i in range(1,len(points)-1):
        u,_=unit(points[i-1],points[i]);v,_=unit(points[i],points[i+1])
        if u[0]*v[0]+u[1]*v[1]<.9999:vertices.append(i)
    vertices.append(len(points)-1);samples=[tuple(points[0])]
    def line(end):
        start=samples[-1]
        distance = math.dist(start,end)
        if distance <= 1e-9:
            return
        count=math.ceil(distance/spacing)
        samples.extend((start[0]+(end[0]-start[0])*j/count,start[1]+(end[1]-start[1])*j/count) for j in range(1,count+1))
    for i in range(1,len(vertices)-1):
        a,b,c=(points[j] for j in vertices[i-1:i+2]);u,la=unit(a,b);v,lb=unit(b,c)
        angle=math.acos(max(-1.,min(1.,u[0]*v[0]+u[1]*v[1])))
        if angle>math.pi-.01:raise ValueError('A reversing corner needs a separate maneuver')
        if vertices[i] in smooth_corners:
            entry_trim, exit_trim = smooth_corners[vertices[i]]
            if not 0 < entry_trim <= la/2 or not 0 < exit_trim <= lb/2:
                raise ValueError('Smooth-corner trims must fit in half of each adjoining segment')
            entry = (b[0]-u[0]*entry_trim,b[1]-u[1]*entry_trim)
            exit_point = (b[0]+v[0]*exit_trim,b[1]+v[1]*exit_trim)
            line(entry)
            samples.extend(smooth_corner(entry,exit_point,u,v,entry_trim,exit_trim,spacing))
            continue
        # Keep at least the middle third straight between two rounded corners.
        incoming = la / (2 if (vertices[i-1], vertices[i]) == joined_corners else 3)
        outgoing = lb / (2 if (vertices[i], vertices[i+1]) == joined_corners else 3)
        tangent = min(corner_radii.get(vertices[i], radius) * math.tan(angle / 2), incoming, outgoing)
        actual=tangent/math.tan(angle/2);sign=1 if u[0]*v[1]-u[1]*v[0]>0 else -1
        entry=(b[0]-u[0]*tangent,b[1]-u[1]*tangent)
        center=(entry[0]-sign*u[1]*actual,entry[1]+sign*u[0]*actual)
        line(entry);start=math.atan2(entry[1]-center[1],entry[0]-center[0])
        count=max(1,math.ceil(actual*angle/spacing))
        samples.extend((center[0]+actual*math.cos(start+sign*angle*j/count),center[1]+actual*math.sin(start+sign*angle*j/count)) for j in range(1,count+1))
    line(points[-1])
    return [[float(p[0]),float(p[1]),math.atan2(samples[min(i+1,len(samples)-1)][1]-samples[max(i-1,0)][1],samples[min(i+1,len(samples)-1)][0]-samples[max(i-1,0)][0])] for i,p in enumerate(samples)]

class PreparedPath:
    """Cache curvature limits and forward/backward speed reachability along arc length."""
    def __init__(self,path,max_speed,acceleration,max_yaw_rate,end_speed=0.,start_speed=0.,
                 terminal_heading_distance=0., lookahead_time=0., parking_tolerance=None):
        if len(path)<2 or not all(math.isfinite(v) for p in path for v in p):
            raise ValueError('A prepared path needs at least two finite poses')
        if not all(math.isfinite(v) and v>0 for v in (max_speed,acceleration,max_yaw_rate)) or not (0<=end_speed<=max_speed and 0<=start_speed<=max_speed):
            raise ValueError('Invalid prepared path limits')
        self.path=path;self.acceleration=acceleration;self.max_yaw_rate=max_yaw_rate;self.end_speed=end_speed
        self.terminal_heading_distance=terminal_heading_distance
        self.lookahead_time = lookahead_time
        self.parking_tolerance = parking_tolerance
        self.parking_deceleration = .60
        self.parking_latency = .06
        self.steps=[math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:])]
        if min(self.steps)<=1e-9:raise ValueError('Repeated prepared path positions')
        self.arc=[0.]
        for ds in self.steps:self.arc.append(self.arc[-1]+ds)
        self.speeds=[max_speed]*len(path)
        for i,ds in enumerate(self.steps):
            curvature=abs(math.atan2(math.sin(path[i+1][2]-path[i][2]),math.cos(path[i+1][2]-path[i][2])))/ds
            limit=min(max_speed,math.sqrt(acceleration/max(curvature,1e-9)),max_yaw_rate/max(curvature,1e-9))
            self.speeds[i]=min(self.speeds[i],limit);self.speeds[i+1]=min(self.speeds[i+1],limit)
        self.speeds[0]=min(self.speeds[0],start_speed);self.speeds[-1]=end_speed
        for i,ds in enumerate(self.steps):
            self.speeds[i+1]=min(self.speeds[i+1],math.sqrt(self.speeds[i]**2+2*acceleration*ds))
        for i in range(len(path)-2,-1,-1):
            self.speeds[i]=min(self.speeds[i],math.sqrt(self.speeds[i+1]**2+2*acceleration*self.steps[i]))

    def ready_to_park(self, x, y, yaw, velocity):
        target = self.path[-1][:2]
        if math.dist((x, y), target) > self.parking_tolerance or abs(velocity.angular.z) > .8:
            return False
        vx, vy = velocity.linear.x, velocity.linear.y
        speed = math.hypot(vx, vy)
        coast = self.parking_latency + speed / (2 * self.parking_deceleration)
        stopped = (x + coast * (vx * math.cos(yaw) - vy * math.sin(yaw)),
                   y + coast * (vx * math.sin(yaw) + vy * math.cos(yaw)))
        return math.dist(stopped, target) <= self.parking_tolerance * .7

    def command(self,progress,x,y,yaw,measured_speed):
        path=self.path;distance=lambda p:math.hypot(p[0]-x,p[1]-y)
        progress=min(range(progress,min(len(path),progress+30)),key=lambda i:distance(path[i]))
        if distance(path[progress])>.20:raise RuntimeError('Vehicle departed the prepared reference')
        target=min(len(path)-1,progress+6)
        if self.lookahead_time:
            lookahead = self.arc[progress] + max(0., measured_speed) * self.lookahead_time
            target = max(target, min(len(path) - 1, bisect.bisect_left(self.arc, lookahead)))
        dx,dy=path[target][0]-x,path[target][1]-y;length=math.hypot(dx,dy)
        forward=dx*math.cos(yaw)+dy*math.sin(yaw)
        if forward<0:raise RuntimeError('Forward tracker passed its prepared lookahead')
        lateral=-dx*math.sin(yaw)+dy*math.cos(yaw)
        curvature=2*lateral/max(length*length,1e-9)
        s=min(self.arc[-1],self.arc[progress]+.015+abs(measured_speed)*.08)
        i=min(len(path)-2,bisect.bisect_right(self.arc,s)-1)
        f=(s-self.arc[i])/self.steps[i]
        speed=math.sqrt(max(0.,(1-f)*self.speeds[i]**2+f*self.speeds[i+1]**2))
        remaining=max(distance(path[-1]),self.arc[-1]-self.arc[progress])
        speed=min(speed,math.sqrt(self.end_speed**2+2*self.acceleration*remaining))
        if not self.end_speed:
            # The nearest discrete sample may already be the endpoint while the car is still approaching.
            if s>=self.arc[-1]:speed=min(2.5*distance(path[-1]),math.sqrt(2*self.acceleration*distance(path[-1])))
            else:speed=min(speed,2.5*distance(path[-1]))
        if self.parking_tolerance is not None:
            # Solve d = v * latency + v² / (2 * deceleration) for speed.
            # This limit becomes active only near the stop and uses measured chassis braking.
            latency_term = self.parking_deceleration * self.parking_latency
            parking_speed = math.sqrt(latency_term**2 + 2 * self.parking_deceleration * remaining)
            speed = min(speed, parking_speed - latency_term)
        speed=min(speed,self.max_yaw_rate/max(abs(curvature),1e-9),math.sqrt(self.acceleration/max(abs(curvature),1e-9)))
        omega = speed * curvature
        if remaining < self.terminal_heading_distance:
            gx, gy, heading = path[-1]
            cross = (y - gy) * math.cos(heading) - (x - gx) * math.sin(heading)
            desired = heading - math.atan2(cross, .27)
            error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
            blend = min(1., 2 * (self.terminal_heading_distance - remaining) / self.terminal_heading_distance)
            omega = (1 - blend) * omega + blend * 3.6 * error
            omega = max(-self.max_yaw_rate, min(self.max_yaw_rate, omega))
        return speed,omega,progress


def path_command(path,progress,x,y,yaw,max_speed,acceleration,max_yaw_rate):
    distance=lambda p:math.hypot(p[0]-x,p[1]-y)
    progress=min(range(progress,min(len(path),progress+30)),key=lambda i:distance(path[i]))
    if distance(path[progress])>.20:raise RuntimeError('Vehicle departed the bend reference')
    target=min(len(path)-1,progress+6)
    dx,dy=path[target][0]-x,path[target][1]-y;length=math.hypot(dx,dy)
    speed=max_speed;along=0.
    for i in range(progress,min(len(path)-1,progress+40)):
        step=math.hypot(path[i+1][0]-path[i][0],path[i+1][1]-path[i][1])
        turn=abs(math.atan2(math.sin(path[i+1][2]-path[i][2]),math.cos(path[i+1][2]-path[i][2])))
        limit=min(max_speed,math.sqrt(acceleration*step/max(turn,1e-6)),max_yaw_rate*step/max(turn,1e-6))
        speed=min(speed,math.sqrt(limit*limit+2*acceleration*along));along+=step
    remaining=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path[progress:],path[progress+1:]))
    remaining=max(remaining,distance(path[-1]))
    speed=min(speed,math.sqrt(2*acceleration*remaining),2.5*distance(path[-1]))
    forward=dx*math.cos(yaw)+dy*math.sin(yaw)
    lateral=-dx*math.sin(yaw)+dy*math.cos(yaw)
    curvature=2*lateral/max(length*length,1e-9)
    speed=min(speed,max_yaw_rate/max(abs(curvature),1e-9))
    if forward<0:raise RuntimeError('Forward bend tracker passed its lookahead')
    return speed,speed*curvature,progress

def braking_clear(clear,x,y,yaw,measured,command,acceleration):
    # The output guard slews translation but zeros yaw immediately on stop.
    # Check immediate braking and 0.1s latency with present/new commands.
    for target in ((0.,0.,0.),measured,command):
        px,py,pa=x,y,yaw;vx,vy,omega=measured
        for i in range(150):
            if not clear(px,py,pa):return False
            if i>=5 or target==(0.,0.,0.):tx,ty,tw=0.,0.,0.
            else:tx,ty,tw=target
            delta=math.hypot(tx-vx,ty-vy);scale=min(1.,acceleration*.02/max(delta,1e-9))
            vx+=(tx-vx)*scale;vy+=(ty-vy)*scale
            omega=tw
            px+=(vx*math.cos(pa)-vy*math.sin(pa))*.02
            py+=(vx*math.sin(pa)+vy*math.cos(pa))*.02;pa+=omega*.02
            if i>=5 and math.hypot(vx,vy)<1e-5:break
    return True
