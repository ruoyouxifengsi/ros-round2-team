import math
from pathlib import Path
import yaml
from bend_path import advance_bend_passages,bend_passage_lines,rounded_path,path_command,braking_clear,validate_settings

route=yaml.safe_load((Path(__file__).parents[1]/'config/route.yaml').read_text())[:9]
points=[(-.75,1.75)]+[(x,2.72 if name in ('bend1','bend2') else y) for name,x,y,_ in route]
path=rounded_path(points)
assert path[0][:2]==list(points[0]) and path[-1][:2]==list(points[-1])
assert all(math.dist(a[:2],b[:2])<=.025001 for a,b in zip(path,path[1:]))
assert all(min(math.dist(p[:2],v) for p in path)<.08 for v in points)
straight_then_corner = [(0., 0.), (.5, 0.), (1., 0.), (1., 1.)]
wider = rounded_path(straight_then_corner, corner_radii={2: .20})
assert wider == rounded_path([straight_then_corner[i] for i in (0, 2, 3)], radius=.20)
assert wider != rounded_path(straight_then_corner, corner_radii={1: .20})
uturn = rounded_path([(1.25,1.70),(1.25,.73),(.75,.73),(.75,1.30)],
                     radius=.25, joined_corners=(1,2))
assert all(1e-9 < math.dist(a[:2],b[:2]) <= .025001 for a,b in zip(uturn,uturn[1:]))
assert min(math.hypot(x-1.,y-.73) for x,y,_ in uturn) < 1e-9
smooth = rounded_path([(0.,0.),(1.,0.),(1.,1.)], smooth_corners={1:(.24,.36)})
assert any(math.dist((x,y),(.76,0.))<1e-9 for x,y,_ in smooth)
assert any(math.dist((x,y),(1.,.36))<1e-9 for x,y,_ in smooth)
assert all(1e-9 < math.dist(a[:2],b[:2]) <= .025001 for a,b in zip(smooth,smooth[1:]))
passages = bend_passage_lines([('a',1.,0.),('b',2.,0.),('exit',3.,0.)], (0.,0.))
assert advance_bend_passages(passages,0,(.5,.19),(2.5,.19)) == 2
assert advance_bend_passages(passages,0,(.5,.26),(2.5,.26)) == 0
assert advance_bend_passages(passages,0,(1.5,0.),(.5,0.)) == 0
assert advance_bend_passages(passages,0,(1.5,0.),(2.5,0.)) == 0
x,y=points[0];index=0;vx=0.;yaw=math.pi/2;visited=set();dt=.01
for _ in range(10000):
    tx,omega,index=path_command(path,index,x,y,yaw,.5,.5,.6)
    vx+=max(-.65*dt,min(.65*dt,tx-vx))
    x+=vx*math.cos(yaw)*dt;y+=vx*math.sin(yaw)*dt;yaw+=omega*dt
    for i,p in enumerate([points[0]]+[(x,y) for _,x,y,_ in route]):
        if math.dist((x,y),p)<.10:visited.add(i)
    if math.dist((x,y),points[-1])<.012:break
assert len(visited)==len(points),(visited,x,y)
assert math.dist((x,y),points[-1])<.012
v,w,_=path_command([[0.,0.,0.],[1.,0.,0.]],1,.985,0.,0.,.7,.8,1.2)
assert v>0 and w==0,'Closest sample at endpoint must not force zero speed before arrival'
validate_settings(yaml.safe_load((Path(__file__).parents[1]/'config/bend_tracking.yaml').read_text()))
assert braking_clear(lambda x,y,a:True,0.,0.,0.,(.5,0.,0.),(.5,0.,0.),.65)
assert not braking_clear(lambda x,y,a:x<.1,0.,0.,0.,(.5,0.,0.),(.5,0.,0.),.65)
assert not braking_clear(lambda x,y,a:False,0.,0.,0.,(0.,0.,0.),(.5,0.,0.),.65)
for bad in ([(0.,0.),(0.,0.)],[(0.,0.),(1.,0.),(0.,0.)]):
    try:rounded_path(bad)
    except ValueError:pass
    else:raise AssertionError('Invalid path accepted')
print('PASS: dense path, ordered gate proximity, acceleration-limited tracking, braking collision checks, invalid geometry')
