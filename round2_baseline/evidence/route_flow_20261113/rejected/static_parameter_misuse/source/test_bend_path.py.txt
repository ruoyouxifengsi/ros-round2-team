import math
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from bend_path import rounded_path,path_command,braking_clear,validate_settings

route=yaml.safe_load(ET.parse(Path(__file__).parents[1]/'launch/control.launch').find('.//rosparam[@param="route"]').text)[:9]
points=[(-.75,1.75)]+[(x,2.72 if name in ('bend1','bend2') else y) for name,x,y,_ in route]
path=rounded_path(points)
assert path[0][:2]==list(points[0]) and path[-1][:2]==list(points[-1])
assert all(math.dist(a[:2],b[:2])<=.025001 for a,b in zip(path,path[1:]))
assert all(min(math.dist(p[:2],v) for p in path)<.08 for v in points)
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
