"""Check kinematic speed envelopes, caching and the real fixed/dynamic handoff."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from bend_path import PreparedPath,rounded_path

route=yaml.safe_load(ET.parse(Path(__file__).parents[1]/'launch/control.launch').find('.//rosparam[@param="route"]').text)[:11]
points=[(-.75,1.75)]+[(x,2.72 if name in ('bend1','bend2') else y) for name,x,y,_ in route]
plan=PreparedPath(rounded_path(points),1.,1.4,2.4)
assert plan.speeds[0]==0 and plan.speeds[-1]==0
for i,ds in enumerate(plan.steps):
    assert abs(plan.speeds[i+1]**2-plan.speeds[i]**2)<=2*1.4*ds+1e-8
assert all(0<v<=1. for v in plan.speeds[1:-1]),'Intermediate gates must not force stops'
x,y=points[0];yaw=math.pi/2;index=0;v=0.;visited=set()
for tick in range(6000):
    command,w,index=plan.command(index,x,y,yaw,v)
    v+=max(-1.6*.01,min(1.6*.01,command-v))
    x+=v*math.cos(yaw)*.01;y+=v*math.sin(yaw)*.01;yaw+=w*.01
    for i,p in enumerate(points):
        if math.dist((x,y),p)<.1:visited.add(i)
    if math.dist((x,y),points[-1])<.0095:break
assert len(visited)==len(points),(visited,x,y)
assert tick<3000 and math.dist((x,y),points[-1])<.0095,(tick,x,y)
assert abs(math.atan2(math.sin(yaw),math.cos(yaw)))<.15,yaw
print('PASS: cached envelope follows an obstacle-free geometric fixture in %.2fs; collision safety requires the integration checks'%(tick*.01))

from unittest.mock import patch
from types import SimpleNamespace as N
import control as motion
import route_flow
cfg=yaml.safe_load((Path(__file__).parents[1]/'config/route_flow.yaml').read_text())
tuning=yaml.safe_load((Path(__file__).parents[1]/'config/bend_tracking.yaml').read_text())
params={'~route':route,'~flow_settings':cfg,'~bend_settings':tuning,'~start_position':[-.75,1.75]}
c=motion.Control.__new__(motion.Control)
with patch.object(motion.rospy,'get_param',side_effect=lambda key,default=None:params.get(key,default)), \
     patch.object(route_flow,'prepare_fixed_paths',wraps=route_flow.prepare_fixed_paths) as build:
    prepared=c.fixed_plans()
    for _ in range(5):assert c.fixed_plans() is prepared
    assert build.call_count==1,'Ready checks must reuse the prepared geometry and speed profile'
    params['~start_position']=[-.74,1.75]
    assert c.fixed_plans() is not prepared and build.call_count==2
calls=[];events=[]
c=N(fixed_plans=lambda:prepared,follow_bends=lambda r,*a,**kw:calls.append((r,kw)),
    report=lambda *event:events.append(event),velocity=N(publish=lambda _:None))
navigation=[]
with patch.object(route_flow.rospy,'get_param',return_value=cfg),patch.object(route_flow.rospy,'set_param'), \
     patch.object(route_flow,'navigate',side_effect=lambda c,p,r,*a,**kw:navigation.append([v[0] for v in r])):
    route_flow.run_flow(c,None,route,100.)
assert len(calls)==1 and [p[0] for p in calls[0][0]]==[p[0] for p in route[:9]]
assert calls[0][1]['prepared'] is prepared['prefix'] and calls[0][1]['handoff']
assert navigation==[['qr_approach','qr']],'The official cone at (-.25,.75) must be avoided before QR as well'
assert events==[('IMAGE_SKIPPED','qr')]
print('PASS: ready cache reused and invalidated; fixed bends hand off before random cones, including the approach to QR')
