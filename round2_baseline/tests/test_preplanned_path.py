"""Check kinematic speed envelopes, caching and the real fixed/dynamic handoff."""
import math
from pathlib import Path
import yaml
from bend_path import PreparedPath,advance_bend_passages,bend_passage_lines,rounded_path

route=yaml.safe_load((Path(__file__).parents[1]/'config/route.yaml').read_text())[:11]
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
params={'~route':route,'~bend_settings':dict(tuning, corner_radius=.16),'~start_position':[-.75,1.75]}
c=motion.Control.__new__(motion.Control)
with patch.object(motion.rospy,'get_param',side_effect=lambda key,default=None:params.get(key,default)), \
     patch.object(route_flow,'prepare_fixed_paths',wraps=route_flow.prepare_fixed_paths) as build:
    prepared=c.fixed_plans()
    for _ in range(5):assert c.fixed_plans() is prepared
    assert build.call_count==1,'Ready checks must reuse the prepared geometry and speed profile'
    params['~bend_settings'] = dict(tuning, corner_radius=.20)
    widened = c.fixed_plans()
    assert widened['prefix'].path != prepared['prefix'].path
    assert widened['prefix_route'] == prepared['prefix_route']
    params['~start_position']=[-.74,1.75]
    assert c.fixed_plans() is not widened and build.call_count==3

def time_to_handoff(plans):
    """Drive the production reference in free space and require every gate in order."""
    path = plans['prefix']
    gates = plans['prefix_route']
    x, y = path.path[0][:2]
    lines = bend_passage_lines(gates,(x,y))
    previous = (x,y)
    gate = 0
    yaw, speed, progress = math.pi / 2, 0., 0
    dt = .01
    for tick in range(3000):
        gate = advance_bend_passages(lines,gate,previous,(x,y))
        previous = (x,y)
        if gate == len(lines) and math.dist((x,y),gates[-1][1:3]) < .10:
            return tick * dt
        target, turn, progress = path.command(progress, x, y, yaw, speed)
        step = tuning['output_acceleration'] * dt
        speed += max(-step, min(step, target - speed))
        x += dt * speed * math.cos(yaw)
        y += dt * speed * math.sin(yaw)
        yaw += dt * turn
    raise AssertionError('Missed waypoint: ' + gates[gate][0])

baseline_time = time_to_handoff(prepared)
trial_time = time_to_handoff(widened)
print('Fixed bends in free space with configured radius and override: %.2fs, %.2fs' %
      (baseline_time, trial_time))

calls=[];events=[]
c=N(fixed_plans=lambda route:prepared,follow_bends=lambda r,*a,**kw:calls.append((r,kw)),
    prepare_qr=lambda:None,
    recognize=lambda stage:events.append(("recognize",stage)),
    report=lambda *event:events.append(event),velocity=N(publish=lambda _:None))
navigation=[]
with patch.object(route_flow.rospy,'get_param',return_value=cfg),patch.object(route_flow.rospy,'set_param'), \
     patch.object(route_flow,'navigate',side_effect=lambda c,p,r,*a,**kw:navigation.append([v[0] for v in r])):
    route_flow.run_flow(c,None,route,100.)
assert len(calls)==1 and [p[0] for p in calls[0][0]]==[p[0] for p in route[:9]]
assert calls[0][1]['prepared'] is prepared['prefix'] and calls[0][1]['handoff']
assert calls[0][1]['check_braking'] is False
assert navigation==[['qr_approach','qr']],'The official cone at (-.25,.75) must be avoided before QR as well'
assert events==[('recognize','qr')]
calls.clear()
route_flow.enter_cone_area(c,['cone_entry',-.25,.37,1.57],object(),100.)
assert calls[0][1].get('check_braking',True) is True
print('PASS: ready cache reused and invalidated; fixed bends hand off before random cones, including the approach to QR')
