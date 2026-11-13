import math
import threading
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import yaml
import route_flow
import control as motion
from finish_region import contained,can_stop_inside

polygon=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/navigation.yaml').read_text())['local_costmap']['footprint']
assert contained(-.25,2.21,math.pi/2,polygon)
assert contained(-.25,2.25,0.,polygon),'No arbitrary final yaw requirement'
assert not contained(-.25,2.15,math.pi/2,polygon),'Rear wheels/body are not fully in'
assert not contained(-.25,2.33,math.pi/2,polygon),'Front would leave the finish region'
assert can_stop_inside(-.25,2.21,math.pi/2,(.15,0.,0.),polygon)
assert not can_stop_inside(-.25,2.21,math.pi/2,(.75,0.,0.),polygon),'Entry alone is insufficient at high speed'
assert not can_stop_inside(-.25,2.21,math.pi/2,(float('nan'),0.,0.),polygon)

# Exercise the coordinator, preserving task stops while removing staging stops.
cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/route_flow.yaml').read_text());cfg['continuous_regions']=True
cfg['preplanned_fixed']=False
calls=[];events=[]
control=NS(report=lambda *event:events.append(event),stopped=lambda:True,
    straight_move=lambda *args,**kw:calls.append(('straight',args,kw)),rotate_short=lambda *args,**kw:calls.append(('turn',args,kw)),
    can_rotate=lambda *args:False,
    follow_bends=lambda route,*args,**kw:calls.append(('entry',route,kw)),
    odom=NS(pose=NS(pose=NS(position=NS(x=-1.75,y=2.25),orientation=NS(x=0.,y=0.,z=1.,w=0.)))),
    velocity=NS(publish=lambda msg:None))
route=[['cone_entry',-.25,.37,1.57],['cone_exit',-1.75,2.2,1.57],['blocks',-1.75,2.25,1.57],
       ['return',-.7,1.2,0.],['finish_approach',-.25,1.25,1.57],['finish',-.25,2.2,1.57]]
with patch.object(route_flow.rospy,'get_param',return_value=cfg),patch.object(route_flow.rospy,'set_param'), \
     patch.object(route_flow,'navigate',side_effect=lambda c,p,r,d,s,**kw:calls.append(('navigate',[point[0] for point in r],kw))):
    route_flow.run_flow(control,None,route,100.)
assert calls[0][0]=='entry' and calls[0][2]['position_tolerance']==.04 and calls[0][2]['yaw_tolerance']==.25
assert calls[1][1]==['cone_exit','blocks']
assert calls[2][0]=='straight' and calls[3][0]=='turn'
assert calls[4][1]==['return','finish_approach'] and calls[4][2]['pass_last']
assert calls[5][0]=='straight' and calls[5][2]['finish_region']
assert events.count(('IMAGE_SKIPPED','blocks'))==1

# Clear turning space skips the retreat; a blocked sweep above retained it.
calls.clear();control.can_rotate=lambda *args:True
with patch.object(route_flow.rospy,'get_param',return_value=cfg),patch.object(route_flow.rospy,'set_param'), \
     patch.object(route_flow,'navigate',side_effect=lambda c,p,r,d,s,**kw:calls.append(('navigate',[point[0] for point in r],kw))):
    route_flow.run_flow(control,None,route[3:],100.)
assert calls[0][0]=='turn' and calls[0][2]['already_stopped']
assert calls[1][0]=='navigate' and calls[1][2]['pass_last']
assert len([c for c in calls if c[0]=='straight'])==1,'Only the final finish motion remains'

# Actual straight controller accepts a stopped region pose 7cm from the nominal goal.
pose=NS(position=NS(x=-.25,y=2.18),orientation=NS(x=0.,y=0.,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
events=[];cancelled=[]
control=NS(manual_supervision=False,cancelled=threading.Event(),received=10.,navigation_frame='odom',
    odom=NS(pose=NS(pose=pose),twist=NS(twist=NS(linear=NS(x=.1,y=0.),angular=NS(z=0.)))),
    client=NS(cancel_all_goals=lambda:cancelled.append(True)),fresh_costmap=lambda deadline:None,
    budget_time=lambda:10.,expired=lambda now,deadline:now>deadline,stopped=lambda:True,
    velocity=NS(publish=lambda msg:None),report=lambda *event:events.append(event))
with patch.object(motion.rospy,'get_param',side_effect=lambda key:polygon if key.endswith('/footprint') else 0.), \
     patch.object(motion.rospy,'get_time',return_value=10.), \
     patch.object(motion.rospy,'is_shutdown',return_value=False),patch.object(motion.time,'monotonic',return_value=10.):
    motion.Control.straight_move(control,-.25,2.25,1.57,100.,1,.025,finish_region=True)
    assert cancelled
    def roll_out():
        pose.position.y=2.4;control.cancelled.set();return True
    control.stopped=roll_out
    try:motion.Control.straight_move(control,-.25,2.25,1.57,100.,1,.025,finish_region=True)
    except RuntimeError:pass
    else:raise AssertionError('Must recheck containment after braking')
# Do not hand over to the straight controller while still turning across the corridor.
events=[];settings=[]
pose=NS(position=NS(x=-.25,y=1.20),orientation=NS(x=0.,y=0.,z=0.,w=1.))
class StillTurning(Exception):pass
def inspect_active_goal():raise StillTurning()
control=NS(manual_supervision=False,cancelled=threading.Event(),received=10.,navigation_frame='odom',
    odom=NS(pose=NS(pose=pose)),client=NS(send_goal=lambda goal:None,get_state=inspect_active_goal),
    budget_time=lambda:10.,expired=lambda now,deadline:now>deadline,report=lambda *event:events.append(event))
with patch.object(route_flow.rospy.Time,'now',return_value=route_flow.rospy.Time(10)), \
     patch.object(route_flow.rospy,'get_time',return_value=10.), \
     patch.object(route_flow.rospy,'is_shutdown',return_value=False),patch.object(route_flow.time,'monotonic',return_value=10.):
    def navigate():
        route_flow.navigate(control,NS(update_configuration=settings.append),[['finish_approach',-.25,1.25,math.pi/2]],100.,{'yaw_goal_tolerance':.12},pass_last=True)
    try:navigate()
    except StillTurning:pass
    else:raise AssertionError('A nearby but sideways vehicle must not switch to straight tracking')
    assert ('PASSED','finish_approach') not in events
    pose.orientation.z=math.sin(math.pi/4);pose.orientation.w=math.cos(math.pi/4)
    navigate()
    assert ('PASSED','finish_approach') in events and settings[-1]['free_goal_vel'] and settings[-1]['yaw_goal_tolerance']==.12
print('PASS: full region containment, stopping envelope, post-brake recheck, heading-gated handoff and task sequencing')
