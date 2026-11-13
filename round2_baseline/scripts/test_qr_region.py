"""Regression for the observed 11.5mm QR stall; retain footprint and heading checks."""
import math,threading
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch
import yaml
import control as motion
import route_flow

polygon=yaml.safe_load((Path(__file__).parents[1]/'config/navigation.yaml').read_text())['local_costmap']['footprint']
pose=N(position=N(x=.1970214324,y=.2388553431),orientation=N(x=0.,y=0.,z=math.sin(.04125212235/2),w=math.cos(.04125212235/2)))
c=motion.Control.__new__(motion.Control)
c.odom=N(pose=N(pose=pose),twist=N(twist=N(linear=N(x=0.,y=0.),angular=N(z=0.))))
c.fresh_costmap=lambda deadline:None
with patch.object(motion.rospy,'get_param',return_value=polygon):
    assert c.qr_ready_to_stop((.2,.25,0.),100.)
    c.odom.twist.twist.linear.x=2.
    assert not c.qr_ready_to_stop((.2,.25,0.),100.),'Must be able to brake within the box'
    c.odom.twist.twist.linear.x=0.
    pose.position.y=.22
    assert not c.qr_ready_to_stop((.2,.25,0.),100.),'Keep the 2cm viewpoint bound'
    pose.position.y=.2388553431
    pose.orientation.z=math.sin(.1/2);pose.orientation.w=math.cos(.1/2)
    assert not c.qr_ready_to_stop((.2,.25,0.),100.),'Heading must remain within 0.08rad'
    pose.orientation.z=0.;pose.orientation.w=1.
    with patch.object(motion.rospy,'get_param',return_value=[(.4,.1),(-.4,.1),(-.4,-.1),(.4,-.1)]):
        assert not c.qr_ready_to_stop((.2,.25,0.),100.),'A center point inside is insufficient'

events=[];cancelled=[];checks=[];sent=[]
c.manual_supervision=True;c.cancelled=threading.Event();c.navigation_frame='odom'
c.budget_time=lambda:10.;c.expired=lambda *args:False
c.report=lambda *event:events.append(event)
c.client=N(send_goal=lambda g:sent.append(g),cancel_all_goals=lambda:cancelled.append(True))
c.qr_ready_to_stop=lambda *args:checks.append(True) or True
c.stopped=lambda:True
with patch.object(route_flow.rospy.Time,'now',return_value=route_flow.rospy.Time(10)), \
     patch.object(route_flow.rospy,'get_time',return_value=10.),patch.object(route_flow.rospy,'is_shutdown',return_value=False):
    route_flow.navigate(c,N(update_configuration=lambda v:None),[['qr',.2,.25,0.]],100.,{})
assert len(checks)==2 and len(cancelled)==1 and events[-1]==('ARRIVED','qr')
events.clear();sent.clear();answers=iter((True,False,False))
c.qr_ready_to_stop=lambda *args:next(answers)
class NavigationResumed(Exception):pass
def active_goal():raise NavigationResumed()
c.client.get_state=active_goal
pose.position.x=.197;pose.position.y=.239
with patch.object(route_flow.rospy.Time,'now',return_value=route_flow.rospy.Time(10)), \
     patch.object(route_flow.rospy,'get_time',return_value=10.),patch.object(route_flow.rospy,'is_shutdown',return_value=False), \
     patch.object(route_flow.time,'sleep'):
    try:route_flow.navigate(c,N(update_configuration=lambda v:None),[['qr',.2,.25,0.]],100.,{})
    except NavigationResumed:pass
    else:raise AssertionError('Drift after braking must resume navigation')
assert len(sent)==2 and ('ARRIVED','qr') not in events
print('PASS: observed QR stall accepted only with bounded viewpoint, heading, full braking containment and post-stop recheck')
