"""Exercise straight docking through a temporary map outage and a permanent outage."""
import threading
from types import SimpleNamespace as NS
from unittest.mock import patch
import control as m

def run(recover):
    clock=[10.];commands=[]
    c=m.Control.__new__(m.Control);c.navigation_frame='map';c.cancelled=threading.Event()
    c.fast=True;c.efficient=True;c.client=NS(cancel_all_goals=lambda:None)
    c.received=10.;c.map_received=7.;c.budget_time=lambda:clock[0]
    c.odom=NS(pose=NS(pose=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
    c.costmap=NS(header=NS(frame_id='map'),info=NS(resolution=.025,width=80,height=80,origin=NS(position=NS(x=-1.,y=-1.))),data=[0]*6400)
    def publish(msg):
        command=(msg.linear.x,msg.linear.y,msg.angular.z);commands.append(command)
        if any(command):
            assert clock[0]-c.map_received<=2,'Never move with an old map'
            c.odom.pose.pose.position=NS(x=.3,y=0.)
    c.velocity=NS(publish=publish)
    def sleep(dt):
        clock[0]+=dt;c.received=clock[0]
        if recover and clock[0]>=10.2:c.map_received=clock[0]
    params={'/move_base/local_costmap/footprint':[[.2,.1],[-.2,.1],[-.2,-.1],[.2,-.1]],'/move_base/local_costmap/footprint_padding':.005}
    with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]),patch.object(m.time,'sleep',side_effect=sleep), \
         patch.object(m.rospy,'get_time',side_effect=lambda:clock[0]), \
         patch.object(m.rospy,'is_shutdown',return_value=False),patch.object(m.rospy,'get_param',side_effect=lambda key,default=None:params.get(key,default)), \
         patch.object(m.rospy,'logwarn'),patch.object(m.rospy,'loginfo'):
        if recover:
            c.straight_move(.3,0.,0.,40.,1,.025)
            assert len(commands)>=6 and not any(any(v) for v in commands[:4]) and any(any(v) for v in commands)
        else:
            try:c.straight_move(.3,0.,0.,40.,1,.025)
            except RuntimeError:pass
            else:raise AssertionError('Permanent map outage must fail')
            assert not any(any(v) for v in commands) and clock[0]<=15.1
run(True);run(False)
print('PASS: real docking holds zero during stale map, resumes after recovery, aborts a permanent outage')
