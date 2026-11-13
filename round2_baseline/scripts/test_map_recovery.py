"""Exercise straight docking through a temporary map outage and a permanent outage."""
import threading
from types import SimpleNamespace as NS
from unittest.mock import patch
import control as m

def run(recover,frame,odom_gap=False):
    clock=[10.];commands=[]
    c=m.Control.__new__(m.Control);c.navigation_frame=frame;c.cancelled=threading.Event()
    c.fast=True;c.efficient=True;c.client=NS(cancel_all_goals=lambda:None)
    c.received=7. if odom_gap else 10.;c.map_received=10. if odom_gap else 7.;c.budget_time=lambda:clock[0]
    c.odom=NS(pose=NS(pose=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
    c.costmap=NS(header=NS(frame_id=frame),info=NS(resolution=.025,width=80,height=80,origin=NS(position=NS(x=-1.,y=-1.))),data=[0]*6400)
    def publish(msg):
        command=(msg.linear.x,msg.linear.y,msg.angular.z);commands.append(command)
        if any(command):
            assert clock[0]-c.map_received<=2,'Never move with an old map'
            assert clock[0]-c.received<=2,'Never move with old odometry'
            c.odom.pose.pose.position=NS(x=.3,y=0.)
    c.velocity=NS(publish=publish)
    def sleep(dt):
        clock[0]+=dt
        if not odom_gap or (recover and clock[0]>=10.2):c.received=clock[0]
        if odom_gap or (recover and clock[0]>=10.2):c.map_received=clock[0]
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
for frame in ('map','odom'):
    run(True,frame);run(False,frame)
    run(True,frame,True);run(False,frame,True)
print('PASS: real docking holds zero during stale map, resumes after recovery, aborts a permanent outage')
