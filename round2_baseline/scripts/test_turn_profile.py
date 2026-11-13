"""Run the real rotation loop against an accelerating yaw plant and fresh free space."""
import math
import threading
from types import SimpleNamespace as N
from unittest.mock import patch
import control as m

def run_turn(target,profiled):
    clock=[10.];angle=[0.];rate=[0.];commands=[]
    grid=N(info=N(resolution=.025,width=40,height=40,origin=N(position=N(x=-.5,y=-.5))),data=[0]*1600)
    polygon=[(.2,.1),(-.2,.1),(-.2,-.1),(.2,-.1)]
    c=m.Control.__new__(m.Control)
    c.client=N(cancel_all_goals=lambda:None);c.cancelled=threading.Event()
    c.expired=lambda now,deadline:now>deadline;c.budget_time=lambda:clock[0]
    c.fresh_costmap=lambda _:grid;c.stopped=lambda:True
    c.odom=N(pose=N(pose=N(position=N(x=0.,y=0.),orientation=N(x=0.,y=0.,z=0.,w=1.))),
             twist=N(twist=N(linear=N(x=0.,y=0.),angular=N(z=0.))))
    c.velocity=N(publish=lambda v:commands.append((clock[0],v.angular.z)))
    def advance(dt):
        demand=commands[-1][1]
        rate[0]+=max(-3.*dt,min(3.*dt,demand-rate[0]))
        angle[0]+=rate[0]*dt;clock[0]+=dt
        q=c.odom.pose.pose.orientation;q.z=math.sin(angle[0]/2);q.w=math.cos(angle[0]/2)
        c.odom.twist.twist.angular.z=rate[0]
    params={'/move_base/local_costmap/footprint':polygon,'/move_base/local_costmap/footprint_padding':.005,
            '~flow_settings/profiled_turns':profiled}
    with patch.object(m.rospy,'get_time',side_effect=lambda:clock[0]),patch.object(m.rospy,'is_shutdown',return_value=False), \
         patch.object(m.rospy,'get_param',side_effect=lambda key,default=None:params.get(key,default)),patch.object(m.time,'sleep',side_effect=advance):
        c.rotate_short(target,100.,2.4,.12)
    return clock[0]-10.,angle[0],commands

if __name__=='__main__':
    old,_,_=run_turn(math.pi,False)
    new,angle,commands=run_turn(math.pi,True)
    print('rotation loop: legacy %.3fs, profiled %.3fs'%(old,new),flush=True)
    assert new<old-.15,(old,new)
    assert abs(math.pi-angle)<.12
    for (t0,w0),(t1,w1) in zip(commands,commands[1:-1]):
        assert abs(w1-w0)<=3.*(t1-t0)+1e-8
    _,angle,_=run_turn(-math.pi,True)
    assert abs(-math.pi-angle)<.12
    print('PASS: faster half turn, bounded command acceleration, both turn directions')
