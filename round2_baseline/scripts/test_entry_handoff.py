"""An aligned, stopped entry must not rebuild its map or repeat standstill waits."""
import threading,time
from types import SimpleNamespace as N
from unittest.mock import patch
import control as m

grid=N(header=N(frame_id='odom'),info=N(resolution=.01))
c=m.Control.__new__(m.Control);c.manual_supervision=False;c.cancelled=threading.Event()
c.odom=N(pose=N(pose=N(position=N(x=0.,y=0.),orientation=N(x=0.,y=0.,z=0.,w=1.))),
         twist=N(twist=N(linear=N(x=0.,y=0.),angular=N(z=0.))))
c.received=time.monotonic();c.costmap=grid;c.fresh_costmap=lambda _:grid
c.budget_time=lambda:1.;c.client=N(cancel_all_goals=lambda:None)
c.velocity=N(publish=lambda msg:None);c.report=lambda *args:None
stops=[];updates=[];c.stopped=lambda:stops.append(True) or True
tuning=dict(max_speed=1.,acceleration=1.4,max_yaw_rate=2.4,output_speed=1.05,output_acceleration=1.6,output_yaw_rate=2.8)
params={'~bend_settings':tuning,'/move_base/local_costmap/footprint':[(.2,.1),(-.2,.1),(-.2,-.1),(.2,-.1)],
        '/move_base/local_costmap/footprint_padding':.005,'~flow_settings/profiled_turns':True}
with patch.object(m,'Client',return_value=N(get_configuration=lambda:{'resolution':.01},update_configuration=lambda value:updates.append(value))), \
     patch.object(m.rospy,'get_param',side_effect=lambda key,default=None:params.get(key,default)), \
     patch.object(m.rospy,'get_time',return_value=1.),patch.object(m.rospy,'is_shutdown',return_value=False):
    c.follow_bends([['cone_entry',0.,0.,0.]],100.,path_override=[[0.,0.,0.]])
assert len(stops)==1,stops
assert not updates,updates
print('PASS: aligned entry stops once, retains its map, and performs no turn')
