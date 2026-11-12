"""Exercise the route coordinator with completed and in-flight action goals."""
import importlib.machinery
import importlib.util
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

path=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).with_name('control.py')
loader=importlib.machinery.SourceFileLoader('route_under_test',str(path))
spec=importlib.util.spec_from_loader(loader.name,loader)
m=importlib.util.module_from_spec(spec);loader.exec_module(m)
settings=[];sent=[];events=[];stops=[]
params={'~route':[['bend1',0.,0.,1.57],['qr',0.,0.,0.]],
        '~docking_settings':{'yaw_goal_tolerance':.08},
        '/move_base/TebLocalPlannerROS/yaw_goal_tolerance':.25}

class Action:
    def send_goal(self,goal):self.polls=0;sent.append(goal)
    def wait_for_result(self,_):self.polls+=1;return self.polls>1
    def get_state(self):return 3
    def cancel_all_goals(self):pass

c=m.Control.__new__(m.Control)
c.client=Action();c.continuous=True;c.fast=True;c.sim_timeouts=False;c.efficient=False
c.navigation_frame='odom'
c.cancelled=threading.Event();c.lock=threading.Lock();c.running=True
c.received=time.monotonic()
c.odom=NS(pose=NS(pose=NS(position=NS(x=.05,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
c.report=lambda state,detail:events.append((state,detail))
c.stopped=lambda:stops.append(True) or True
with patch.object(m,'Client',return_value=NS(update_configuration=lambda v:settings.append(dict(v)))), \
     patch.object(m.rospy,'get_param',side_effect=lambda key,default=None:params.get(key,default)), \
     patch.object(m.rospy.Time,'now',return_value=m.rospy.Time(1)), \
     patch.object(m.rospy,'loginfo'):
    c.run()
assert ('PASSED','bend1') in events,events
assert ('ARRIVED','qr') in events and ('IMAGE_SKIPPED','qr') in events,events
assert len(stops)==1,stops
assert settings[0]['free_goal_vel'] and not settings[-1]['free_goal_vel'],settings
assert settings[-1]['yaw_goal_tolerance']==.08,settings
assert len(sent)==2 and events[-1][0]=='CONTROL_ROUTE_DONE',events
print('PASS: switch bend while goal active; restore precise QR settings and stop once')
