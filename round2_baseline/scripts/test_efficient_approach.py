"""Shortest-turn geometry and faster approach convergence without relaxed accuracy."""
import math
import json
from pathlib import Path
import sys
import threading
import time
import yaml
from types import SimpleNamespace as NS
from unittest.mock import patch
import control as m
from control import short_turn_command,straight_velocity,entry_velocity,footprint_clear

grid=NS(info=NS(resolution=.025,width=100,height=100,origin=NS(position=NS(x=-1.25,y=-1.25))),data=[0]*10000)
poly=[(.2,.1),(-.2,.1),(-.2,-.1),(.2,-.1)]
assert short_turn_command(grid,0,0,0,math.pi/2,poly,.005)>0
assert short_turn_command(grid,0,0,math.radians(179),math.radians(-179),poly,.005)==0
assert short_turn_command(grid,0,0,math.radians(170),math.radians(-170),poly,.005)>0
assert short_turn_command(grid,0,0,math.radians(-170),math.radians(170),poly,.005)<0
# This cell is clear at both endpoints but intersects the rotating corner.
ix=int((.15+1.25)/.025);iy=int((.15+1.25)/.025)
grid.data[iy*100+ix]=100
assert short_turn_command(grid,0,0,0,math.pi/2,poly,.005) is None
grid.data[iy*100+ix]=-1
assert short_turn_command(grid,0,0,0,math.pi/2,poly,.005) is None
for direction,distance in ((-1,.45),(1,.93)):
    for heading in (0.,math.pi/2):
        for offset in (-.02,.02):
            x,y=-offset*math.sin(heading),offset*math.cos(heading);a=heading+.22
            gx,gy=direction*distance*math.cos(heading),direction*distance*math.sin(heading)
            for _ in range(3000):
                if math.hypot(gx-x,gy-y)<.012:break
                v,w,remaining=straight_velocity(x,y,a,gx,gy,heading,direction,True,True)
                assert remaining>0 and abs(v)<=(.28 if direction>0 else .22)
                assert v*v<=.5*remaining+1e-9
                x+=v*math.cos(a)*.02;y+=v*math.sin(a)*.02;a+=w*.02
            assert math.hypot(gx-x,gy-y)<.012,(direction,heading,offset,x,y)
print('PASS: shortest sign, angle wrap, swept-footprint blocked/unknown rejection, precise forward/reverse convergence')

for heading in (0.,.1,math.pi/2):
    x,y,a=-.2225,.256,heading+.03
    for _ in range(500):
        vx,vy,w,distance=entry_velocity(x,y,a,-.25,.35,heading)
        if distance<.015:break
        assert math.hypot(vx,vy)<=.12+1e-9
        x+=(vx*math.cos(a)-vy*math.sin(a))*.02
        y+=(vx*math.sin(a)+vy*math.cos(a))*.02;a+=w*.02
    assert distance<.015 and abs(a-heading)<.04,(x,y,a)
print('PASS: short holonomic entry translation converges without a large turn')

sent=[];events=[];turns=[];moves=[]
class Action:
    def send_goal(self,goal):
        sent.append(goal)
        c.odom.pose.pose.position=NS(x=goal.target_pose.pose.position.x,y=goal.target_pose.pose.position.y)
    def wait_for_result(self,_):return True
    def get_state(self):return 3
    def cancel_all_goals(self):pass
c=m.Control.__new__(m.Control)
c.client=Action();c.continuous=True;c.fast=True;c.sim_timeouts=False;c.efficient=True
c.cancelled=threading.Event();c.lock=threading.Lock();c.running=True;c.received=time.monotonic()
c.odom=NS(pose=NS(pose=NS(position=NS(x=-.2225,y=.256),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
c.report=lambda state,detail:events.append((state,detail));c.stopped=lambda:True
def move(x,y,deadline):
    assert not turns and not sent
    moves.append((x,y));c.odom.pose.pose.position=NS(x=x,y=y)
c.move_entry=move
def rotate(target,deadline):
    assert moves==[(-.25,.35)] and not sent,'Use checked translation before rotation, not a TEB goal'
    assert ('ARRIVED','cone_entry') not in events
    turns.append(target)
c.rotate_short=rotate
params={'~route':[['cone_entry',-.25,.35,1.57]],'~docking_settings':{}}
with patch.object(m,'Client',return_value=NS(update_configuration=lambda v:None)), \
     patch.object(m.rospy,'get_param',side_effect=lambda key,default=None:params.get(key,default)), \
     patch.object(m.rospy.Time,'now',return_value=m.rospy.Time(1)):
    c.run()
assert turns==[1.57] and ('ARRIVED','cone_entry') in events and events[-1][0]=='CONTROL_ROUTE_DONE',events
print('PASS: reach entry position before shortest turn; report arrival only after turn and stop')

if len(sys.argv)>1:
    snapshot=json.loads(Path(sys.argv[1]).read_text());m=snapshot['local']
    grid=NS(info=NS(resolution=m['resolution'],width=m['width'],height=m['height'],origin=NS(position=NS(x=m['origin'][0],y=m['origin'][1]))),data=m['data'])
    cfg=yaml.safe_load((Path(__file__).parents[1]/'config/navigation.yaml').read_text())['local_costmap']
    poly,pad=cfg['footprint'],cfg['footprint_padding'];x,y=snapshot['position'];a=snapshot['rpy'][2]
    assert short_turn_command(grid,x,y,a,1.57,poly,pad) is None
    assert all(footprint_clear(grid,x+(-.25-x)*i/20,y+(.35-y)*i/20,a,poly,pad) for i in range(21))
    assert short_turn_command(grid,-.25,.35,a,1.57,poly,pad)>0
    print('PASS: recorded boundary blocks rotate-first; checked translation and destination turn are clear')
