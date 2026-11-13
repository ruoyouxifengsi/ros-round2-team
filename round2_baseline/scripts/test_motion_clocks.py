"""Replay slow simulation clocks through the real bend control loop, without Gazebo."""
import threading
import math
import unittest
from contextlib import ExitStack
from types import SimpleNamespace as NS
from unittest.mock import patch
import bend_path
import control as m


class MotionClockTest(unittest.TestCase):
    def run_bends(self, *, rate=.12, run_limit=540., moving=True, stale=False, target=2., map_hold=False, manual=False, pause_for=0.):
        wall=[100.];sim=[1000.];commands=[];events=[];config=[];tracking_positions=[];held=[False]
        c=m.Control.__new__(m.Control)
        c.manual_supervision=manual
        c.sim_timeouts=False;c.cancelled=threading.Event();c.received=wall[0]
        c.client=NS(cancel_all_goals=lambda:None)
        pose=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))
        c.odom=NS(pose=NS(pose=pose),twist=NS(twist=NS(linear=NS(x=.14,y=0.),angular=NS(z=0.))))
        c.costmap=NS(info=NS(resolution=.01))
        def fresh_costmap(deadline):
            if map_hold and not held[0]:
                # A stopped-command wait can still allow some measured braking motion.
                pose.position=NS(x=.3,y=0.);held[0]=True
            return c.costmap
        c.fresh_costmap=fresh_costmap
        c.stopped=lambda:True;c.rotate_short=lambda *args,**kwargs:False
        c.report=lambda state,detail:events.append((state,detail))
        c.velocity=NS(publish=lambda msg:commands.append((msg.linear.x,msg.angular.z)))
        tuning=dict(max_speed=1.,acceleration=1.4,max_yaw_rate=2.4,
                    output_speed=1.05,output_acceleration=1.6,output_yaw_rate=2.8)
        params={'~bend_settings':tuning,'/move_base/local_costmap/footprint':[],
                '/move_base/local_costmap/footprint_padding':.005}
        client=NS(get_configuration=lambda:{'resolution':.025},update_configuration=config.append)
        def path_command(*args):
            tracking_positions.append(args[2]);return .14,0.,0
        def sleep(dt):
            wall[0]+=1.;sim[0]+=rate
            if not stale:c.received=wall[0]
            if moving and wall[0]>100.+pause_for:pose.position=NS(x=pose.position.x+.14*rate,y=0.)
            if wall[0]>10000:raise AssertionError('Watchdog did not terminate')
        error=None
        with ExitStack() as stack:
            for owner,name,kw in [
                (m.time,'monotonic',dict(side_effect=lambda:wall[0])),
                (m.time,'sleep',dict(side_effect=sleep)),
                (m.rospy,'get_time',dict(side_effect=lambda:sim[0])),
                (m.rospy,'is_shutdown',dict(return_value=False)),
                (m.rospy,'get_param',dict(side_effect=lambda key:params[key])),
                (m,'Client',dict(return_value=client)),
                (bend_path,'path_command',dict(side_effect=path_command)),
                (bend_path,'braking_clear',dict(return_value=True)),
            ]:stack.enter_context(patch.object(owner,name,**kw))
            try:c.follow_bends([('bend8',target,0.,0.)],wall[0]+run_limit,
                               path_override=[(0.,0.,0.),(target,0.,0.)])
            except RuntimeError as exc:error=str(exc)
        self.assertEqual(commands[-1],(0.,0.))
        self.assertEqual(config[-1],{'resolution':.025})
        if map_hold:self.assertAlmostEqual(tracking_positions[0],.3)
        return error,wall[0]-100.,sim[0]-1000.,events

    def test_bends_refresh_pose_after_map_hold(self):
        error,_,_,_=self.run_bends(map_hold=True)
        self.assertIsNone(error)

    def test_manual_wait_survives_run_stage_and_progress_limits(self):
        error,wall,sim,events=self.run_bends(rate=1.,run_limit=40.,manual=True,pause_for=70.,target=1.96)
        self.assertIsNone(error)
        self.assertGreater(wall,70.)
        self.assertIn(('ARRIVED','bend8'),events)

    def test_slow_simulation_completes_after_sixty_wall_seconds(self):
        error,wall,sim,events=self.run_bends()
        self.assertIsNone(error)
        self.assertGreater(wall,60.)
        self.assertLess(sim,15.)
        self.assertIn(('ARRIVED','bend8'),events)

    def test_global_wall_deadline_is_still_enforced(self):
        error,wall,sim,_=self.run_bends(run_limit=40.)
        self.assertIn('Whole-run',error);self.assertLessEqual(wall,41.)

    def test_stale_odometry_still_stops_in_wall_time(self):
        error,wall,sim,_=self.run_bends(stale=True)
        self.assertIsNotNone(error);self.assertLessEqual(wall,3.)

    def test_no_progress_uses_simulation_time(self):
        error,wall,sim,_=self.run_bends(moving=False)
        self.assertIn('progress',error)
        self.assertGreaterEqual(sim,5.);self.assertLess(sim,5.2)

    def test_local_deadline_is_still_enforced(self):
        error,wall,sim,_=self.run_bends(rate=1.,target=100.)
        self.assertIn('60 simulation seconds',error);self.assertGreaterEqual(sim,60.);self.assertLessEqual(sim,61.)

    def check_short_motion(self,turn):
        wall=[100.];sim=[1000.];heading=[0.];commands=[]
        c=m.Control.__new__(m.Control);c.sim_timeouts=False;c.cancelled=threading.Event()
        c.fast=True;c.efficient=True;c.received=wall[0]
        c.client=NS(cancel_all_goals=lambda:None);c.stopped=lambda:True
        c.fresh_costmap=lambda deadline:None
        pose=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))
        c.odom=NS(pose=NS(pose=pose))
        c.velocity=NS(publish=lambda msg:commands.append((msg.linear.x,msg.angular.z)))
        def sleep(dt):
            wall[0]+=1.;sim[0]+=.02;c.received=wall[0]
            vx,omega=commands[-1];heading[0]+=omega*.02
            pose.orientation=NS(x=0.,y=0.,z=math.sin(heading[0]/2),w=math.cos(heading[0]/2))
            pose.position=NS(x=pose.position.x+vx*.02,y=0.)
        params={'/move_base/local_costmap/footprint':[],
                '/move_base/local_costmap/footprint_padding':.005,'~route_flow':True}
        with patch.object(m.time,'monotonic',side_effect=lambda:wall[0]), \
             patch.object(m.time,'sleep',side_effect=sleep), \
             patch.object(m.rospy,'get_time',side_effect=lambda:sim[0]), \
             patch.object(m.rospy,'is_shutdown',return_value=False), \
             patch.object(m.rospy,'get_param',side_effect=lambda key,default=None:params.get(key,default)), \
             patch.object(m,'footprint_clear',return_value=True):
            if turn:c.rotate_short(1.5,640.)
            else:c.straight_move(1.,0.,0.,640.,1,.025)
        self.assertEqual(commands[-1],(0.,0.))
        self.assertGreater(wall[0]-100.,15. if turn else 90.)
        self.assertLess(sim[0]-1000.,15.)
        if turn:self.assertLess(abs(heading[0]-1.5),.12)
        else:self.assertLess(abs(pose.position.x-1.),.025)

    def test_slow_rotation_completes(self):self.check_short_motion(True)

    def test_slow_docking_completes(self):self.check_short_motion(False)


if __name__=='__main__':unittest.main()
