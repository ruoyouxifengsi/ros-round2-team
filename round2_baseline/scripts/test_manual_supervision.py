"""Supervised waiting, final output gating, planner retry and user cancellation."""
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import control as m
import route_flow
from attitude_guard import Guard


class ManualSupervisionTest(unittest.TestCase):
    def test_output_stops_despite_planner_commands_and_recovers(self):
        for missing in ('map','scan','odom'):
            with self.subTest(missing=missing):
                guard=Guard(linear_accel=1.,attitude_protection=False,manual_supervision=True)
                outputs=[]
                for i in range(1,251):
                    t=10.+i*.05;outage=12.<t<20.
                    if not (outage and missing=='odom'):guard.observe(t,t,[0,0,0,1])
                    for name in ('map','scan'):
                        if not (outage and missing==name):guard.observe_navigation_data(name,t,t)
                    guard.input([.1,0.,0.],t,t)
                    value=guard.tick(t,t);outputs.append(value[0])
                    if 14.1<t<20.:self.assertEqual(value,[0.,0.,0.])
                self.assertGreater(max(outputs[:20]),0.)
                self.assertGreater(outputs[-1],0.)
                self.assertNotEqual(guard.mode,'FAULT')

    def test_duplicate_data_does_not_refresh_watchdog(self):
        guard=Guard(manual_supervision=True)
        guard.observe_navigation_data('map',10.,100.)
        guard.observe_navigation_data('map',10.,109.)
        self.assertEqual(guard.navigation_data['map'],(10.,100.))

    def test_user_cancel_interrupts_indefinite_data_wait(self):
        c=m.Control.__new__(m.Control);c.manual_supervision=True;c.cancelled=threading.Event()
        c.navigation_frame='odom';c.costmap=None;c.map_received=0.;c.received=0.
        c.scan_received=c.scan_stamp=0.;commands=[];clock=[10.]
        c.velocity=NS(publish=lambda msg:commands.append((msg.linear.x,msg.linear.y,msg.angular.z)))
        c.budget_time=lambda:clock[0]
        def sleep(dt):
            clock[0]+=dt
            if clock[0]>20.:c.cancelled.set()
        with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(m.time,'sleep',side_effect=sleep), \
             patch.object(m.rospy,'get_time',side_effect=lambda:clock[0]), \
             patch.object(m.rospy,'is_shutdown',return_value=False),patch.object(m.rospy,'logwarn'):
            with self.assertRaisesRegex(RuntimeError,'Cancelled'):c.fresh_costmap(11.)
        self.assertGreater(clock[0],20.)
        self.assertTrue(commands);self.assertFalse(any(any(v) for v in commands))

    def test_stop_confirmation_waits_for_data_and_can_be_cancelled(self):
        for cancel in (False,True):
            c=m.Control.__new__(m.Control);c.manual_supervision=True;c.cancelled=threading.Event()
            clock=[10.];c.received=0.
            c.odom=NS(twist=NS(twist=NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))))
            c.velocity=NS(publish=lambda msg:None)
            def sleep(dt):
                clock[0]+=dt
                if clock[0]>=20.:
                    if cancel:c.cancelled.set()
                    else:c.received=clock[0]
            with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]), \
                 patch.object(m.time,'sleep',side_effect=sleep),patch.object(m.rospy,'is_shutdown',return_value=False):
                self.assertEqual(c.stopped(),not cancel)
            self.assertGreaterEqual(clock[0],20.)

    def test_planner_abort_retries_without_losing_goal_or_progress_timeout(self):
        c=m.Control.__new__(m.Control);c.manual_supervision=True;c.cancelled=threading.Event()
        c.navigation_frame='odom';clock=[10.];goals=[];events=[]
        pose=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))
        c.odom=NS(pose=NS(pose=pose));c.received=10.;c.stopped=lambda:True
        c.fresh_costmap=lambda deadline:None;c.budget_time=lambda:clock[0]
        c.velocity=NS(publish=lambda msg:None);c.report=lambda *event:events.append(event)
        def state():
            if clock[0]>=60.:return 3
            if clock[0]>=20. and len(goals)==1:return 4
            return 1
        c.client=NS(send_goal=goals.append,get_state=state)
        def sleep(dt):
            clock[0]+=1.;c.received=clock[0]
            if clock[0]>=60.:pose.position=NS(x=1.,y=0.)
            if clock[0]>100:raise AssertionError('Navigation failed to complete')
        with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(m.time,'sleep',side_effect=sleep), \
             patch.object(m.rospy,'get_time',side_effect=lambda:clock[0]), \
             patch.object(m.rospy.Time,'now',return_value=m.rospy.Time(10)), \
             patch.object(m.rospy,'is_shutdown',return_value=False),patch.object(m.rospy,'logwarn_throttle'):
            route_flow.navigate(c,NS(update_configuration=lambda settings:None),[['blocks',1.,0.,0.]],15.,{})
        self.assertEqual(len(goals),2)
        self.assertTrue(all(g.target_pose.pose.position.x==1. for g in goals))
        self.assertIn(('ARRIVED','blocks'),events)


if __name__=='__main__':unittest.main()
