"""Regression for fresh simulation data arriving more than two wall seconds apart."""
import threading,unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import control as m
from attitude_guard import Guard

def running_guard(simulated=True):
    g=Guard(linear_accel=1.,attitude_protection=False,manual_supervision=True)
    g.simulated_time=simulated;g.mode='RUN';g.last_clock=10.;g.clock_progress_wall=100.
    for name in ('map','scan'):g.observe_navigation_data(name,10.,100.)
    return g

def output(g,sim,wall):
    g.observe(sim,wall,[0,0,0,1]);g.input([.1,0.,0.],sim,wall)
    return g.tick(sim,wall)

class SlowSimulationTest(unittest.TestCase):
    def test_final_output_keeps_fresh_simulation_data(self):
        self.assertGreater(output(running_guard(),10.2,102.72)[0],0.)

    def test_coordinator_accepts_same_observed_gap(self):
        c=m.Control.__new__(m.Control);c.manual_supervision=True;c.simulated_time=True
        c.cancelled=threading.Event();c.navigation_frame='odom'
        c.costmap=NS(header=NS(frame_id='odom',stamp=NS(to_sec=lambda:10.)))
        c.map_received=c.scan_received=100.;c.scan_stamp=10.;c.received=102.72
        c.budget_time=lambda:102.72;c.velocity=NS(publish=lambda msg:None)
        with patch.object(m.time,'monotonic',return_value=102.72),patch.object(m.rospy,'get_time',return_value=10.2), \
             patch.object(m.rospy,'is_shutdown',return_value=False),patch.object(m.rospy,'logwarn'), \
             patch.object(m.time,'sleep',side_effect=AssertionError('Fresh simulation data incorrectly held stopped')):
            self.assertIs(c.fresh_costmap(1000.),c.costmap)

    def test_real_age_limit_and_actual_outages_still_stop(self):
        self.assertEqual(output(running_guard(False),10.2,102.72),[0.,0.,0.])
        self.assertEqual(output(running_guard(),10.51,101.),[0.,0.,0.])
        self.assertEqual(output(running_guard(),10.2,105.01),[0.,0.,0.])
        g=running_guard()
        for name in ('map','scan'):g.observe_navigation_data(name,10.,104.)
        self.assertEqual(output(g,10.2,105.01),[0.,0.,0.])
        g=running_guard();g.last_clock=10.2
        self.assertEqual(output(g,10.2,103.01),[0.,0.,0.])

if __name__=='__main__':unittest.main()
