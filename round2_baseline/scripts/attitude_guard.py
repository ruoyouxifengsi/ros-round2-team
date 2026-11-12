#!/usr/bin/env python3
"""Gate navigation velocities and scans using measured odometry attitude."""
import bisect
from collections import deque
import json
import math
import sys
import threading
import time


def tilt_degrees(q):
    n = sum(v*v for v in q)
    if not all(math.isfinite(v) for v in q) or n < 1e-12:
        raise ValueError('invalid quaternion')
    return math.degrees(math.acos(max(-1., min(1., 1.-2.*(q[0]**2+q[1]**2)/n))))


class YawFeedback:
    """Bounded PI trim for measured low-speed yaw tracking error."""
    def __init__(self):
        self.integral = self.target = 0.
        self.maximum = .65

    def reset(self):
        self.integral = self.target = 0.

    def step(self, target, measured, dt):
        if abs(target) < .01:
            self.reset()
            return 0.
        if target*self.target < 0:
            self.reset()
        self.target = target
        error = target-measured
        self.integral = max(-.25,min(.25,self.integral+.5*error*dt))
        value = target+.5*error+self.integral
        return math.copysign(max(0.,min(self.maximum,value*math.copysign(1.,target))),target)


class Guard:
    def __init__(self, max_speed=.12, linear_accel=.2, hold_tilt_deg=1.0, angular_feedback=False, curvature_accel=0., attitude_protection=True):
        if not all(math.isfinite(v) and v > 0 for v in (max_speed,linear_accel)):
            raise ValueError('velocity and acceleration limits must be positive and finite')
        if (isinstance(hold_tilt_deg,bool) or not isinstance(hold_tilt_deg,(int,float)) or
                not math.isfinite(hold_tilt_deg) or not .3 < hold_tilt_deg < 8.):
            raise ValueError('hold_tilt_deg must be finite and strictly between 0.3 and 8 degrees')
        self.max_speed, self.linear_accel = max_speed, linear_accel
        self.max_yaw_rate = .8
        if not math.isfinite(curvature_accel) or curvature_accel<0:raise ValueError('invalid curvature acceleration')
        self.curvature_accel = curvature_accel
        self.attitude_protection = attitude_protection
        self.hold_tilt_deg = float(hold_tilt_deg)
        self.mode, self.reason = 'BLOCKED', 'waiting for odometry'
        self.odom = self.cmd = self.level_since = self.last_clock = None
        self.serial = self.release_serial = self.trips = 0
        self.velocity = [0., 0., 0.]
        self.hold_seconds = 0.
        self.clock_progress_wall = None
        self.yaw_feedback = YawFeedback() if angular_feedback else None
        self.yaw_rate = 0.
        self.normal_limits=(self.max_speed,self.linear_accel,self.max_yaw_rate,
                            self.yaw_feedback.maximum if self.yaw_feedback else None)

    def route_limits(self,state,bend_settings,whole_route=False):
        active=state.get('state') in ('MOVING','PASSED') and (whole_route or state.get('detail','').startswith('bend'))
        if bend_settings and active:
            values=[bend_settings[k] for k in ('output_speed','output_acceleration','output_yaw_rate')]
            if not all(math.isfinite(v) and v>0 for v in values):raise ValueError('Invalid bend output limits')
            self.max_speed,self.linear_accel,self.max_yaw_rate=values
            if self.yaw_feedback:self.yaw_feedback.maximum=self.max_yaw_rate
        else:
            self.max_speed,self.linear_accel,self.max_yaw_rate,feedback_limit=self.normal_limits
            if self.yaw_feedback:self.yaw_feedback.maximum=feedback_limit

    def input(self, velocity, sim, wall):
        if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
            self.cmd = None
            return
        self.serial += 1
        self.cmd = (velocity, sim, wall, self.serial)

    def observe(self, stamp, wall, q, yaw_rate=0.):
        try:
            tilt = tilt_degrees(q)
            if not math.isfinite(stamp) or stamp <= 0 or not math.isfinite(yaw_rate):
                raise ValueError('invalid odometry time')
        except ValueError:
            self.odom = None
            self.zero('invalid odometry', True)
            return False
        if self.odom is not None:
            if stamp == self.odom[0]:
                return False
            if stamp < self.odom[0]:
                self.zero('reset odometry clock', True)
        self.odom = (stamp, wall, tilt)
        self.yaw_rate = yaw_rate
        if self.attitude_protection and tilt >= 8.:
            self.mode = 'FAULT'
            self.zero('excessive tilt')
        elif self.attitude_protection and tilt >= self.hold_tilt_deg and self.mode != 'FAULT':
            if self.mode != 'HOLD':
                self.trips += 1
            self.mode = 'HOLD'
            self.level_since = None
            self.release_serial = self.serial
            self.zero('waiting for level attitude')
        return True

    def zero(self, reason, block=False):
        if self.yaw_feedback:
            self.yaw_feedback.reset()
        self.velocity = [0., 0., 0.]
        self.reason = reason
        if block and self.mode != 'FAULT':
            self.mode = 'BLOCKED'
            self.level_since = None
            self.release_serial = self.serial
        return self.velocity[:]

    def tick(self, sim, wall):
        previous = self.last_clock
        self.last_clock = sim
        dt = max(0., min(.1, sim-previous)) if previous is not None else 0.
        if self.mode == 'FAULT':
            return self.zero('excessive tilt: restart after inspection')
        if not math.isfinite(sim) or sim <= 0 or (previous is not None and sim < previous):
            self.clock_progress_wall = None
            self.odom = self.cmd = None
            return self.zero('invalid or reset clock', True)
        if self.clock_progress_wall is None or previous is None or sim > previous:
            self.clock_progress_wall = wall
        if wall-self.clock_progress_wall > 3.:
            return self.zero('simulation clock is not advancing', True)
        if self.odom is None or not (-.1 <= sim-self.odom[0] <= .5) or not (0. <= wall-self.odom[1] <= 3.):
            return self.zero('stale or invalid odometry', True)
        stamp, _, tilt = self.odom
        if self.attitude_protection and tilt >= 8.:
            self.mode = 'FAULT'
            return self.zero('excessive tilt')
        if self.attitude_protection and tilt >= self.hold_tilt_deg:
            if self.mode != 'HOLD':
                self.trips += 1
            self.mode = 'HOLD'
            self.level_since = None
            self.release_serial = self.serial
        if self.mode != 'RUN':
            self.hold_seconds += dt
            if not self.attitude_protection:
                self.mode = 'RUN'
                self.release_serial = self.serial
            elif tilt <= .3:
                if self.level_since is None:
                    self.level_since = stamp
                if stamp-self.level_since >= .3:
                    self.mode = 'RUN'
                    self.release_serial = self.serial
            else:
                self.level_since = None
            return self.zero('waiting for level attitude and a new command' if self.attitude_protection else 'waiting for a new command')
        if (self.cmd is None or self.cmd[3] <= self.release_serial or
                not (0. <= sim-self.cmd[1] <= .5) or wall-self.cmd[2] > 3.):
            return self.zero('waiting for fresh command')
        target = list(self.cmd[0])
        magnitude = math.hypot(*target[:2])
        if magnitude > self.max_speed:
            target[0] *= self.max_speed/magnitude
            target[1] *= self.max_speed/magnitude
        normal_accel=math.hypot(*target[:2])*abs(target[2])
        if self.attitude_protection and self.curvature_accel and normal_accel>self.curvature_accel:
            factor=math.sqrt(self.curvature_accel/normal_accel)
            target=[value*factor for value in target]
        delta = [target[i]-self.velocity[i] for i in range(2)]
        norm = math.hypot(*delta)
        scale = min(1., self.linear_accel*dt/norm) if norm else 1.
        for i in range(2):
            self.velocity[i] += delta[i]*scale
        self.velocity[2] = max(-self.max_yaw_rate, min(self.max_yaw_rate, target[2]))
        if self.yaw_feedback:
            self.velocity[2] = self.yaw_feedback.step(self.velocity[2],self.yaw_rate,dt)
        self.reason = 'passing bounded navigation command'
        return self.velocity[:]

    def scan_is_level(self, pair, stamp, sim, wall):
        return bool(self.mode != 'FAULT' and self.odom is not None and
                    self.clock_progress_wall is not None and
                    0. <= wall-self.clock_progress_wall <= 3. and
                    0. <= wall-self.odom[1] <= 3. and
                    -.1 <= sim-self.odom[0] <= .5 and
                    math.isfinite(stamp) and stamp > 0. and 0. <= sim-stamp <= .5 and
                    len(pair) == 2 and pair[0][0] <= stamp <= pair[1][0] and
                    0. <= pair[1][0]-pair[0][0] <= .15 and max(p[1] for p in pair) <= .7)


def self_test():
    g = Guard()
    assert g.hold_tilt_deg == 1.0
    assert g.tick(0, 0) == [0, 0, 0]
    for t in [1., 1.1, 1.2, 1.31]:
        g.observe(t,t,[0,0,0,1]);g.input([.12,0,.5],t,t);g.tick(t,t)
    assert g.mode == 'RUN' and g.velocity == [0,0,0]
    g.input([.12,0,.5],1.36,1.36)
    assert 0 < g.tick(1.36,1.36)[0] <= .010001
    g.observe(1.4,1.4,[0,math.sin(math.radians(1.1)/2),0,math.cos(math.radians(1.1)/2)])
    assert g.tick(1.4,1.4) == [0,0,0] and g.trips == 1
    for t in [1.5,1.7,1.81]:
        g.observe(t,t,[0,0,0,1]);g.tick(t,t)
    assert g.mode == 'RUN' and g.tick(1.82,1.82) == [0,0,0]
    g.input([float('nan'),0,0],1.82,1.82)
    assert g.tick(1.83,1.83) == [0,0,0]
    assert g.tick(5,5) == [0,0,0] and g.mode == 'BLOCKED'
    g.observe(5,5,[0,math.sin(.1),0,math.cos(.1)])
    assert g.tick(5,5) == [0,0,0] and g.mode == 'FAULT'
    g.observe(6,6,[0,0,0,1]);g.input([.1,0,0],6,6)
    assert g.tick(6,6) == [0,0,0]
    h = Guard()
    for t in [1., 1.1, 1.2, 1.31]:
        h.observe(t,t,[0,0,0,1]);h.tick(t,t)
    h.input([.1,0,.5],1.32,1.32)
    assert h.tick(1.32,1.32)[2] == .5
    assert not h.observe(1.31,4.4,[0,0,0,1])
    assert h.odom[1] == 1.31
    h.input([.1,0,.5],1.32,4.4)
    assert h.tick(1.32,4.4) == [0,0,0] and h.mode == 'BLOCKED'
    assert not h.scan_is_level([(1.2,0.),(1.31,0.)],1.25,1.32,4.4)
    h.observe(1.4,4.5,[0,0,0,1]);h.tick(1.4,4.5)
    assert h.scan_is_level([(1.31,.2),(1.4,.3)],1.35,1.4,4.5)
    assert not h.scan_is_level([(1.31,.2),(1.4,.71)],1.35,1.4,4.5)
    assert not h.scan_is_level([(1.1,.2),(1.4,.3)],1.35,1.4,4.5)
    assert not h.scan_is_level([(1.31,.2),(1.4,.3)],1.35,2.,4.5)
    h.observe(1.5,4.6,[0,math.sin(.1),0,math.cos(.1)])
    h.observe(1.6,4.7,[0,0,0,1])
    assert h.mode == 'FAULT' and h.tick(1.6,4.7) == [0,0,0]
    fast = Guard(max_speed=.22,linear_accel=2.2)
    for t in [1.,1.1,1.2,1.31]:
        fast.observe(t,t,[0,0,0,1]);fast.tick(t,t)
    fast.input([.22,0,1.],1.36,1.36)
    first = fast.tick(1.36,1.36)
    assert abs(first[0]-.11) < 1e-9 and first[2] == .8
    fast.observe(1.41,1.41,[0,0,0,1])
    assert abs(fast.tick(1.41,1.41)[0]-.22) < 1e-9
    previous = fast.velocity[:]
    fast.input([.22,.22,-1.],1.46,1.46)
    diagonal = fast.tick(1.46,1.46)
    assert math.hypot(*diagonal[:2]) <= .22+1e-9 and diagonal[2] == -.8
    assert math.hypot(diagonal[0]-previous[0],diagonal[1]-previous[1]) <= .11+1e-9
    fast.observe(1.51,1.51,[0,0,0,1])
    diagonal = fast.tick(1.51,1.51)
    assert abs(math.hypot(*diagonal[:2])-.22) < 1e-9
    assert abs(diagonal[0]-diagonal[1]) < 1e-9
    fast.observe(1.52,1.52,[0,math.sin(math.radians(1.1)/2),0,math.cos(math.radians(1.1)/2)])
    assert fast.tick(1.52,1.52) == [0,0,0] and fast.mode == 'HOLD'
    for speed,accel in [(0.,2.2),(.22,0.),(-.22,2.2),(.22,float('nan'))]:
        try:
            Guard(speed,accel)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid guard limits accepted')
    diagnostic = Guard(hold_tilt_deg=2.)
    for t in [1.,1.1,1.2,1.31]:
        diagnostic.observe(t,t,[0,0,0,1]);diagnostic.tick(t,t)
    diagnostic.input([.12,0,.5],1.36,1.36)
    diagnostic.observe(1.36,1.36,[0,math.sin(math.radians(1.5)/2),0,math.cos(math.radians(1.5)/2)])
    assert diagnostic.tick(1.36,1.36)[2] == .5 and diagnostic.mode == 'RUN'
    diagnostic.observe(1.4,1.4,[0,math.sin(math.radians(2.1)/2),0,math.cos(math.radians(2.1)/2)])
    assert diagnostic.tick(1.4,1.4) == [0,0,0] and diagnostic.mode == 'HOLD'
    for t in [1.5,1.9]:
        diagnostic.observe(t,t,[0,math.sin(math.radians(.4)/2),0,math.cos(math.radians(.4)/2)])
        assert diagnostic.tick(t,t) == [0,0,0] and diagnostic.mode == 'HOLD'
    for t in [2.,2.1,2.31]:
        diagnostic.observe(t,t,[0,0,0,1]);diagnostic.tick(t,t)
    assert diagnostic.mode == 'RUN'
    diagnostic.observe(2.4,2.4,[0,math.sin(math.radians(8.1)/2),0,math.cos(math.radians(8.1)/2)])
    assert diagnostic.tick(2.4,2.4) == [0,0,0] and diagnostic.mode == 'FAULT'
    for threshold in [.3,8.,0.,-1.,float('nan'),float('inf'),True,None,'2']:
        try:
            Guard(hold_tilt_deg=threshold)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid hold threshold accepted')
    for target in [.25,-.25]:
        feedback=YawFeedback();actual=0.
        for _ in range(200):
            command=feedback.step(target,actual,.05)
            assert abs(command)<=.65
            plant=math.copysign(max(0.,abs(command)-.23)*1.2,command)
            actual+=(plant-actual)*.2
        assert abs(actual-target)<.03
        assert feedback.step(0.,actual,.05)==0. and feedback.integral==0.
    closed=Guard(angular_feedback=True)
    for t in [1.,1.1,1.2,1.31]:
        closed.observe(t,t,[0,0,0,1]);closed.tick(t,t)
    closed.input([0,0,.25],1.36,1.36)
    assert closed.tick(1.36,1.36)[2]>.25
    assert closed.tick(5.,5.)==[0,0,0] and closed.yaw_feedback.integral==0.
    disabled=Guard(max_speed=.55,linear_accel=10.,curvature_accel=.18,attitude_protection=False)
    tilted=[0,math.sin(math.radians(11.)/2),0,math.cos(math.radians(11.)/2)]
    disabled.observe(1.,1.,tilted);disabled.tick(1.,1.)
    disabled.observe(1.1,1.1,tilted);disabled.input([.5,0,.6],1.1,1.1)
    assert disabled.tick(1.1,1.1)==[.5,0.,.6] and disabled.mode=='RUN' and disabled.trips==0
    assert not disabled.scan_is_level([(1.,11.),(1.1,11.)],1.05,1.1,1.1)
    assert disabled.tick(5.,5.)==[0,0,0] and disabled.mode=='BLOCKED'
    assert not disabled.observe(5.1,5.1,[float('nan'),0,0,1])
    assert disabled.tick(5.1,5.1)==[0,0,0]
    print('PASS: protection off bypasses tilt stops and curvature scaling; invalid/stale-data stops and laser filtering remain')
    scoped=Guard(max_speed=.55,linear_accel=.65,angular_feedback=True)
    limits=dict(output_speed=.9,output_acceleration=1.3,output_yaw_rate=2.1)
    for state in ('MOVING','PASSED'):
        scoped.route_limits(dict(state=state,detail='bend3'),limits)
        assert (scoped.max_speed,scoped.linear_accel,scoped.yaw_feedback.maximum)==(.9,1.3,2.1)
    for state,stage in (('ARRIVED','bend8'),('MOVING','cone_exit'),('FAILED','bend3'),('CONTROL_ROUTE_DONE','')):
        scoped.route_limits(dict(state=state,detail=stage),limits)
        assert (scoped.max_speed,scoped.linear_accel,scoped.yaw_feedback.maximum)==(.55,.65,.65)
    scoped.route_limits(dict(state='MOVING',detail='bend1'),{})
    assert scoped.max_speed==.55
    print('PASS: bend bounds only activate with opt-in settings and restore on exit/failure')
    print('PASS: startup, new-command release, slew and clamp, stale data, frozen clock, duplicate odometry, scan gate, latched fault, bounded yaw feedback and stop reset')


def main():
    import rospy
    from actionlib_msgs.msg import GoalID
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    rospy.init_node('attitude_guard')
    guard = Guard(rospy.get_param('~max_linear_speed',.12),rospy.get_param('~linear_acceleration',.2),
                  rospy.get_param('~hold_tilt_deg',1.0),rospy.get_param('~angular_feedback',False),rospy.get_param('~curvature_accel',0.),
                  rospy.get_param('~attitude_protection',True))
    lock = threading.RLock()
    bend_settings=rospy.get_param('~bend_settings',{})
    whole_route=rospy.get_param('~route_flow',False)
    def route_state(message):
        state=json.loads(message.data)
        with lock:
            guard.route_limits(state,bend_settings,whole_route)
    rospy.Subscriber('/baseline/status',String,route_state,queue_size=10)
    history, scans = deque(maxlen=300), deque(maxlen=100)
    stats = {'scan_passed': 0, 'scan_dropped': 0}
    output = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    scan_output = rospy.Publisher('/nav_scan',LaserScan,queue_size=3)
    status = rospy.Publisher('/attitude_guard/status',String,queue_size=1,latch=True)
    cancel = rospy.Publisher('/move_base/cancel',GoalID,queue_size=1)
    def receive_odom(m):
        q=m.pose.pose.orientation
        with lock:
            stamp=m.header.stamp.to_sec()
            if history and stamp < history[-1][0]:
                history.clear();scans.clear()
            accepted=guard.observe(stamp,time.monotonic(),[q.x,q.y,q.z,q.w],m.twist.twist.angular.z)
            if accepted:
                history.append((stamp,guard.odom[2]))
            elif guard.odom is None:
                history.clear()
    def receive_cmd(m):
        with lock:
            guard.input([m.linear.x,m.linear.y,m.angular.z],rospy.get_time(),time.monotonic())
    def receive_scan(m):
        with lock:
            scans.append((m,time.monotonic()))
    rospy.Subscriber('/odom',Odometry,receive_odom,queue_size=30)
    rospy.Subscriber('/nav_cmd_vel',Twist,receive_cmd,queue_size=1)
    rospy.Subscriber('/scan',LaserScan,receive_scan,queue_size=10)
    rospy.on_shutdown(lambda: output.publish(Twist()))
    last_status=0.;cancelled=False
    try:
        while not rospy.is_shutdown():
            wall=time.monotonic();sim=rospy.get_time()
            with lock:
                velocity=guard.tick(sim,wall)
                msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=velocity
                output.publish(msg)
                if guard.mode == 'FAULT' and not cancelled and cancel.get_num_connections():
                    cancel.publish(GoalID());cancelled=True
                while scans:
                    scan,received=scans[0];stamp=scan.header.stamp.to_sec()
                    if not history or history[-1][0] < stamp:
                        if wall-received <= 1.:break
                        scans.popleft();stats['scan_dropped']+=1;continue
                    scans.popleft()
                    stamps=[entry[0] for entry in history]
                    idx=bisect.bisect_left(stamps,stamp)
                    pair=list(history)[max(0,idx-1):idx+1]
                    if guard.scan_is_level(pair,stamp,sim,wall):
                        scan_output.publish(scan);stats['scan_passed']+=1
                    else:stats['scan_dropped']+=1
                if wall-last_status>=.5:
                    data=dict(stats,mode=guard.mode,reason=guard.reason,trips=guard.trips,attitude_protection=guard.attitude_protection,
                              tilt_deg=guard.odom[2] if guard.odom else None,
                              hold_tilt_deg=guard.hold_tilt_deg,
                              hold_seconds=guard.hold_seconds,output=velocity)
                    status.publish(String(data=json.dumps(data)));last_status=wall
            time.sleep(.05)
    finally:
        output.publish(Twist())


if __name__ == '__main__':
    if '--self-test' in sys.argv:self_test()
    else:main()
