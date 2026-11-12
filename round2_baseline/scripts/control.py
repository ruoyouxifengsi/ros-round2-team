#!/usr/bin/env python3
import json
import threading
import time
import math
import ast
import traceback
import rospy
import actionlib
from dynamic_reconfigure.client import Client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist


def footprint_clear(grid, x, y, yaw, polygon, padding):
    r=grid.info.resolution;ox=grid.info.origin.position.x;oy=grid.info.origin.position.y
    c,s=math.cos(yaw),math.sin(yaw)
    vertices=[]
    for px,py in polygon:
        px+=math.copysign(padding,px);py+=math.copysign(padding,py)
        vertices.append((x+c*px-s*py,y+s*px+c*py))
    margin=r/math.sqrt(2)
    ix0=math.floor((min(p[0] for p in vertices)-margin-ox)/r)
    ix1=math.floor((max(p[0] for p in vertices)+margin-ox)/r)
    iy0=math.floor((min(p[1] for p in vertices)-margin-oy)/r)
    iy1=math.floor((max(p[1] for p in vertices)+margin-oy)/r)
    if ix0<0 or iy0<0 or ix1>=grid.info.width or iy1>=grid.info.height:return False
    edges=list(zip(vertices,vertices[1:]+vertices[:1]))
    xmin,xmax=min(p[0] for p in vertices),max(p[0] for p in vertices)
    ymin,ymax=min(p[1] for p in vertices),max(p[1] for p in vertices)
    half=r/2
    for iy in range(iy0,iy1+1):
        for ix in range(ix0,ix1+1):
            value=grid.data[iy*grid.info.width+ix]
            if 0<=value<100:continue
            px,py=ox+(ix+.5)*r,oy+(iy+.5)*r
            if px+half<xmin or px-half>xmax or py+half<ymin or py-half>ymax:continue
            separated=False
            for (ax,ay),(bx,by) in edges:
                dx,dy=bx-ax,by-ay
                if dx*(py-ay)-dy*(px-ax)+half*(abs(dx)+abs(dy))<0:
                    separated=True;break
            if not separated:return False
    return True


def straight_velocity(px,py,angle,x,y,yaw,direction,fast=False):
    dx,dy=x-px,y-py
    cross=(py-y)*math.cos(yaw)-(px-x)*math.sin(yaw)
    target=yaw-direction*max(-.12,min(.12,2.*cross))
    error=math.atan2(math.sin(target-angle),math.cos(target-angle))
    remaining=direction*(dx*math.cos(yaw)+dy*math.sin(yaw))
    speed=min(.18,1.2*max(0.,remaining),math.sqrt(.5*max(0.,remaining))) if fast else min(.08,.8*max(0.,remaining))
    v=direction*speed if abs(error)<.25 else 0.
    return v,max(-.15,min(.15,1.5*error)),remaining


class Control:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.lock = threading.Lock()
        self.running = False
        self.fast = rospy.get_param('~speed_profile','stable') in ('fast','sprint')
        self.continuous = rospy.get_param('~continuous_bends',False)
        self.sim_timeouts = rospy.get_param('~timeout_clock','wall')=='sim'
        self.cancelled = threading.Event()
        self.odom = None
        self.received = 0
        self.guard = None
        self.guard_received = 0
        self.costmap = None
        self.map_received = 0
        self.velocity = rospy.Publisher('/nav_cmd_vel',Twist,queue_size=1)
        self.status = rospy.Publisher('/baseline/status', String, queue_size=10, latch=True)
        self.ready = rospy.Publisher('/baseline/ready', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/odom', Odometry, self.observe)
        rospy.Subscriber('/attitude_guard/status', String, self.observe_guard)
        rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,self.observe_map,queue_size=1)
        rospy.Service('/nav_start', SetBool, self.start)
        rospy.Timer(rospy.Duration(.5),lambda _:self.ready.publish(Bool(data=self.preflight() is None)))
        self.report('IDLE', 'CONTROL_ONLY: image tasks omitted')

    def observe_guard(self, msg):
        self.guard=json.loads(msg.data)
        self.guard_received=time.monotonic()

    def observe_map(self, msg):
        self.costmap=msg;self.map_received=time.monotonic()

    def preflight(self):
        if self.running or self.odom is None or time.monotonic()-self.received>2:
            return 'Busy or stale odometry'
        if not self.guard or time.monotonic()-self.guard_received>2 or self.guard.get('mode')=='FAULT':
            return 'Guard unavailable or faulted'
        p=self.odom.pose.pose.position
        sx,sy=rospy.get_param('~start_position',[-.75,1.75])
        if math.hypot(p.x-sx,p.y-sy)>.15:
            return 'Vehicle is not at the starting region; restart the scene'
        v=self.odom.twist.twist
        if math.hypot(v.linear.x,v.linear.y)>.02 or abs(v.angular.z)>.05:
            return 'Vehicle has not stopped'
        return None

    def observe(self, msg):
        self.odom = msg
        self.received = time.monotonic()

    def budget_time(self):
        return rospy.get_time() if self.sim_timeouts else time.monotonic()

    def report(self, state, detail):
        record = dict(state=state, detail=detail, stamp=rospy.Time.now().to_sec(), control_only=True)
        self.status.publish(json.dumps(record))
        rospy.loginfo(json.dumps(record))

    def start(self, req):
        with self.lock:
            if not req.data:
                self.cancelled.set()
                self.client.cancel_all_goals()
                return SetBoolResponse(True, 'Cancellation requested')
            reason=self.preflight()
            if reason:
                return SetBoolResponse(False, reason)
            if not self.client.wait_for_server(rospy.Duration(1)):
                return SetBoolResponse(False, 'Navigation unavailable')
            self.running = True
            self.cancelled.clear()
            threading.Thread(target=self.run, daemon=True).start()
            return SetBoolResponse(True, 'Control-only run started')

    def stopped(self):
        deadline = time.monotonic()+5
        stable = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            v = self.odom.twist.twist
            if time.monotonic()-self.received < 1 and math.hypot(v.linear.x,v.linear.y)<0.02 and abs(v.angular.z)<0.05:
                stable = stable or time.monotonic()
                if time.monotonic()-stable > 0.5:
                    return True
            else:
                stable = None
            time.sleep(0.05)
        return False

    def run(self):
        try:
            planner = Client('/move_base/TebLocalPlannerROS', timeout=5)
            docking = rospy.get_param('~docking_settings', {})
            travel = {k:rospy.get_param('/move_base/TebLocalPlannerROS/'+k) for k in docking}
            run_deadline = self.budget_time()+rospy.get_param('~run_timeout', 540.0)
            route=rospy.get_param('~route')
            for index,(name,x,y,yaw) in enumerate(route):
                if self.cancelled.is_set():
                    raise RuntimeError('Cancelled')
                distance = 0.08 if name in ('cone_exit','blocks','return') else rospy.get_param('~travel_clearance',0.06)
                if name in ('qr','qr_exit','finish'):distance=0.02
                # The 2cm objective repeatedly proposes colliding corner trajectories.
                if name=='bend1':distance=0.06
                settings = dict(docking if name in ('qr','qr_exit','finish') else travel)
                passing = self.continuous and name.startswith('bend') and index<len(route)-1
                settings['free_goal_vel'] = passing
                if passing:settings['yaw_goal_tolerance']=math.pi
                settings['min_obstacle_dist'] = distance
                if name in ('cone_exit','blocks'):settings['xy_goal_tolerance']=.05
                planner.update_configuration(settings)
                self.report('MOVING',name)
                if name in ('qr_exit','finish'):
                    self.straight_move(x,y,yaw,run_deadline,-1 if name=='qr_exit' else 1,.03 if name=='qr_exit' else .025)
                    if not self.stopped():raise RuntimeError('Straight docking did not stop')
                    self.report('ARRIVED',name)
                    continue
                g = MoveBaseGoal()
                g.target_pose.header.frame_id = 'odom'
                g.target_pose.header.stamp = rospy.Time.now()
                g.target_pose.pose.position.x = x
                g.target_pose.pose.position.y = y
                g.target_pose.pose.orientation.z = math.sin(yaw/2)
                g.target_pose.pose.orientation.w = math.cos(yaw/2)
                self.client.send_goal(g)
                deadline = self.budget_time()+(150 if name=='cone_exit' else 90)
                progress_at=self.budget_time()
                previous=self.odom.pose.pose
                approach_released=distance<=0.02 or name in ('cone_exit','blocks','return')
                passed=False
                while not self.client.wait_for_result(rospy.Duration(0.05 if passing else 0.2)):
                    now=self.budget_time();wall_now=time.monotonic()
                    current=self.odom.pose.pose
                    if passing and not self.cancelled.is_set() and wall_now-self.received<1 and math.hypot(current.position.x-x,current.position.y-y)<.10:
                        passed=True
                        break
                    # Keep the original final-approach objective so a larger travel
                    # clearance cannot prevent convergence to an existing waypoint.
                    if not approach_released and math.hypot(current.position.x-x,current.position.y-y)<.15:
                        planner.update_configuration({'min_obstacle_dist':0.02})
                        approach_released=True
                        rospy.loginfo('Final approach clearance restored for %s',name)
                    q0,q1=previous.orientation,current.orientation
                    moved=math.hypot(current.position.x-previous.position.x,current.position.y-previous.position.y)>.01
                    turned=abs(q0.x*q1.x+q0.y*q1.y+q0.z*q1.z+q0.w*q1.w)<math.cos(.02/2)
                    if moved or turned:
                        previous=current;progress_at=now
                    if now-progress_at>28:
                        raise RuntimeError('No measured movement for 28 budget seconds: '+name)
                    if self.cancelled.is_set() or now>deadline or now>run_deadline or wall_now-self.received>2:
                        raise RuntimeError('Cancelled, stale odometry or stage timeout: '+name)
                if passed:
                    self.report('PASSED',name)
                    continue
                if self.client.get_state()!=3 or not self.stopped():
                    raise RuntimeError('Navigation or stop failed: '+name)
                self.report('ARRIVED', name)
                if name in ('qr','blocks'):
                    self.report('IMAGE_SKIPPED', name)
            self.report('CONTROL_ROUTE_DONE','Not a complete competition finish; wheel placement requires verification')
        except Exception as exc:
            traceback.print_exc()
            self.client.cancel_all_goals()
            self.stopped()
            self.report('FAILED',str(exc))
        finally:
            with self.lock:
                self.running = False

    def straight_move(self, x, y, yaw, run_deadline, direction, tolerance):
        self.client.cancel_all_goals()
        polygon=rospy.get_param('/move_base/local_costmap/footprint')
        if isinstance(polygon,str):polygon=ast.literal_eval(polygon)
        padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
        deadline=min(run_deadline,self.budget_time()+90)
        progress_at=self.budget_time();previous=self.odom.pose.pose.position
        try:
            while not rospy.is_shutdown():
                now=self.budget_time();wall_now=time.monotonic()
                if self.cancelled.is_set() or now>deadline or wall_now-self.received>2:
                    raise RuntimeError('Straight docking cancelled, stale odometry or timeout')
                if self.costmap is None or wall_now-self.map_received>2 or self.costmap.header.frame_id.lstrip('/')!='odom':
                    raise RuntimeError('Straight docking requires a fresh odom-frame costmap')
                pose=self.odom.pose.pose;q=pose.orientation;p=pose.position
                angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                dx,dy=x-p.x,y-p.y;distance=math.hypot(dx,dy)
                if distance<tolerance:return
                if math.hypot(p.x-previous.x,p.y-previous.y)>.005:
                    progress_at=now;previous=p
                if now-progress_at>28:raise RuntimeError('Straight docking made no progress')
                heading_error=math.atan2(math.sin(angle-yaw),math.cos(angle-yaw))
                if abs(heading_error)>.3:raise RuntimeError('Straight docking heading left the corridor')
                v,omega,remaining=straight_velocity(p.x,p.y,angle,x,y,yaw,direction,self.fast)
                if remaining<=0:raise RuntimeError('Straight docking passed target without alignment')
                safe=True;px,py,pa=p.x,p.y,angle
                cross_squared=max(0.,distance*distance-remaining*remaining)
                stop_along=math.sqrt(max(0.,tolerance*tolerance-cross_squared))
                prediction_time=min(1.,max(0.,remaining-stop_along+.003)/max(abs(v),.001))
                for i in range(11):
                    safe &= footprint_clear(self.costmap,px,py,pa,polygon,padding)
                    dt=min(.1,max(0.,prediction_time-i*.1))
                    px+=v*math.cos(pa)*dt;py+=v*math.sin(pa)*dt;pa+=omega*dt
                msg=Twist()
                if safe:msg.linear.x=v;msg.angular.z=omega
                elif all(footprint_clear(self.costmap,p.x,p.y,angle+omega*.1*i,polygon,padding) for i in range(11)):
                    msg.angular.z=omega
                self.velocity.publish(msg)
                time.sleep(.05)
        finally:
            self.velocity.publish(Twist())


if __name__=='__main__':
    rospy.init_node('round2_control')
    control = Control()
    rospy.spin()
