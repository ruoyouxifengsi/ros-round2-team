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


def short_turn_command(grid,x,y,angle,target,polygon,padding,angular_limit=.5,tolerance=.12):
    error=math.atan2(math.sin(target-angle),math.cos(target-angle))
    if abs(error)<tolerance:return 0.
    steps=max(1,math.ceil(abs(error)/.04))
    if not all(footprint_clear(grid,x,y,angle+error*i/steps,polygon,padding) for i in range(steps+1)):
        return None
    return max(-angular_limit,min(angular_limit,1.6*error))


def entry_velocity(px,py,angle,x,y,heading):
    dx,dy=x-px,y-py;distance=math.hypot(dx,dy)
    gain=min(1.6,.12/max(distance,1e-9))
    error=math.atan2(math.sin(heading-angle),math.cos(heading-angle))
    vx=gain*(dx*math.cos(angle)+dy*math.sin(angle))
    vy=gain*(-dx*math.sin(angle)+dy*math.cos(angle))
    return vx,vy,max(-.3,min(.3,1.6*error)),distance


def straight_velocity(px,py,angle,x,y,yaw,direction,fast=False,efficient=False,flow=False):
    dx,dy=x-px,y-py
    cross=(py-y)*math.cos(yaw)-(px-x)*math.sin(yaw)
    target=yaw-direction*max(-.12,min(.12,(4. if efficient and fast else 2.)*cross))
    error=math.atan2(math.sin(target-angle),math.cos(target-angle))
    remaining=direction*(dx*math.cos(yaw)+dy*math.sin(yaw))
    speed=min(.18,1.2*max(0.,remaining),math.sqrt(.5*max(0.,remaining))) if fast else min(.08,.8*max(0.,remaining))
    if efficient and fast:
        speed=min(.28 if direction>0 else .22,1.5*max(0.,remaining),math.sqrt(.5*max(0.,remaining)))
    if flow:speed=min(.75 if direction>0 else .45,2.5*max(0.,remaining),math.sqrt(1.6*max(0.,remaining)))
    v=direction*speed if abs(error)<(.15 if efficient and fast else .25) else 0.
    angular_limit=.8 if flow else (.4 if efficient and fast else .15)
    return v,max(-angular_limit,min(angular_limit,(3. if efficient and fast else 1.5)*error)),remaining


class Control:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.lock = threading.Lock()
        self.running = False
        self.fast = rospy.get_param('~speed_profile','stable') in ('fast','sprint')
        self.continuous = rospy.get_param('~continuous_bends',False)
        self.efficient = rospy.get_param('~efficient_approach',False)
        self.navigation_frame = rospy.get_param('~navigation_frame','odom')
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
        rospy.Subscriber(rospy.get_param('~pose_topic','/odom'), Odometry, self.observe)
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
        # Only the whole-run budget follows timeout_clock; motion watchdogs use ROS time.
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
            if rospy.get_param('~route_flow',False):
                import sys
                from pathlib import Path
                if str(Path(__file__).parent) not in sys.path:sys.path.insert(0,str(Path(__file__).parent))
                from route_flow import run_flow
                run_flow(self,planner,route,run_deadline)
                self.report('CONTROL_ROUTE_DONE','CONTROL ONLY: image recognition and its processing time omitted')
                return
            if rospy.get_param('~bend_path',False):
                bends=[]
                for point in route:
                    if not point[0].startswith('bend'):break
                    bends.append(point)
                if bends:
                    self.follow_bends(bends,run_deadline)
                    route=route[len(bends):]
            # Leave room for map resolution and localization error inside the same task boxes.
            inset=rospy.get_param('~docking_inset',0.)
            route=[[name,x+(inset if name=='qr' else 0.),y+(inset if name=='finish' else 0.),yaw] for name,x,y,yaw in route]
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
                docking_cruise=self.efficient and self.fast and name=='qr'
                if docking_cruise:settings['max_vel_x']=.20
                if name in ('cone_exit','blocks'):settings['xy_goal_tolerance']=.05
                planner.update_configuration(settings)
                self.report('MOVING',name)
                entry_turn=self.efficient and name=='cone_entry'
                if entry_turn:
                    self.move_entry(x,y,run_deadline)
                    if not self.stopped():raise RuntimeError('Entry translation did not stop')
                    self.rotate_short(yaw,run_deadline)
                    if not self.stopped():raise RuntimeError('Entry rotation did not stop')
                    p=self.odom.pose.pose.position
                    if math.hypot(p.x-x,p.y-y)>settings.get('xy_goal_tolerance',.03):
                        raise RuntimeError('Entry rotation left the position tolerance')
                    self.report('ARRIVED',name)
                    continue
                if name in ('qr_exit','finish'):
                    self.straight_move(x,y,yaw,run_deadline,-1 if name=='qr_exit' else 1,.03 if name=='qr_exit' else .025)
                    if not self.stopped():raise RuntimeError('Straight docking did not stop')
                    self.report('ARRIVED',name)
                    continue
                g = MoveBaseGoal()
                g.target_pose.header.frame_id = self.navigation_frame
                g.target_pose.header.stamp = rospy.Time.now()
                g.target_pose.pose.position.x = x
                g.target_pose.pose.position.y = y
                g.target_pose.pose.orientation.z = math.sin(yaw/2)
                g.target_pose.pose.orientation.w = math.cos(yaw/2)
                self.client.send_goal(g)
                deadline = rospy.get_time()+(150 if name=='cone_exit' else 90)
                progress_at=rospy.get_time()
                previous=self.odom.pose.pose
                approach_released=distance<=0.02 or name in ('cone_exit','blocks','return')
                passed=False
                while not self.client.wait_for_result(rospy.Duration(0.05 if passing else 0.2)):
                    now=rospy.get_time();wall_now=time.monotonic()
                    current=self.odom.pose.pose
                    if docking_cruise and math.hypot(current.position.x-x,current.position.y-y)<.12:
                        planner.update_configuration({'max_vel_x':docking['max_vel_x']})
                        docking_cruise=False
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
                        raise RuntimeError('No measured movement for 28 simulation seconds: '+name)
                    if self.cancelled.is_set() or now>deadline or self.budget_time()>run_deadline or wall_now-self.received>2:
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

    def follow_bends(self,route,run_deadline,path_override=None,position_tolerance=.023,yaw_tolerance=.12):
        import sys
        from pathlib import Path
        if str(Path(__file__).parent) not in sys.path:sys.path.insert(0,str(Path(__file__).parent))
        from bend_path import rounded_path
        p=self.odom.pose.pose.position
        vertices=[(p.x,p.y)]+[(x,2.72 if name in ('bend1','bend2') else y) for name,x,y,_ in route]
        path=path_override if path_override is not None else rounded_path(vertices)
        from bend_path import path_command, braking_clear, validate_settings
        tuning=rospy.get_param('~bend_settings')
        validate_settings(tuning)
        costmap_client=Client('/move_base/local_costmap',timeout=5)
        old_resolution=costmap_client.get_configuration()['resolution']
        polygon=rospy.get_param('/move_base/local_costmap/footprint')
        if isinstance(polygon,str):polygon=ast.literal_eval(polygon)
        padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
        self.client.cancel_all_goals()
        try:
            costmap_client.update_configuration({'resolution':.01})
            ready=time.monotonic()+5
            while self.costmap is None or abs(self.costmap.info.resolution-.01)>1e-6:
                if time.monotonic()>ready:raise RuntimeError('Bend costmap resolution update timed out')
                time.sleep(.02)
            name,x,y,yaw=route[-1]
            self.report('MOVING',route[0][0])
            index=0;progress=0;deadline=rospy.get_time()+60
            last_progress=rospy.get_time();previous=p
            while not rospy.is_shutdown():
                now=time.monotonic();pose=self.odom.pose.pose;position=pose.position;q=pose.orientation
                if self.cancelled.is_set():raise RuntimeError('Continuous bend path cancelled')
                if now-self.received>2:raise RuntimeError('Continuous bend path odometry stale for 2 wall seconds')
                if self.budget_time()>run_deadline:raise RuntimeError('Whole-run time budget exceeded during bends')
                if rospy.get_time()>deadline:raise RuntimeError('Continuous bend path exceeded 60 simulation seconds')
                if math.hypot(position.x-previous.x,position.y-previous.y)>.01:
                    last_progress=rospy.get_time();previous=position
                if rospy.get_time()-last_progress>5:raise RuntimeError('Continuous bend path made no measured progress for 5 simulation seconds')
                if index<len(route)-1 and math.hypot(position.x-route[index][1],position.y-route[index][2])<.10:
                    self.report('PASSED',route[index][0]);index+=1
                    self.report('MOVING',route[index][0])
                if math.hypot(position.x-x,position.y-y)<position_tolerance:break
                angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                vx,omega,progress=path_command(path,progress,position.x,position.y,angle,
                    tuning['max_speed'],tuning['acceleration'],tuning['max_yaw_rate'])
                grid=self.fresh_costmap(run_deadline)
                measured=self.odom.twist.twist
                safe=braking_clear(lambda px,py,pa:footprint_clear(grid,px,py,pa,polygon,padding),
                    position.x,position.y,angle,(measured.linear.x,measured.linear.y,measured.angular.z),
                    (vx,0.,omega),tuning['output_acceleration'])
                msg=Twist()
                if safe:msg.linear.x=vx;msg.angular.z=omega
                self.velocity.publish(msg);time.sleep(.02)
            self.velocity.publish(Twist())
            if index!=len(route)-1 or not self.stopped():
                raise RuntimeError('Continuous bend path failed or missed a waypoint: '+route[index][0])
            self.rotate_short(yaw,run_deadline,tuning['max_yaw_rate'],yaw_tolerance)
            if not self.stopped():raise RuntimeError('Bend exit rotation did not stop')
            self.report('ARRIVED',route[-1][0])
        finally:
            self.velocity.publish(Twist())
            costmap_client.update_configuration({'resolution':old_resolution})

    def fresh_costmap(self,deadline):
        started=time.monotonic();waiting=False
        while not rospy.is_shutdown():
            wall=time.monotonic();grid=self.costmap
            if grid is not None and grid.header.frame_id.lstrip('/')!=self.navigation_frame:
                raise RuntimeError('Costmap frame differs from the navigation frame')
            if grid is not None and wall-self.map_received<=2:
                if waiting:rospy.loginfo('Costmap recovered; resuming checked motion')
                return grid
            self.velocity.publish(Twist())
            if self.navigation_frame!='map' or self.cancelled.is_set() or wall-self.received>2 or self.budget_time()>deadline or wall-started>=5:
                raise RuntimeError('Fresh costmap unavailable; vehicle held stopped')
            if not waiting:
                rospy.logwarn('Costmap stale: holding zero velocity until fresh data')
                waiting=True
            time.sleep(.05)
        raise RuntimeError('Shutdown while waiting for a fresh costmap')

    def move_entry(self,x,y,run_deadline):
        self.client.cancel_all_goals()
        if not self.stopped():raise RuntimeError('Entry translation requires a stopped vehicle')
        polygon=rospy.get_param('/move_base/local_costmap/footprint')
        if isinstance(polygon,str):polygon=ast.literal_eval(polygon)
        padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
        p=self.odom.pose.pose
        if math.hypot(p.position.x-x,p.position.y-y)>.25:
            raise RuntimeError('Entry translation is only permitted near its waypoint')
        q=p.orientation;heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        deadline=rospy.get_time()+10
        try:
            while not rospy.is_shutdown():
                wall=time.monotonic()
                if self.cancelled.is_set() or rospy.get_time()>deadline or self.budget_time()>run_deadline or wall-self.received>2:
                    raise RuntimeError('Entry translation cancelled, stale odometry or timeout')
                grid=self.fresh_costmap(run_deadline)
                p=self.odom.pose.pose;q=p.orientation
                angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                vx,vy,omega,distance=entry_velocity(p.position.x,p.position.y,angle,x,y,heading)
                if distance<.015:return
                if abs(math.atan2(math.sin(angle-heading),math.cos(angle-heading)))>.2:
                    raise RuntimeError('Entry translation lost its heading')
                dt=min(1.,distance/max(math.hypot(vx,vy),.001))/20
                px,py,pa=p.position.x,p.position.y,angle
                for i in range(21):
                    if not footprint_clear(grid,px,py,pa,polygon,padding):
                        raise RuntimeError('Entry translation intersects occupied or unknown space')
                    if i<20:
                        px+=(vx*math.cos(pa)-vy*math.sin(pa))*dt
                        py+=(vx*math.sin(pa)+vy*math.cos(pa))*dt;pa+=omega*dt
                msg=Twist();msg.linear.x=vx;msg.linear.y=vy;msg.angular.z=omega
                self.velocity.publish(msg);time.sleep(.05)
        finally:
            self.velocity.publish(Twist())

    def rotate_short(self,target,run_deadline,angular_limit=.5,tolerance=.12):
        self.client.cancel_all_goals()
        if not self.stopped():raise RuntimeError('Entry rotation requires a stopped vehicle')
        polygon=rospy.get_param('/move_base/local_costmap/footprint')
        if isinstance(polygon,str):polygon=ast.literal_eval(polygon)
        padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
        deadline=rospy.get_time()+15
        try:
            while not rospy.is_shutdown():
                wall=time.monotonic()
                if self.cancelled.is_set() or rospy.get_time()>deadline or self.budget_time()>run_deadline or wall-self.received>2:
                    raise RuntimeError('Entry rotation cancelled, stale odometry or timeout')
                grid=self.fresh_costmap(run_deadline)
                pose=self.odom.pose.pose;q=pose.orientation
                angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                omega=short_turn_command(grid,pose.position.x,pose.position.y,angle,target,polygon,padding,angular_limit,tolerance)
                if omega is None:raise RuntimeError('Shortest entry rotation intersects occupied or unknown space')
                if omega==0:return
                msg=Twist();msg.angular.z=omega;self.velocity.publish(msg)
                time.sleep(.05)
        finally:
            self.velocity.publish(Twist())

    def straight_move(self, x, y, yaw, run_deadline, direction, tolerance):
        self.client.cancel_all_goals()
        polygon=rospy.get_param('/move_base/local_costmap/footprint')
        if isinstance(polygon,str):polygon=ast.literal_eval(polygon)
        padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
        deadline=rospy.get_time()+90
        progress_at=rospy.get_time();previous=self.odom.pose.pose.position
        try:
            while not rospy.is_shutdown():
                now=rospy.get_time();wall_now=time.monotonic()
                if self.cancelled.is_set() or now>deadline or self.budget_time()>run_deadline or wall_now-self.received>2:
                    raise RuntimeError('Straight docking cancelled, stale odometry or timeout')
                grid=self.fresh_costmap(run_deadline)
                pose=self.odom.pose.pose;q=pose.orientation;p=pose.position
                angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                dx,dy=x-p.x,y-p.y;distance=math.hypot(dx,dy)
                if distance<tolerance:return
                if math.hypot(p.x-previous.x,p.y-previous.y)>.005:
                    progress_at=now;previous=p
                if now-progress_at>28:raise RuntimeError('Straight docking made no progress')
                heading_error=math.atan2(math.sin(angle-yaw),math.cos(angle-yaw))
                if abs(heading_error)>.3:raise RuntimeError('Straight docking heading left the corridor')
                v,omega,remaining=straight_velocity(p.x,p.y,angle,x,y,yaw,direction,self.fast,self.efficient,rospy.get_param('~route_flow',False))
                if remaining<=0:raise RuntimeError('Straight docking passed target without alignment')
                safe=True;px,py,pa=p.x,p.y,angle
                cross_squared=max(0.,distance*distance-remaining*remaining)
                stop_along=math.sqrt(max(0.,tolerance*tolerance-cross_squared))
                prediction_time=min(1.,max(0.,remaining-stop_along+.003)/max(abs(v),.001))
                steps=max(10,math.ceil(abs(v)*prediction_time/.01)) if self.efficient else 10
                for i in range(steps+1):
                    safe &= footprint_clear(grid,px,py,pa,polygon,padding)
                    dt=(prediction_time/steps if i<steps else 0.) if self.efficient else min(.1,max(0.,prediction_time-i*.1))
                    px+=v*math.cos(pa)*dt;py+=v*math.sin(pa)*dt;pa+=omega*dt
                msg=Twist()
                if safe:msg.linear.x=v;msg.angular.z=omega
                elif all(footprint_clear(grid,p.x,p.y,angle+omega*.1*i,polygon,padding) for i in range(11)):
                    msg.angular.z=omega
                self.velocity.publish(msg)
                time.sleep(.05)
        finally:
            self.velocity.publish(Twist())


if __name__=='__main__':
    rospy.init_node('round2_control')
    control = Control()
    rospy.spin()
