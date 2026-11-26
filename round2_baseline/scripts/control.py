#!/usr/bin/env python3
import json
import threading
import time
import math
import ast
import traceback
import sys
from pathlib import Path
if str(Path(__file__).parent) not in sys.path:sys.path.insert(0,str(Path(__file__).parent))
from navigation_freshness import navigation_data_fresh
import rospy
import actionlib
from dynamic_reconfigure.client import Client
from move_base_msgs.msg import MoveBaseAction
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String, Bool, Empty
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, LaserScan


from motion_geometry import footprint_clear, profiled_turn_velocity, short_turn_command, straight_velocity, yaw_from_orientation


def load_footprint():
    polygon = rospy.get_param('/move_base/local_costmap/footprint')
    return ast.literal_eval(polygon) if isinstance(polygon, str) else polygon


class Control:
    manual_supervision = False

    def __init__(self):
        self.fixed_qr_trial = rospy.get_param('~fixed_qr_trial', False)
        self.client = None if self.fixed_qr_trial else actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.lock = threading.Lock()
        self.running = False
        self.vision_enabled = rospy.get_param('~vision', True)
        self.vision_task = ''
        self.qr_parked_pub = None
        self.qr_action = None
        if self.vision_enabled and rospy.get_param('~qr_preposition', False):
            self.qr_parked_pub = rospy.Publisher('/round2_vision/qr_parked', Empty, queue_size=1)
        self.fast = rospy.get_param('~speed_profile','stable') in ('fast','sprint')
        self.efficient = rospy.get_param('~efficient_approach',False)
        self.navigation_frame = rospy.get_param('~navigation_frame','odom')
        self.block_prepare_pub = None
        self.arm_joints = None
        hybrid_arm = (rospy.get_param('~flow_settings/hybrid_full', False)
                      and rospy.get_param('~hybrid_optimize', True))
        if self.vision_enabled and self.navigation_frame == 'odom' and (
                hybrid_arm or rospy.get_param('~flow_settings/block_preposition', False)):
            self.block_prepare_pub = rospy.Publisher('/round2_vision/prepare_block', Empty, queue_size=1)
        self.sim_timeouts = rospy.get_param('~timeout_clock','wall')=='sim'
        self.manual_supervision = rospy.get_param('~manual_supervision',False)
        self.simulated_time = rospy.get_param('/use_sim_time',False)
        self.cancelled = threading.Event()
        self.odom = None
        self.received = 0
        self.guard = None
        self.guard_received = 0
        self.costmap = None
        self.map_received = 0
        self.scan_received = self.scan_stamp = 0.
        self.prepared_cache = None
        self.velocity = rospy.Publisher('/nav_cmd_vel',Twist,queue_size=1)
        self.status = rospy.Publisher('/baseline/status', String, queue_size=10, latch=True)
        self.ready = rospy.Publisher('/baseline/ready', Bool, queue_size=1, latch=True)
        rospy.Subscriber(rospy.get_param('~pose_topic','/odom'), Odometry, self.observe)
        rospy.Subscriber('/attitude_guard/status', String, self.observe_guard)
        if self.vision_enabled and hybrid_arm:
            self.arm_stow_pose = rospy.get_param('/round2_vision/stow_pose')
            rospy.Subscriber('/joint_states', JointState, self.observe_arm, queue_size=1)
        if self.client is not None:
            rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,self.observe_map,queue_size=1)
        if self.manual_supervision:
            rospy.Subscriber('/nav_scan',LaserScan,self.observe_scan,queue_size=1)
        rospy.Service('/nav_start', SetBool, self.start)
        rospy.Timer(rospy.Duration(.5),lambda _:self.ready.publish(Bool(data=self.preflight() is None)))
        self.report('IDLE', 'Waiting for start')

    def observe_guard(self, msg):
        self.guard=json.loads(msg.data)
        self.guard_received=time.monotonic()

    def observe_map(self, msg):
        self.costmap=msg;self.map_received=time.monotonic()

    def observe_scan(self, msg):
        self.scan_stamp=msg.header.stamp.to_sec();self.scan_received=time.monotonic()

    def expired(self, now, deadline):
        return not self.manual_supervision and now>deadline

    def retry_goal(self, goal):
        self.velocity.publish(Twist())
        rospy.logwarn_throttle(5.,'Navigation goal interrupted; supervised mode will retry until cancelled')
        until=time.monotonic()+1.
        while time.monotonic()<until:
            if self.cancelled.is_set() or rospy.is_shutdown():raise RuntimeError('Cancelled')
            time.sleep(.05)
        self.fresh_costmap(float('inf'))
        goal.target_pose.header.stamp=rospy.Time.now()
        self.client.send_goal(goal)

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
        self.fixed_plans()
        return None

    def fixed_plans(self, route=None):
        from route_flow import prepare_fixed_paths
        if route is None:
            route = rospy.get_param('~route')
        tuning=rospy.get_param('~bend_settings');start=rospy.get_param('~start_position',[-.75,1.75])
        key=json.dumps([route,tuning,start],sort_keys=True)
        cached=getattr(self,'prepared_cache',None)
        if cached is None or cached[0]!=key:
            cached=(key,prepare_fixed_paths(route,tuning,start));self.prepared_cache=cached
        return cached[1]

    def observe(self, msg):
        self.odom = msg
        self.received = time.monotonic()

    def budget_time(self):
        # Only the whole-run budget follows timeout_clock; motion watchdogs use ROS time.
        return rospy.get_time() if self.sim_timeouts else time.monotonic()

    def report(self, state, detail):
        record = dict(state=state, detail=detail, stamp=rospy.Time.now().to_sec(), control_only=not self.vision_enabled)
        self.status.publish(json.dumps(record))
        rospy.loginfo(json.dumps(record))

    def start(self, req):
        with self.lock:
            if not req.data:
                self.cancelled.set()
                if self.client is not None:
                    self.client.cancel_all_goals()
                self.velocity.publish(Twist())
                return SetBoolResponse(True, 'Cancellation requested')
            reason=self.preflight()
            if reason:
                return SetBoolResponse(False, reason)
            if self.client is not None and not self.client.wait_for_server(rospy.Duration(1)):
                return SetBoolResponse(False, 'Navigation unavailable')
            self.running = True
            self.cancelled.clear()
            threading.Thread(target=self.run, daemon=True).start()
            return SetBoolResponse(True, 'Run started')

    def stopped(self, timeout=None):
        deadline = time.monotonic()+(timeout if timeout is not None else 5)
        stable = None
        while not rospy.is_shutdown() and not self.cancelled.is_set():
            if (timeout is not None or not self.manual_supervision) and time.monotonic()>=deadline:break
            self.velocity.publish(Twist())
            v = self.odom.twist.twist
            if time.monotonic()-self.received < 1 and math.hypot(v.linear.x,v.linear.y)<0.02 and abs(v.angular.z)<0.05:
                stable = stable or time.monotonic()
                if time.monotonic()-stable > 0.5:
                    return True
            else:
                stable = None
            time.sleep(0.05)
        return False

    def prepare_qr(self):
        if self.qr_parked_pub is not None:
            from vision_client import start_image_task
            self.qr_action = start_image_task('qr_approach')

    def observe_arm(self, msg):
        self.arm_joints = (time.monotonic(), dict(zip(msg.name, msg.position)))

    def arm_is_stowed(self):
        observation = self.arm_joints
        if observation is None or time.monotonic() - observation[0] > .5:
            return False
        return all(abs(observation[1].get(f'arm_joint{index}', math.inf) - target) < .035
                   for index, target in enumerate(self.arm_stow_pose, 1))

    def prepare_block(self):
        if self.block_prepare_pub is None or not self.block_prepare_pub.get_num_connections():
            return False
        self.block_prepare_pub.publish(Empty())
        rospy.loginfo('Requested first block camera pose during the approach')
        return True

    def recognize(self, stage):
        if not self.vision_enabled:
            self.report('IMAGE_SKIPPED', stage)
            return
        from vision_client import finish_image_task, image_task
        if stage == 'qr':
            if self.qr_action is None:
                result = image_task(self, stage)
            else:
                from finish_region import contained
                pose = self.odom.pose.pose
                p = pose.position
                if not contained(p.x,p.y,yaw_from_orientation(pose.orientation),
                                 load_footprint(),margin=0.,box=(0.,.5,0.,.5)):
                    raise RuntimeError('Stopped outside the QR zone')
                self.qr_parked_pub.publish(Empty())
                result = finish_image_task(self,stage,self.qr_action)
                self.qr_action = None
            self.vision_task = result.task_label
        else:
            image_task(self, stage, self.vision_task)

    def run(self):
        self.vision_task = ''
        try:
            deadline = self.budget_time() + rospy.get_param('~run_timeout', 540.0)
            route = rospy.get_param('~route')
            if self.fixed_qr_trial and rospy.has_param('~hybrid_trial'):
                from hybrid_trial import run_hybrid_trial
                completion = run_hybrid_trial(self, route, deadline)
            elif self.fixed_qr_trial:
                from fixed_qr import run_fixed_qr
                completion = run_fixed_qr(self, route, deadline)
            else:
                from route_flow import run_flow
                planner = Client('/move_base/TebLocalPlannerROS', timeout=None if self.manual_supervision else 5)
                completion = run_flow(self, planner, route, deadline)
            self.report('CONTROL_ROUTE_DONE', completion or 'Route stopped at finish')
        except Exception as exc:
            if not self.cancelled.is_set():traceback.print_exc()
            if self.client is not None:
                self.client.cancel_all_goals()
            self.stopped(timeout=5)
            self.report('CANCELLED' if self.cancelled.is_set() else 'FAILED',str(exc))
        finally:
            if self.qr_action is not None:
                self.qr_action[0].cancel_goal()
                self.qr_action = None
            with self.lock:
                self.running = False

    def follow_bends(self,route,run_deadline,path_override=None,position_tolerance=.023,yaw_tolerance=.12,
                     prepared=None,handoff=False,check_braking=True,select_path=None,collision_map=None,
                     resume_path=False, align_at_stop=False, local_geometry=False, finish_footprint=None,
                     local_velocity=False, motion_guard=None, avoidance_active=None, drive_reverse=False):
        from bend_path import advance_bend_passages, bend_passage_lines, braking_clear, path_command, validate_settings

        if local_velocity or motion_guard is not None:
            from trajectory_opt import local_speed

        p = self.odom.pose.pose.position
        path = prepared.path if prepared is not None else path_override
        if path is None:
            raise ValueError('Bend tracking requires a prepared path or an explicit reference')
        tuning=rospy.get_param('~bend_settings')
        validate_settings(tuning)
        flow = rospy.get_param('~flow_settings', {})
        check_braking = check_braking and not (flow.get('debug_bends_only') and flow.get('debug_no_bend_brake'))
        navigation_enabled = not local_geometry and not getattr(self, 'fixed_qr_trial', False)
        if local_geometry and self.client is not None:
            self.client.cancel_all_goals()
        change_resolution = False
        if navigation_enabled:
            costmap_client=Client('/move_base/local_costmap',timeout=None if self.manual_supervision else 5)
            old_resolution=costmap_client.get_configuration()['resolution']
            change_resolution=abs(old_resolution-.01)>1e-6
            polygon = load_footprint()
            padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
            self.client.cancel_all_goals()
        elif check_braking:
            raise ValueError('Braking collision checks require move_base')
        try:
            if change_resolution:costmap_client.update_configuration({'resolution':.01})
            ready=time.monotonic()+5
            while navigation_enabled and (self.costmap is None or abs(self.costmap.info.resolution-.01)>1e-6):
                self.velocity.publish(Twist())
                if self.cancelled.is_set() or rospy.is_shutdown():raise RuntimeError('Cancelled')
                if self.expired(time.monotonic(),ready):raise RuntimeError('Bend costmap resolution update timed out')
                time.sleep(.02)
            name,x,y,yaw=route[-1]
            self.report('MOVING',route[0][0])
            index=0;progress=0;deadline=rospy.get_time()+60
            if resume_path:
                progress = min(range(len(path)), key=lambda i: math.dist(path[i][:2], (p.x, p.y)))
            last_progress=rospy.get_time();previous=p
            last_xy = (p.x,p.y)
            passages = bend_passage_lines(route, path[0][:2])
            extra_clear = None
            while not rospy.is_shutdown():
                grid=self.fresh_costmap(run_deadline) if navigation_enabled else None
                now=time.monotonic();pose=self.odom.pose.pose;position=pose.position;q=pose.orientation
                if self.cancelled.is_set():raise RuntimeError('Continuous bend path cancelled')
                if now-self.received>2:
                    if not self.manual_supervision:raise RuntimeError('Continuous bend path odometry stale for 2 wall seconds')
                    self.velocity.publish(Twist());continue
                if self.expired(self.budget_time(),run_deadline):raise RuntimeError('Whole-run time budget exceeded during bends')
                if self.expired(rospy.get_time(),deadline):raise RuntimeError('Continuous bend path exceeded 60 simulation seconds')
                if math.hypot(position.x-previous.x,position.y-previous.y)>.01:
                    last_progress=rospy.get_time();previous=position
                if self.expired(rospy.get_time(),last_progress+5):raise RuntimeError('Continuous bend path made no measured progress for 5 simulation seconds')
                current_xy = (position.x,position.y)
                passed = advance_bend_passages(passages,index,last_xy,current_xy)
                last_xy = current_xy
                while index<passed:
                    self.report('PASSED',route[index][0]);index+=1
                    self.report('MOVING',route[index][0])
                angle=yaw_from_orientation(q)
                if select_path is not None and index == len(route) - 1:
                    selection = select_path()
                    if selection is not None:
                        name,x,y,yaw,prepared,collision_map,yaw_tolerance = selection
                        route = [[name, x, y, yaw]]
                        index, passages = 0, []
                        progress = min(range(len(prepared.path)),
                                       key=lambda i: math.dist(prepared.path[i][:2], current_xy))
                        select_path = None
                measured=self.odom.twist.twist
                parking = prepared is not None and prepared.parking_tolerance is not None
                reached = (prepared.ready_to_park(position.x, position.y, angle, measured) if parking
                           else math.hypot(position.x-x,position.y-y)<position_tolerance)
                if reached:
                    if handoff and index == len(route) - 1 and abs(math.atan2(math.sin(angle-yaw),math.cos(angle-yaw)))<yaw_tolerance:
                        if not check_braking or braking_clear(lambda px,py,pa:footprint_clear(grid,px,py,pa,polygon,padding),
                            position.x,position.y,angle,(measured.linear.x,measured.linear.y,measured.angular.z),(0.,0.,0.),tuning['output_acceleration']):
                            self.report('PASSED',route[-1][0]);return
                    break
                if prepared:
                    if drive_reverse:
                        speed, omega, progress = prepared.command(
                            progress, position.x, position.y, angle + math.pi, -measured.linear.x)
                        vx = -speed
                    else:
                        vx,omega,progress=prepared.command(progress,position.x,position.y,angle,measured.linear.x)
                else:
                    vx,omega,progress=path_command(path,progress,position.x,position.y,angle,
                        tuning['max_speed'],tuning['acceleration'],tuning['max_yaw_rate'])
                actual = (position.x, position.y, angle)
                check_motion = avoidance_active is None or avoidance_active(name, actual)
                safe=not check_motion or not check_braking or braking_clear(lambda px,py,pa:footprint_clear(grid,px,py,pa,polygon,padding),
                    position.x,position.y,angle,(measured.linear.x,measured.linear.y,measured.angular.z),
                    (vx,0.,omega),tuning['output_acceleration'])
                if check_motion and collision_map is not None and not collision_map.clear(*actual):
                    raise RuntimeError('Measured pose left the checked footprint corridor: ' + name)
                extra_clear = motion_guard(name, prepared, progress, actual) if check_motion and motion_guard else None
                if check_motion and (extra_clear is not None or (local_velocity and collision_map is not None)):
                    def clear(px, py, pa):
                        return ((collision_map is None or collision_map.clear(px, py, pa))
                                and (extra_clear is None or extra_clear(px, py, pa)))
                    vx, omega = local_speed(
                        clear, actual,
                        (measured.linear.x, measured.linear.y, measured.angular.z),
                        (vx, omega), tuning['output_acceleration'])
                msg=Twist()
                if safe:
                    msg.linear.x=vx;msg.angular.z=omega
                self.velocity.publish(msg);time.sleep(.02)
            self.velocity.publish(Twist())
            if index!=len(route)-1 or not self.stopped():
                raise RuntimeError('Continuous bend path failed or missed a waypoint: '+route[index][0])
            if prepared is not None and prepared.parking_tolerance is not None:
                final = self.odom.pose.pose.position
                error = math.dist((x, y), (final.x, final.y))
                if error > prepared.parking_tolerance:
                    raise RuntimeError(f'Vehicle stopped {error:.3f} m from the block parking point')
            if navigation_enabled:
                yaw_rate = prepared.max_yaw_rate if prepared is not None else tuning['max_yaw_rate']
                turned=self.rotate_short(yaw,run_deadline,yaw_rate,yaw_tolerance,already_stopped=True)
                if turned and not self.stopped():raise RuntimeError('Bend exit rotation did not stop')
            else:
                angle = yaw_from_orientation(self.odom.pose.pose.orientation)
                if abs(math.remainder(angle - yaw, 2 * math.pi)) >= yaw_tolerance:
                    if not align_at_stop or collision_map is None:
                        raise RuntimeError('Fixed path stopped with the wrong heading: '+name)
                    self.report('ALIGNING', name)
                    self.rotate_short(yaw, run_deadline, .8, yaw_tolerance * .7, already_stopped=True,
                                      collision_map=collision_map, extra_clear=extra_clear)
                    if not self.stopped():
                        raise RuntimeError('Terminal heading correction did not stop')
                    final = self.odom.pose.pose
                    error = math.remainder(yaw_from_orientation(final.orientation) - yaw, 2 * math.pi)
                    if math.dist((x, y), (final.position.x, final.position.y)) >= position_tolerance or abs(error) >= yaw_tolerance:
                        raise RuntimeError('Terminal heading correction did not settle within the parking tolerances')
            if finish_footprint is not None:
                from finish_region import contained
                final = self.odom.pose.pose
                parking_pose = (final.position.x, final.position.y, yaw_from_orientation(final.orientation))
                if not contained(*parking_pose, finish_footprint):
                    raise RuntimeError('Vehicle did not stop fully inside the finish box')
                if collision_map is not None and not collision_map.clear(*parking_pose):
                    raise RuntimeError('Stopped vehicle intersects an obstacle at the finish')
            self.report('ARRIVED',route[-1][0])
        finally:
            self.velocity.publish(Twist())
            if change_resolution:costmap_client.update_configuration({'resolution':old_resolution})

    def fresh_costmap(self,deadline):
        started=time.monotonic();waiting=False
        while not rospy.is_shutdown():
            wall=time.monotonic();grid=self.costmap
            if grid is not None and grid.header.frame_id.lstrip('/')!=self.navigation_frame:
                raise RuntimeError('Costmap frame differs from the navigation frame')
            if self.cancelled.is_set():raise RuntimeError('Cancelled while checking navigation data')
            if self.expired(self.budget_time(),deadline):raise RuntimeError('Whole-run time budget exceeded while checking navigation data')
            map_age=wall-self.map_received;odom_age=wall-self.received
            sensors_fresh=True;map_fresh=map_age<=2
            if self.manual_supervision:
                sim=rospy.get_time()
                simulated=getattr(self,'simulated_time',False)
                sensors_fresh=navigation_data_fresh(sim,wall,self.scan_stamp,self.scan_received,simulated)
                map_fresh=grid is not None and navigation_data_fresh(sim,wall,grid.header.stamp.to_sec(),self.map_received,simulated)
            if grid is not None and map_fresh and odom_age<=2 and sensors_fresh:
                if waiting:rospy.loginfo('Navigation data recovered; resuming checked motion')
                return grid
            self.velocity.publish(Twist())
            if self.expired(wall,started+5):
                raise RuntimeError('Navigation data unavailable after 5 wall seconds; vehicle held stopped '
                                   '(map age %.3fs, odometry age %.3fs)'%(map_age,odom_age))
            if not waiting:
                rospy.logwarn('Navigation data stale: holding zero velocity (map age %.3fs, odometry age %.3fs, sensors fresh %s)',map_age,odom_age,sensors_fresh)
                waiting=True
            time.sleep(.05)
        raise RuntimeError('Shutdown while waiting for a fresh costmap')

    def qr_ready_to_stop(self,target,run_deadline):
        from finish_region import can_stop_inside
        self.fresh_costmap(run_deadline)
        pose=self.odom.pose.pose;p=pose.position;q=pose.orientation
        angle=yaw_from_orientation(q)
        position_tolerance = .035 if self.qr_action is not None else .02
        if math.hypot(p.x-target[0],p.y-target[1])>=position_tolerance or abs(math.atan2(math.sin(angle-target[2]),math.cos(angle-target[2])))>=.08:
            return False
        polygon = load_footprint()
        v=self.odom.twist.twist
        return can_stop_inside(p.x,p.y,angle,(v.linear.x,v.linear.y,v.angular.z),polygon,box=(0.,.5,0.,.5))

    def can_rotate(self,target,run_deadline):
        grid=self.fresh_costmap(run_deadline)
        polygon = load_footprint()
        padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
        pose=self.odom.pose.pose;q=pose.orientation
        angle=yaw_from_orientation(q)
        return short_turn_command(grid,pose.position.x,pose.position.y,angle,target,polygon,padding) is not None

    def rotate_short(self,target,run_deadline,angular_limit=.5,tolerance=.12,already_stopped=False,
                     collision_map=None, extra_clear=None):
        if collision_map is None:
            self.client.cancel_all_goals()
        if already_stopped:
            v=self.odom.twist.twist
            already_stopped=time.monotonic()-self.received<1 and math.hypot(v.linear.x,v.linear.y)<.02 and abs(v.angular.z)<.05
        if not already_stopped and not self.stopped():raise RuntimeError('Entry rotation requires a stopped vehicle')
        if collision_map is None:
            polygon = load_footprint()
            padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
        deadline=rospy.get_time()+15
        previous=0.;last_time=rospy.get_time();turned=False;large_turn=None
        try:
            while not rospy.is_shutdown():
                if collision_map is None:
                    grid = self.fresh_costmap(run_deadline)
                stale = collision_map is not None and time.monotonic() - self.received > 2
                if (self.cancelled.is_set() or stale or self.expired(rospy.get_time(),deadline) or
                        self.expired(self.budget_time(),run_deadline)):
                    raise RuntimeError('Entry rotation cancelled, stale odometry or timeout')
                pose=self.odom.pose.pose;q=pose.orientation
                angle=yaw_from_orientation(q)
                error=math.atan2(math.sin(target-angle),math.cos(target-angle))
                if collision_map is None:
                    omega=short_turn_command(grid,pose.position.x,pose.position.y,angle,target,polygon,padding,angular_limit,tolerance)
                elif abs(error) < tolerance:
                    omega = 0.
                else:
                    a = (pose.position.x, pose.position.y, angle)
                    b = (*a[:2], target)
                    safe = collision_map.sweep_clear([a, b])
                    if safe and extra_clear is not None:
                        samples = max(1, math.ceil(abs(error) / .025))
                        safe = all(extra_clear(*a[:2], angle + error * i / samples)
                                   for i in range(samples + 1))
                    omega = max(-angular_limit, min(angular_limit, 2.6 * error)) if safe else None
                if omega is None:raise RuntimeError('Shortest entry rotation intersects occupied or unknown space')
                if omega==0:return turned
                if large_turn is None:large_turn=abs(error)>.35
                if large_turn:
                    now=rospy.get_time();dt=max(0.,min(.1,now-last_time));last_time=now
                    omega=profiled_turn_velocity(error,previous,dt,angular_limit,tolerance)
                    previous=omega
                msg=Twist();msg.angular.z=omega;self.velocity.publish(msg)
                turned|=abs(omega)>0
                time.sleep(.05)
        finally:
            self.velocity.publish(Twist())

    def straight_move(self, x, y, yaw, run_deadline, direction, tolerance, finish_region=False,
                      collision_map=None, reverse_speed_limit=None):
        if collision_map is None:
            self.client.cancel_all_goals()
            polygon = load_footprint()
            padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
        if finish_region:
            from finish_region import contained,can_stop_inside
        deadline=rospy.get_time()+90
        progress_at=rospy.get_time();previous=self.odom.pose.pose.position
        try:
            while not rospy.is_shutdown():
                if collision_map is None:
                    grid = self.fresh_costmap(run_deadline)
                    clear = lambda px, py, a: footprint_clear(grid, px, py, a, polygon, padding)
                else:
                    clear = collision_map.clear
                now=rospy.get_time();wall_now=time.monotonic()
                stale = collision_map is not None and wall_now - self.received > 2
                if self.cancelled.is_set() or stale or self.expired(now, deadline) or self.expired(self.budget_time(), run_deadline):
                    raise RuntimeError('Straight docking cancelled, stale odometry or timeout')
                pose=self.odom.pose.pose;q=pose.orientation;p=pose.position
                angle=yaw_from_orientation(q)
                dx,dy=x-p.x,y-p.y;distance=math.hypot(dx,dy)
                if finish_region:
                    v=self.odom.twist.twist
                    if can_stop_inside(p.x,p.y,angle,(v.linear.x,v.linear.y,v.angular.z),polygon):
                        if not self.stopped():raise RuntimeError('Finish region did not stop')
                        pose=self.odom.pose.pose;p=pose.position;q=pose.orientation
                        angle=yaw_from_orientation(q)
                        if contained(p.x,p.y,angle,polygon):return
                        continue
                elif distance<tolerance:return
                if math.hypot(p.x-previous.x,p.y-previous.y)>.005:
                    progress_at=now;previous=p
                if self.expired(now,progress_at+28):raise RuntimeError('Straight docking made no progress')
                heading_error=math.atan2(math.sin(angle-yaw),math.cos(angle-yaw))
                if abs(heading_error)>.3:raise RuntimeError('Straight docking heading left the corridor')
                v,omega,remaining=straight_velocity(p.x,p.y,angle,x,y,yaw,direction,self.fast,self.efficient)
                if remaining<=0:raise RuntimeError('Straight docking passed target without alignment')
                if reverse_speed_limit is not None and direction < 0 and v < 0:
                    v = -min(reverse_speed_limit, 5. * remaining, math.sqrt(2.2 * remaining))
                safe=True;px,py,pa=p.x,p.y,angle
                cross_squared=max(0.,distance*distance-remaining*remaining)
                stop_along=math.sqrt(max(0.,tolerance*tolerance-cross_squared))
                prediction_time=min(1.,max(0.,remaining-stop_along+.003)/max(abs(v),.001))
                steps=max(10,math.ceil(abs(v)*prediction_time/.01)) if self.efficient else 10
                for i in range(steps+1):
                    safe &= clear(px,py,pa)
                    dt=(prediction_time/steps if i<steps else 0.) if self.efficient else min(.1,max(0.,prediction_time-i*.1))
                    px+=v*math.cos(pa)*dt;py+=v*math.sin(pa)*dt;pa+=omega*dt
                msg=Twist()
                if safe:msg.linear.x=v;msg.angular.z=omega
                elif all(clear(p.x,p.y,angle+omega*.1*i) for i in range(11)):
                    msg.angular.z=omega
                self.velocity.publish(msg)
                time.sleep(.05)
        finally:
            self.velocity.publish(Twist())


if __name__=='__main__':
    rospy.init_node('round2_control')
    control = Control()
    rospy.spin()
