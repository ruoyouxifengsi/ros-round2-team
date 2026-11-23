"""Task-preserving sequencing: continuous tracking, reverse staging, forward entry and TEB avoidance."""
import math
import time
import rospy
from motion_geometry import yaw_from_orientation
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist

def ready_to_dock(pose,target):
    x,y,yaw=target;q=pose.orientation
    heading=yaw_from_orientation(q)
    dx,dy=x-pose.position.x,y-pose.position.y
    along=dx*math.cos(yaw)+dy*math.sin(yaw)
    cross=-dx*math.sin(yaw)+dy*math.cos(yaw)
    return 0<along<.09 and abs(cross)<.003 and abs(math.atan2(math.sin(heading-yaw),math.cos(heading-yaw)))<.06

def via_reached(point,x,y):
    name,tx,ty,_=point
    if name=='return':return x>tx
    if name=='qr_approach':return x>tx-.05 and y<ty+.20
    return math.hypot(x-tx,y-ty)<.10

def entry_curve(start,end,heading):
    x,y=start;tx,ty,target_yaw=end
    handle=.55*min(abs(tx-x),abs(ty-y))
    controls=[(x,y),(x+handle*math.cos(heading),y+handle*math.sin(heading)),
              (tx-handle*math.cos(target_yaw),ty-handle*math.sin(target_yaw)),(tx,ty)]
    path=[]
    for i in range(31):
        t=i/30.;u=1-t
        p=[u**3*controls[0][j]+3*u*u*t*controls[1][j]+3*u*t*t*controls[2][j]+t**3*controls[3][j] for j in (0,1)]
        d=[3*u*u*(controls[1][j]-controls[0][j])+6*u*t*(controls[2][j]-controls[1][j])+3*t*t*(controls[3][j]-controls[2][j]) for j in (0,1)]
        path.append(p+[math.atan2(d[1],d[0])])
    return path


def prepare_fixed_paths(route,tuning,start):
    from bend_path import PreparedPath,rounded_path
    prefix=[]
    for point in route:
        if point[0].startswith('bend'):prefix.append(point)
        else:break
    limits=(tuning['max_speed'],tuning['acceleration'],tuning['max_yaw_rate'])
    plans={'prefix_route':prefix}
    if prefix:
        points=[tuple(start)]+[(x,2.72 if name in ('bend1','bend2') else y) for name,x,y,_ in prefix]
        handoff=len(route)>len(prefix) and route[len(prefix)][0]=='qr_approach'
        # This continuation shapes the exit tangent; tracking hands off at bend8 before entering random-cone space.
        if handoff:points.append(tuple(route[len(prefix)][1:3]))
        plans['prefix']=PreparedPath(rounded_path(points),*limits,end_speed=.2 if handoff else 0.)
        plans['handoff_prefix']=handoff
    waypoints = {name: (x, y, yaw) for name, x, y, yaw in route}
    if 'qr_exit' in waypoints and 'cone_entry' in waypoints:
        exit_pose = waypoints['qr_exit']
        curve = entry_curve(exit_pose[:2], waypoints['cone_entry'], exit_pose[2])
        plans['entry'] = PreparedPath(curve, *limits, end_speed=.15)
    return plans

def navigate(control,planner,route,deadline,settings,pass_last=False,announce=True):
    values=dict(settings,free_goal_vel=pass_last)
    if pass_last and route[-1][0]!='finish_approach':values['yaw_goal_tolerance']=math.pi
    planner.update_configuration(values)
    name,x,y,yaw=route[-1]
    goal=MoveBaseGoal();goal.target_pose.header.frame_id=control.navigation_frame
    goal.target_pose.header.stamp=rospy.Time.now()
    goal.target_pose.pose.position.x=x;goal.target_pose.pose.position.y=y
    goal.target_pose.pose.orientation.z=math.sin(yaw/2);goal.target_pose.pose.orientation.w=math.cos(yaw/2)
    if announce:control.report('MOVING',route[0][0])
    control.client.send_goal(goal)
    index=0;last_progress=rospy.get_time();previous=control.odom.pose.pose;approach=False
    while not rospy.is_shutdown():
        if control.manual_supervision:control.fresh_costmap(deadline)
        pose=control.odom.pose.pose;p=pose.position
        if control.cancelled.is_set() or (not control.manual_supervision and time.monotonic()-control.received>2) or control.expired(control.budget_time(),deadline):
            raise RuntimeError('Route navigation cancelled, stale or timed out: '+name)
        moved=math.hypot(p.x-previous.position.x,p.y-previous.position.y)>.01
        a,b=previous.orientation,pose.orientation
        turned=abs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w)<math.cos(.02/2)
        if moved or turned:previous=pose;last_progress=rospy.get_time()
        if control.expired(rospy.get_time(),last_progress+28):raise RuntimeError('Route navigation made no progress for 28 simulation seconds: '+name)
        if name=='qr' and not approach and math.hypot(p.x-x,p.y-y)<.15:
            planner.update_configuration(dict(max_vel_x=.20,min_obstacle_dist=.02));approach=True
        reached_via=via_reached(route[index],p.x,p.y)
        if index<len(route)-1 and reached_via:
            control.report('PASSED',route[index][0]);index+=1;control.report('MOVING',route[index][0])
        if name=='qr' and index==len(route)-1 and control.qr_ready_to_stop((x,y,yaw),deadline):
            control.client.cancel_all_goals()
            if not control.stopped():raise RuntimeError('QR region did not stop')
            if control.qr_ready_to_stop((x,y,yaw),deadline):
                control.report('ARRIVED',name);return
            control.client.send_goal(goal);continue
        if name=='qr' and index==len(route)-1 and ready_to_dock(pose,(x,y,yaw)):
            control.straight_move(x,y,yaw,deadline,1,.0095)
            if not control.stopped():raise RuntimeError('QR final docking did not stop')
            control.report('ARRIVED',name);return
        q=pose.orientation
        angle=yaw_from_orientation(q)
        aligned=name!='finish_approach' or abs(math.atan2(math.sin(angle-yaw),math.cos(angle-yaw)))<.25
        if pass_last and index==len(route)-1 and math.hypot(p.x-x,p.y-y)<.10 and aligned:
            control.report('PASSED',name);return
        state=control.client.get_state()
        if state in (2,3,4,5,8,9):
            if state!=3 and control.manual_supervision:
                control.retry_goal(goal);continue
            if state!=3 or index!=len(route)-1 or not control.stopped():raise RuntimeError('Route goal failed: '+name)
            control.report('ARRIVED',name);return
        time.sleep(.02)
    raise RuntimeError('Shutdown during route navigation')

def follow_fixed_prefix(control, route, plans, deadline):
    prefix = plans.get('prefix_route', [])
    if prefix:
        handoff = plans['handoff_prefix']
        control.follow_bends(
            prefix, deadline, prepared=plans['prefix'], handoff=handoff,
            position_tolerance=.10 if handoff else .023,
            yaw_tolerance=math.pi if handoff else .12,
        )
    return route[len(prefix):]


def enter_cone_area(control, waypoint, entry_plan, deadline):
    if entry_plan is not None:
        control.follow_bends([waypoint], deadline, prepared=entry_plan,
                             position_tolerance=.02, yaw_tolerance=.25, handoff=True)
        return

    # A truncated development route may omit the QR exit waypoint.
    pose = control.odom.pose.pose
    q = pose.orientation
    heading = yaw_from_orientation(q)
    curve = entry_curve((pose.position.x, pose.position.y), waypoint[1:], heading)
    control.follow_bends([waypoint], deadline, path_override=curve,
                         position_tolerance=.04, yaw_tolerance=.25)


def depart_block_area(control, return_waypoint, deadline, align_return):
    _, x, y, _ = return_waypoint
    pose = control.odom.pose.pose
    p, q = pose.position, pose.orientation
    heading = yaw_from_orientation(q)
    turn_yaw = -math.pi / 2
    candidate = math.atan2(y - p.y, x - p.x)
    if align_return and -1.35 < candidate < -.5 and control.can_rotate(candidate, deadline):
        turn_yaw = candidate
    elif not control.can_rotate(turn_yaw, deadline):
        control.straight_move(p.x - .08 * math.cos(heading), p.y - .08 * math.sin(heading),
                              heading, deadline, -1, .008)
        if not control.stopped():
            raise RuntimeError('Recognition-area departure did not stop')

    control.rotate_short(turn_yaw, deadline, 2.4, .12, already_stopped=True)
    if not control.stopped():
        raise RuntimeError('Recognition-area departure turn did not stop')


def run_flow(control, planner, route, deadline):
    settings = rospy.get_param('~flow_settings')
    navigation = settings['navigation']
    plans = control.fixed_plans(route)
    route = follow_fixed_prefix(control, route, plans, deadline)

    index = 0
    while index < len(route):
        waypoint = route[index]
        name, x, y, yaw = waypoint
        next_name = route[index + 1][0] if index + 1 < len(route) else None
        pair = name, next_name

        if pair == ('qr_approach', 'qr'):
            control.prepare_qr()
            qr_settings = dict(navigation, xy_goal_tolerance=.01,
                               yaw_goal_tolerance=.08, min_obstacle_dist=.06)
            navigate(control, planner, route[index:index + 2], deadline, qr_settings)
            control.recognize('qr')
            index += 2
            continue

        if pair == ('cone_exit', 'blocks'):
            navigate(control, planner, route[index:index + 2], deadline, navigation)
            control.recognize('block')
            index += 2
            continue

        if pair == ('return', 'finish_approach'):
            control.report('MOVING', name)
            depart_block_area(control, waypoint, deadline, settings.get('align_return', False))
            return_settings = dict(navigation, xy_goal_tolerance=.06, yaw_goal_tolerance=.12)
            finish_follows = index + 2 < len(route) and route[index + 2][0] == 'finish'
            navigate(control, planner, route[index:index + 2], deadline, return_settings,
                     announce=False, pass_last=finish_follows)
            index += 2
            continue

        if name == 'qr_exit':
            control.report('MOVING', name)
            control.straight_move(x, y, yaw, deadline, -1, .012)
            if not control.stopped():
                raise RuntimeError('QR departure did not stop')
            control.report('ARRIVED', name)
        elif name == 'cone_entry':
            enter_cone_area(control, waypoint, plans.get('entry'), deadline)
        elif name == 'finish':
            control.report('MOVING', name)
            control.straight_move(x, y, yaw, deadline, 1, .07, finish_region=True)
            control.report('ARRIVED', name)
        else:
            navigate(control, planner, [waypoint], deadline, navigation,
                     pass_last=name == 'return' and next_name is not None)
            stage = {'qr': 'qr', 'blocks': 'block'}.get(name)
            if stage:
                control.recognize(stage)
        index += 1

    control.velocity.publish(Twist())
