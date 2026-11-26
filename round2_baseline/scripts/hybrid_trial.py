"""Lidar-guided Hybrid A* through QR, blocks, and the finish."""
from bisect import bisect_left
from concurrent.futures import ThreadPoolExecutor
from contextlib import nullcontext
import json
import math
from pathlib import Path
import threading
import time

import rospkg
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import LaserScan

from bend_path import PreparedPath, rounded_path
from cone_obstacles import detect
from finish_region import contained
from fixed_qr import QR_BOX
from hybrid_path import (BLUE_EXIT_CONE, CANDIDATES, GATE, SPAWN, FootprintMap, center, fixed_walls,
                         connector, docking_path, heading_error, hybrid_astar, legal_layouts, NoPath, qr_reference,
                         qr_pocket_exit, viewpoint_score)
from motion_geometry import yaw_from_orientation
from trajectory_opt import optimize_trajectory

HYBRID_LIMITS = (.90, 1.25, 2.2)
OBSERVATION_LIMITS = (.62, 1.05, 1.8)
BLOCK_CONE_MARGIN = .030
RETURN_CONE_MARGIN = .035
RETURN_STRAIGHT_TAIL = .80
BLOCK_ARM_LINE_Y = 1.75  # Cone-row centerline marked at the block-area entrance.
# The arm-only projection of the stowed-to-forward motion, with mesh clearance.
# The ordinary chassis footprint is checked separately.
FORWARD_ARM_SWEEP = [(-.12, -.09), (.45, -.09), (.45, .09), (-.12, .09)]
# Circumscribe the extended camera arm for any azimuth during retraction.
RETRACTING_ARM_SWEEP = [(.47 * math.cos(i * math.tau / 16), .47 * math.sin(i * math.tau / 16))
                        for i in range(16)]


class ConeTable:
    def __init__(self, frame):
        self.frame = frame
        self.lock = threading.Lock()
        self.states = {}
        self.events = []
        self.streaks = {}
        self.transforms = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.transforms)
        self.subscription = rospy.Subscriber('/nav_scan', LaserScan, self.observe, queue_size=1)
        self.scans = 0

    def snapshot(self):
        with self.lock:
            return dict(self.states)

    def close(self):
        self.subscription.unregister()
        self.listener.unregister()

    def observe(self, scan):
        try:
            transform = self.transforms.lookup_transform(
                self.frame, scan.header.frame_id, scan.header.stamp).transform
        except tf2_ros.TransformException:
            return
        q = transform.rotation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        if max(abs(roll), abs(pitch)) > math.radians(.7):
            return
        x, y = transform.translation.x, transform.translation.y
        cones = set(detect(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                           scan.range_max, x, y, yaw))
        with self.lock:
            self.scans += 1
            for cell in sorted(CANDIDATES - self.states.keys()):
                cx, cy = center(cell)
                seen = True if (cx, cy) in cones else clear_ray_evidence(scan, x, y, yaw, cx, cy)
                previous, count = self.streaks.get(cell, (None, 0))
                count = count + 1 if previous == seen else 1
                self.streaks[cell] = seen, count
                if seen is not None and count >= 3:
                    self.record(cell, seen, scan.header.stamp.to_sec(), (x, y, yaw), 'laser')
            if sum(self.states.values()) == 5:
                for cell in sorted(CANDIDATES - self.states.keys()):
                    self.record(cell, False, scan.header.stamp.to_sec(), (x, y, yaw), 'five_cones')

    def record(self, cell, occupied, stamp, sensor_pose, source):
        self.states[cell] = occupied
        event = dict(cell=cell, occupied=occupied, stamp=stamp, lidar=sensor_pose, source=source)
        self.events.append(event)
        rospy.loginfo('Cone observation %s', json.dumps(event))


def clear_ray_evidence(scan, x, y, yaw, cx, cy):
    distance = math.hypot(cx - x, cy - y)
    if not scan.range_min < distance < min(2.5, scan.range_max - .13):
        return None
    bearing = heading_error(math.atan2(cy - y, cx - x), yaw)
    width = math.asin(min(1., .06 / distance))
    first = math.ceil((bearing - width - scan.angle_min) / scan.angle_increment)
    last = math.floor((bearing + width - scan.angle_min) / scan.angle_increment)
    if first < 0 or last >= len(scan.ranges) or last - first < 4:
        return None
    # In Gazebo +Inf is a valid no-return beam up to range_max. Occlusions end early.
    rays = scan.ranges[first:last + 1]
    clear = sum(r == math.inf or distance + .13 < r <= scan.range_max for r in rays)
    return False if clear >= .9 * len(rays) else None


def actual_pose(control):
    pose = control.odom.pose.pose
    return pose.position.x, pose.position.y, yaw_from_orientation(pose.orientation)


def record_pose(control, target):
    pose = actual_pose(control)
    twist = control.odom.twist.twist
    return dict(stamp=rospy.get_time(), pose=pose, linear_speed=math.hypot(twist.linear.x, twist.linear.y),
                angular_speed=twist.angular.z, position_error=math.dist(pose[:2], target[:2]),
                heading_error=heading_error(pose[2], target[2]))


def save(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2))
    tmp.replace(path)


QR_OBSERVATION = (.11, SPAWN[1], SPAWN[2])


def run_hybrid_trial(control, route, deadline):
    return run_hybrid_segment(control, route, deadline, rospy.get_param('~hybrid_trial'))


def run_full_hybrid(control, route, deadline):
    from control import load_footprint
    from vision_client import vision_client

    if control.navigation_frame != 'odom':
        raise RuntimeError('The hybrid course geometry requires odom')
    if control.vision_enabled:
        vision_client()
    bend_route = [row for row in route if row[0].startswith('bend') and row[0] != 'bend8']
    # Start decelerating on the westbound tangent while the lidar resolves the gate.
    bend_route.append(['bend8', *QR_OBSERVATION])
    bend_reference = control.fixed_plans(bend_route)['prefix']
    prefix_route = [*bend_route[:-1], ['qr_observation', *QR_OBSERVATION]]
    settings = dict(output=rospy.get_param('~hybrid_evidence', ''),
                    optimize=rospy.get_param('~hybrid_optimize', True),
                    world=str(Path(rospkg.RosPack().get_path('car3'))/'world/round2.world'),
                    fast_qr=True, known_layout=[])
    run_hybrid_segment(control, route, deadline, settings,
                       full_prefix=(prefix_route, bend_reference), footprint=load_footprint())


def run_hybrid_segment(control, route, deadline, settings, *, full_prefix=None, footprint=None):
    output = settings.get('output')
    walls = tuple(fixed_walls(settings['world']))
    if footprint is None:
        footprint = rospy.get_param('~parking_geometry/local_costmap/footprint')
    layouts = tuple(legal_layouts())
    points = {row[0]: row[1:] for row in route}
    qr, goal = points['qr'], points['blocks']
    fast_qr = settings.get('fast_qr', False)
    optimize = settings.get('optimize', False)
    initial_pose = actual_pose(control) if full_prefix else SPAWN
    evidence = {'spawn': record_pose(control, initial_pose), 'branches': {}, 'fast_qr': fast_qr,
                'full_route': full_prefix is not None, 'optimized': optimize}
    branch_paths = {}
    for occupied in (() if fast_qr else (False, True)):
        path = qr_reference(SPAWN, occupied, qr)
        possible = frozenset().union(*(layout for layout in layouts if (GATE in layout) == occupied))
        if not FootprintMap(walls, footprint, possible).sweep_clear(path):
            raise RuntimeError(f'QR approach {occupied} failed the offline footprint check')
        branch_paths[occupied] = PreparedPath(path, .32, .65, 1.4)
        evidence['branches']['detour' if occupied else 'direct'] = path

    def cancelled():
        return rospy.is_shutdown() or control.cancelled.is_set() or control.expired(control.budget_time(), deadline)

    table = ConeTable(control.navigation_frame)
    try:
        start = actual_pose(control)
        fallback = QR_OBSERVATION
        if full_prefix is None:
            prefix = rounded_path([start[:2], fallback[:2]], spacing=.014)
            approach_route = [['qr_observation', *fallback]]
            approach = PreparedPath(prefix, .52 if fast_qr else .32, .85 if fast_qr else .65, 1.4)
        else:
            approach_route, approach = full_prefix
        evidence['prefix'] = approach.path
        evidence['departure_stamp'] = rospy.get_time()
        branch = None

        def select():
            nonlocal branch
            x, y, yaw = actual_pose(control)
            if full_prefix and (x > .62 or abs(y - SPAWN[1]) > .10 or abs(heading_error(yaw, SPAWN[2])) > .32):
                return None
            observed = table.snapshot()
            occupied = observed.get(GATE)
            if occupied is None:
                return None
            if full_prefix:
                control.prepare_qr()
            branch = 'detour' if occupied else 'direct'
            evidence['branch'] = dict(name=branch, stamp=rospy.get_time(), pose=actual_pose(control),
                                      gate=GATE, occupied=occupied)
            if fast_qr:
                compatible = match_layouts(layouts, observed)
                if not compatible:
                    raise RuntimeError('Cone observations conflict with all legal layouts before QR')
                union = frozenset().union(*compatible)
                reference = plan_qr(control, qr, walls, footprint, union, compatible, cancelled, evidence)
            else:
                reference = branch_paths[occupied]
            control.report('QR_BRANCH', branch)
            control.report('MOVING', 'qr')
            return 'qr', *qr, reference, None, .12

        control.follow_bends(approach_route, deadline, prepared=approach, select_path=select,
                             check_braking=False, position_tolerance=.022, yaw_tolerance=.18 if full_prefix else .12,
                             local_geometry=full_prefix is not None)
        evidence['observation_wait_sim_s'] = 0.
        if branch is None:
            control.report('OBSERVING', 'qr_gate')
            began = rospy.get_time()
            until = time.monotonic() + 12
            selected = select()
            while selected is None and not cancelled() and time.monotonic() < until:
                time.sleep(.03)
                selected = select()
            evidence['observation_wait_sim_s'] = rospy.get_time() - began
            if selected is None:
                raise RuntimeError('Lidar did not resolve QR gate before either turn')
            control.follow_bends([['qr', *qr]], deadline, prepared=selected[4],
                                 check_braking=False, position_tolerance=.022, yaw_tolerance=.12, resume_path=True,
                                 local_geometry=full_prefix is not None)

        if not contained(*actual_pose(control), footprint, margin=0., box=QR_BOX):
            raise RuntimeError('Vehicle did not stop fully inside the QR box')
        evidence['qr_stop'] = record_pose(control, qr)
        control.report('QR_PARKED', 'full footprint inside the QR box')
        if full_prefix:
            control.recognize('qr')
        else:
            dwell_until = rospy.get_time() + settings['dwell']
            while rospy.get_time() < dwell_until:
                if cancelled():
                    raise RuntimeError('QR dwell cancelled')
                time.sleep(.02)
        if settings.get('qr_only', False):
            return 'QR approach stopped inside the box'
        known = tuple(map(tuple, settings['known_layout']))
        remaining = ((frozenset(known),) if known else match_layouts(layouts, table.snapshot()))
        if not remaining:
            raise RuntimeError('Cone observations conflict with all legal layouts')
        occupied = frozenset().union(*remaining)
        collision_map = FootprintMap(walls, footprint, occupied)
        reverse = (-.72 if GATE in occupied else points['qr_exit'][0], qr[1], qr[2])
        evidence['reverse_start'] = record_pose(control, qr)
        blue_blocked = all(BLUE_EXIT_CONE in layout for layout in remaining)
        green_exit = optimize and blue_blocked and take_green_qr_exit(control, deadline, cancelled,
                                                                        collision_map, qr, evidence)
        if not green_exit:
            if not collision_map.sweep_clear([actual_pose(control), reverse]):
                raise RuntimeError('QR reverse corridor failed the footprint check')
            control.report('MOVING', 'qr_exit')
            control.straight_move(*reverse, deadline, -1, .012, collision_map=collision_map,
                                  reverse_speed_limit=.65 if fast_qr else None)
            if not control.stopped():
                raise RuntimeError('QR reverse did not stop before changing direction')
            evidence['reverse_stop'] = record_pose(control, reverse)
            control.report('ARRIVED', 'qr_exit')
        drive_blocks(control, deadline, cancelled, table, walls, footprint, layouts, known, tuple(goal),
                     evidence, terminal_alignment=fast_qr, local_geometry=full_prefix is not None,
                     optimize=optimize)
        evidence['blocks_stop'] = record_pose(control, evidence['planning']['path'][-1])
        if full_prefix:
            overlap_arm = optimize and control.vision_enabled
            worker_context = ThreadPoolExecutor(max_workers=1) if overlap_arm else nullcontext()
            with worker_context as worker:
                finish = tuple(points['finish'])
                approach = tuple(points['finish_approach'])
                pending = None
                if overlap_arm:
                    stopped_pose = actual_pose(control)
                    remaining = match_layouts(layouts, table.snapshot())
                    if remaining:
                        occupied = frozenset().union(*remaining)
                        pending = worker.submit(plan_finish_while_recognizing, stopped_pose, finish,
                                                math.dist(approach[:2], finish[:2]), walls,
                                                footprint, occupied, cancelled)
                    parameter = '/round2_vision/block_vehicle_pose'
                    overlap = '/round2_vision/block_depart_during_stow'
                    rospy.set_param(parameter, list(stopped_pose))
                    rospy.set_param(overlap, True)
                    try:
                        control.recognize('block')
                    finally:
                        rospy.delete_param(parameter)
                        rospy.set_param(overlap, False)
                else:
                    control.recognize('block')
                drive_finish(control, deadline, cancelled, table, walls, footprint, layouts,
                             approach, finish, evidence, optimize, pending.result() if pending else None)
            return 'Hybrid A* route stopped inside the finish box'
        return 'Hybrid A* trial stopped at the block waypoint'
    finally:
        table.close()
        evidence['observations'] = table.events
        evidence['processed_scans'] = table.scans
        if output:
            save(Path(output), evidence)



def take_green_qr_exit(control, deadline, cancelled, course, qr, evidence):
    start = actual_pose(control)
    try:
        backing, forward = qr_pocket_exit(start, course, cancelled)
    except NoPath:
        # The straight green alternative needs room for both the reverse and the turn.
        for x in (-.84, -.72, -.60):
            reverse = (x, qr[1], qr[2])
            west = (*reverse[:2], math.pi)
            if not course.sweep_clear([start, reverse, west]):
                continue
            control.report('QR_EXIT_BRANCH', 'green_straight')
            control.report('MOVING', 'qr_exit')
            control.straight_move(*reverse, deadline, -1, .012, collision_map=course,
                                  reverse_speed_limit=.65)
            if not control.stopped():
                raise RuntimeError('Green QR reverse did not stop')
            evidence['reverse_stop'] = record_pose(control, reverse)
            control.rotate_short(math.pi, deadline, 1.2, .10, already_stopped=True,
                                 collision_map=course)
            if not control.stopped():
                raise RuntimeError('Green QR departure turn did not stop')
            evidence['qr_exit_branch'] = 'green_straight'
            control.report('ARRIVED', 'qr_exit')
            return True
        return False

    pocket = (*backing[-1][:2], backing[-1][2] - math.pi)
    evidence['qr_exit_branch'] = 'green_pocket'
    evidence['qr_exit_paths'] = dict(reverse_travel=backing, forward=forward)
    control.report('QR_EXIT_BRANCH', 'green_pocket')
    control.follow_bends([['qr_pocket', *pocket]], deadline,
                         prepared=PreparedPath(backing, .42, .80, 1.65, terminal_heading_distance=.14),
                         position_tolerance=.023, yaw_tolerance=.15, check_braking=False,
                         collision_map=course, local_geometry=True, align_at_stop=True,
                         local_velocity=True, drive_reverse=True)
    evidence['reverse_stop'] = record_pose(control, pocket)
    # Join to the right turn from the measured stopped pose.
    current = actual_pose(control)
    for index in range(3, min(len(forward), 26)):
        for curve in connector(current, forward[index], 5.8):
            if course.sweep_clear([current, *curve, *forward[index + 1:]]):
                forward = [current, *curve, *forward[index + 1:]]
                break
        else:
            continue
        break
    else:
        forward, _ = hybrid_astar(current, forward[-1], course, cancelled, limit=1500)
    control.follow_bends([['qr_exit', *forward[-1]]], deadline,
                         prepared=PreparedPath(forward, .56, 1.0, 1.9, terminal_heading_distance=.14),
                         position_tolerance=.028, yaw_tolerance=.18, check_braking=False,
                         collision_map=course, local_geometry=True, align_at_stop=True,
                         local_velocity=True)
    return True


def plan_finish_while_recognizing(start, goal, tail, walls, footprint, occupied, cancelled):
    began = time.monotonic()
    for straight_tail, cone_margin, limit in (
            (min(tail, RETURN_STRAIGHT_TAIL), RETURN_CONE_MARGIN, 14000), (tail, None, 36000)):
        course = FootprintMap(walls, footprint, occupied, cone_margin=cone_margin)
        try:
            path, expanded = docking_path(start, goal, straight_tail, course, cancelled, limit=limit)
        except NoPath:
            continue
        path = optimize_trajectory(path, course, HYBRID_LIMITS, cancelled, straight_tail=straight_tail)
        return path, expanded, occupied, time.monotonic() - began, cone_margin, straight_tail
    return None


def drive_finish(control, deadline, cancelled, table, walls, footprint, layouts, approach, goal,
                 evidence, optimize, preplanned=None):
    remaining = match_layouts(layouts, table.snapshot())
    if not remaining:
        raise RuntimeError('Cone observations conflict with all legal layouts before finish')
    occupied = frozenset().union(*remaining)
    course = FootprintMap(walls, footprint, occupied)

    start = actual_pose(control)
    tail = math.dist(approach[:2], goal[:2])
    plan_started = time.monotonic()
    departure_yaw = None
    turn_sim_s = 0.
    cone_margin = None
    reused = (preplanned is not None and occupied <= preplanned[2]
              and math.dist(start[:2], preplanned[0][0][:2]) < .012
              and abs(heading_error(start[2], preplanned[0][0][2])) < .06
              and course.sweep_clear([start, preplanned[0][0]]))
    if reused:
        path, expanded, _, _, cone_margin, tail = preplanned
    elif optimize:
        try:
            path, expanded = docking_path(start, goal, tail, course, cancelled, limit=36000)
        except NoPath:
            path = None
    else:
        path = None
    if path is None:
        if optimize:
            # The car can rotate in place, unlike the forward search primitives.
            # Try the smallest checked turns first and require a viable onward route.
            for offset in (math.pi / 3, 2 * math.pi / 3, math.pi):
                for direction in (-1, 1):
                    yaw = math.remainder(start[2] + direction * offset, 2 * math.pi)
                    turned_pose = (*start[:2], yaw)
                    if not course.sweep_clear([start, turned_pose]):
                        continue
                    try:
                        docking_path(turned_pose, goal, tail, course, cancelled, limit=6000)
                    except NoPath:
                        continue
                    departure_yaw = yaw
                    break
                if departure_yaw is not None:
                    break
            if departure_yaw is None:
                raise NoPath('No checked departure heading leads to the finish')
        else:
            departure_yaw = -.95
            if not course.sweep_clear([start, (*start[:2], departure_yaw)]):
                departure_yaw = -math.pi / 2
        if optimize and control.vision_enabled:
            while not control.arm_is_stowed():
                if cancelled() or time.monotonic() - plan_started > 8:
                    raise RuntimeError('Arm did not retract before the in-place departure turn')
                time.sleep(.02)
        control.report('TURNING', 'finish')
        turn_began = rospy.get_time()
        control.rotate_short(departure_yaw, deadline, 1.2, .08, already_stopped=True,
                             collision_map=course)
        if not control.stopped():
            raise RuntimeError('Finish departure turn did not stop')
        turn_sim_s = rospy.get_time() - turn_began
        plan_started = time.monotonic()
        path, expanded = docking_path(actual_pose(control), goal, tail, course, cancelled, limit=72000)
    if optimize and not reused:
        path = optimize_trajectory(path, course, HYBRID_LIMITS, cancelled, straight_tail=tail)
    evidence['finish_planning'] = dict(path=path, expanded=expanded,
                                       departure_yaw=departure_yaw,
                                       planned_during_recognition=reused,
                                       cone_margin=cone_margin,
                                       straight_tail=tail,
                                       turn_sim_s=turn_sim_s,
                                       planning_wall_s=preplanned[3] if reused else time.monotonic() - plan_started,
                                       remaining_layouts=len(remaining),
                                       occupied_union=sorted(occupied))
    reference = PreparedPath(path, *HYBRID_LIMITS, terminal_heading_distance=.20)
    motion_guard = None
    if optimize and control.vision_enabled:
        retracting = FootprintMap(walls, RETRACTING_ARM_SWEEP, occupied)
        evidence['arm_stowed_at_finish_motion'] = control.arm_is_stowed()

        def motion_guard(name, prepared, progress, pose):
            if control.arm_is_stowed():
                return None
            if ('arm_retraction_overlap' not in evidence
                    and control.odom.twist.twist.linear.x > .02):
                evidence['arm_retraction_overlap'] = record_pose(control, pose)
            return retracting.clear

    control.follow_bends([['finish', *goal]], deadline, prepared=reference,
                         position_tolerance=.02, yaw_tolerance=.10, check_braking=False,
                         collision_map=FootprintMap(walls, footprint, occupied, margin=0.),
                         local_geometry=True, finish_footprint=footprint, motion_guard=motion_guard)
    evidence['finish_stop'] = record_pose(control, goal)


def plan_qr(control, goal, walls, footprint, occupied, layouts, cancelled, evidence):
    """Search from live odometry and preserve a straight, east-facing docking tail."""
    start = actual_pose(control)
    measured_speed = max(0., control.odom.twist.twist.linear.x)
    obstacle_map = FootprintMap(walls, footprint, occupied, margin=.027)
    began = time.monotonic()
    path, expanded = docking_path(start, goal, .20, obstacle_map, cancelled, limit=4000)
    evidence['qr_planning'] = dict(stamp=rospy.get_time(), path=path, wall_s=time.monotonic() - began,
                                   remaining_layouts=len(layouts), occupied_union=sorted(occupied),
                                   expanded=expanded, start_speed=measured_speed,
                                   distance=sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:])))
    return PreparedPath(path, .67, 1.15, 2.15, start_speed=min(.67, measured_speed),
                        terminal_heading_distance=.18)

def match_layouts(layouts, observations):
    return tuple(layout for layout in layouts
                 if all((cell in layout) == occupied for cell, occupied in observations.items()))


def preplan_observation_exit(plan, viewpoint, layout):
    try:
        return layout, plan(viewpoint, (layout,))
    except NoPath:
        return layout, None


def drive_blocks(control, deadline, cancelled, table, walls, footprint, layouts, known, goal, evidence,
                 *, terminal_alignment=False, local_geometry=False, optimize=False):
    """Track a common route, gathering only the observations needed to select it."""
    visited = []
    evidence['exploration'] = []
    last_observations, last_candidates = None, None

    def candidates():
        nonlocal last_observations, last_candidates
        if known:
            return (frozenset(known),)
        current = table.snapshot()
        if current != last_observations:
            last_observations = current
            last_candidates = match_layouts(layouts, current)
        if not last_candidates:
            raise RuntimeError('Cone observations conflict with all legal layouts')
        return last_candidates

    def plan(pose, remaining, *, explore=False):
        occupied = frozenset().union(*remaining)
        cone_margin = BLOCK_CONE_MARGIN if optimize and not explore else None
        obstacle_map = FootprintMap(walls, footprint, occupied, cone_margin=cone_margin)
        began = time.monotonic()
        score = viewpoint_score(walls, remaining, visited) if explore else None
        limit = 1400 if explore else 36000 if len(remaining) == 1 else 2800
        # The camera must still be able to look toward the production tables.
        headings = [goal[2] + offset for offset in (0., -.35, .35, -.70, .70)] if optimize and not explore else None
        try:
            path, expanded = hybrid_astar(pose, goal, obstacle_map, cancelled, limit=limit,
                                          viewpoint=score, goal_headings=headings)
        except NoPath:
            if explore and optimize:
                path, expanded = hybrid_astar(pose, goal, obstacle_map, cancelled, limit=12000,
                                              viewpoint=score, viewpoint_horizon=2.5)
            elif cone_margin is not None and len(remaining) == 1:
                cone_margin = None
                obstacle_map = FootprintMap(walls, footprint, occupied)
                path, expanded = hybrid_astar(pose, goal, obstacle_map, cancelled, limit=limit,
                                              goal_headings=headings)
            else:
                raise
        if optimize:
            path = optimize_trajectory(path, obstacle_map,
                                       OBSERVATION_LIMITS if explore else HYBRID_LIMITS, cancelled)
        return dict(wall_s=time.monotonic() - began, expanded=expanded, path=path,
                    remaining_layouts=len(remaining), occupied_union=sorted(occupied), cone_margin=cone_margin)

    def runtime_map(plan):
        return FootprintMap(walls, footprint, plan['occupied_union'], margin=0.)

    free_parking = optimize and local_geometry
    arm_prechecked = False
    entered_block_area = False
    prepared_arm = False

    def approach_avoidance_active(name, pose):
        nonlocal entered_block_area, prepared_arm
        if name != 'blocks' or not free_parking:
            return True
        if pose[1] >= BLOCK_ARM_LINE_Y and not entered_block_area:
            entered_block_area = True
            evidence['block_area_entry'] = record_pose(control, pose)
            control.report('ENTERED', 'block_area')
        if entered_block_area and arm_prechecked and not prepared_arm and control.arm_is_stowed():
            prepared_arm = control.prepare_block()
            if prepared_arm:
                evidence['arm_preparation'] = record_pose(control, pose)
                control.report('ARM_PREPARE', 'blocks')
        return not entered_block_area

    # The camera compensates from the measured chassis yaw once parked.
    block_yaw_tolerance = math.pi if free_parking else .10
    block_stop_radius = .075 if free_parking else .025

    def announce(plan, start_speed=0.):
        nonlocal arm_prechecked
        path = plan['path']
        if free_parking and control.vision_enabled:
            arm_clearance = FootprintMap(walls, FORWARD_ARM_SWEEP, plan['occupied_union'], margin=.018)
            crossing = next((i for i, pose in enumerate(path) if pose[1] >= BLOCK_ARM_LINE_Y), None)
            arm_prechecked = crossing is not None and arm_clearance.sweep_clear(path[max(0, crossing - 1):])
        plan['stamp'] = rospy.get_time()
        plan['distance'] = sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:]))
        plan['start_speed'] = start_speed
        evidence['planning'] = plan
        control.report('PLANNED', f"{len(path)} poses; {plan['remaining_layouts']} compatible layouts")
        reference = PreparedPath(path, *HYBRID_LIMITS, start_speed=min(.90, start_speed),
                                 terminal_heading_distance=.10 if terminal_alignment and not free_parking else 0.,
                                 lookahead_time=.22 if free_parking else 0.,
                                 parking_tolerance=block_stop_radius if free_parking else None)
        return reference

    for _ in range(5):
        initial = candidates()
        began = time.monotonic()
        try:
            direct = plan(actual_pose(control), initial)
        except NoPath:
            if known or len(initial) == 1:
                raise
        else:
            reference = announce(direct)
            control.follow_bends([['blocks', *reference.path[-1]]], deadline, prepared=reference, check_braking=False,
                                 position_tolerance=block_stop_radius, yaw_tolerance=block_yaw_tolerance,
                                 collision_map=runtime_map(direct),
                                 align_at_stop=terminal_alignment, local_geometry=local_geometry,
                                 avoidance_active=approach_avoidance_active)
            return

        exploration = plan(actual_pose(control), initial, explore=True)
        exploration.update(stamp=rospy.get_time(), initial_search_wall_s=time.monotonic() - began - exploration['wall_s'])
        evidence['exploration'].append(exploration)
        end = exploration['path'][-1]
        tried = initial
        future = None
        selected = False
        found_route = False
        retried = False
        reference = PreparedPath(exploration['path'], *(OBSERVATION_LIMITS if optimize else (.30, .65, 1.4)))
        attempts = exploration['replans'] = []
        control.report('EXPLORING', f'{len(initial)} compatible layouts')

        def join_candidate(candidate, remaining, *, preplanned=False):
            nonlocal selected
            current = actual_pose(control)
            path = candidate['path']
            occupied = frozenset().union(*remaining)
            checked = FootprintMap(walls, footprint, occupied)
            # Re-anchor a tangent-matched connector at the live odometry pose.
            for index in range(0, min(len(path) - 1, 30), 2):
                for curve in connector(current, path[index], 5.8):
                    if not checked.sweep_clear([current, *curve]):
                        continue
                    joined = dict(candidate, path=[current, *curve, *path[index + 1:]],
                                  remaining_layouts=len(remaining), occupied_union=sorted(occupied),
                                  preplanned=preplanned)
                    speed = max(0., control.odom.twist.twist.linear.x)
                    joined_reference = announce(joined, speed)
                    selected = True
                    control.report('MOVING', 'blocks')
                    return ('blocks', *joined_reference.path[-1], joined_reference,
                            runtime_map(joined), block_yaw_tolerance)
            return None

        speculative = None
        attempted = set()
        cached = {}
        checked_layouts = None
        compatible_paths = []
        with ThreadPoolExecutor(max_workers=2 if optimize else 1) as worker:
            def update():
                nonlocal future, tried, found_route, retried
                nonlocal speculative, checked_layouts, compatible_paths
                current_layouts = candidates()
                if speculative is not None and speculative.done():
                    layout, result = speculative.result()
                    if result is not None:
                        cached[layout] = result
                    speculative = None
                    checked_layouts = None
                if optimize and checked_layouts != current_layouts:
                    occupied = frozenset().union(*current_layouts)
                    checked = FootprintMap(walls, footprint, occupied)
                    compatible_paths = [path for layout, path in cached.items()
                                        if layout in current_layouts and checked.sweep_clear(path['path'])]
                    checked_layouts = current_layouts
                for candidate in compatible_paths:
                    selection = join_candidate(candidate, current_layouts, preplanned=True)
                    if selection is not None:
                        return selection
                if optimize and speculative is None and 1 < len(current_layouts) <= 8:
                    for layout in current_layouts:
                        if layout in attempted:
                            continue
                        attempted.add(layout)
                        speculative = worker.submit(preplan_observation_exit, plan, end, layout)
                        break
                if future is not None and future.done():
                    try:
                        candidate = future.result()
                    except NoPath as exc:
                        attempts.append(dict(stamp=rospy.get_time(), outcome=str(exc)))
                    else:
                        found_route = True
                        selection = join_candidate(candidate, current_layouts)
                        if selection is not None:
                            return selection
                        attempts.append(dict(stamp=rospy.get_time(), outcome='No checked moving splice'))
                    future = None
                if future is None and current_layouts != tried:
                    tried = current_layouts
                    current = actual_pose(control)
                    index = min(range(len(reference.path)),
                                key=lambda i: math.dist(reference.path[i][:2], current[:2]))
                    # Plan ahead of the live pose while following the already checked common prefix.
                    anchor = min(len(reference.path) - 1, bisect_left(reference.arc, reference.arc[index] + .085))
                    retried = False
                    found_route = False
                    future = worker.submit(plan, reference.path[anchor], tried)
                elif future is None and found_route and not retried:
                    retried = True
                    future = worker.submit(plan, actual_pose(control), tried)
                return None

            control.follow_bends([['cone_viewpoint', *end]], deadline,
                                 prepared=reference,
                                 check_braking=False, position_tolerance=.025, yaw_tolerance=.32,
                                 select_path=update, collision_map=runtime_map(exploration),
                                 align_at_stop=terminal_alignment, local_geometry=local_geometry,
                                 local_velocity=optimize, avoidance_active=approach_avoidance_active)
        if selected:
            return
        visited.append(actual_pose(control))
        began = rospy.get_time()
        until = time.monotonic() + 1.2
        while not found_route and candidates() == tried and not cancelled() and time.monotonic() < until:
            time.sleep(.02)
        exploration['wait_sim_s'] = rospy.get_time() - began
    raise RuntimeError('No common block route after five checked observation segments')
