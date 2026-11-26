"""Development trial for two preplanned QR approaches selected by lidar."""
import math
import threading
import time

import rospy
import tf
import tf2_ros
from sensor_msgs.msg import LaserScan

from bend_path import PreparedPath, rounded_path
from cone_obstacles import detect
from finish_region import contained
from motion_geometry import yaw_from_orientation


CONE_CENTER = (-.25, .75)  # 2-5 in the course diagram
QR_BOX = (0., .5, 0., .5)


def observe_cone(scan, x, y, yaw):
    """Return occupied, visibly clear, or unknown for the 2-5 cell center."""
    if CONE_CENTER in detect(scan.ranges, scan.angle_min, scan.angle_increment,
                             scan.range_min, scan.range_max, x, y, yaw):
        return True

    dx, dy = CONE_CENTER[0] - x, CONE_CENTER[1] - y
    distance = math.hypot(dx, dy)
    bearing = math.remainder(math.atan2(dy, dx) - yaw, 2 * math.pi)
    if not scan.range_min < distance < min(2.5, scan.range_max - .10):
        return None
    half_width = math.asin(min(1., .06 / distance))
    first = math.ceil((bearing - half_width - scan.angle_min) / scan.angle_increment)
    last = math.floor((bearing + half_width - scan.angle_min) / scan.angle_increment)
    if first < 0 or last >= len(scan.ranges) or last - first < 4:
        return None
    # Free rays must pass behind the expected cone, rather than end short of it.
    rays = scan.ranges[first:last + 1]
    return False if sum(r > distance + .09 for r in rays) >= .85 * len(rays) else None


class ConeObservation:
    """Latch a stationary cone grid cell as soon as three scans agree."""

    def __init__(self, frame):
        self.frame = frame
        self.transforms = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.transforms)
        self.ready = threading.Event()
        self.occupied = None
        self.streak = 0
        self.subscription = rospy.Subscriber('/nav_scan', LaserScan, self.observe, queue_size=1)

    def observe(self, scan):
        if self.ready.is_set():
            return
        try:
            pose = self.transforms.lookup_transform(
                self.frame, scan.header.frame_id, scan.header.stamp).transform
        except tf2_ros.TransformException:
            return
        orientation = pose.rotation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w))
        position = pose.translation
        seen = None
        if max(abs(roll), abs(pitch)) <= math.radians(.7):
            seen = observe_cone(scan, position.x, position.y, yaw)
        if seen is None:
            self.streak = 0
        elif seen == self.occupied:
            self.streak += 1
        else:
            self.occupied = seen
            self.streak = 1
        if self.streak >= 3:
            rospy.loginfo('QR lidar confirmed 2-5 %s (scan stamp %.3f, lidar x=%.3f y=%.3f yaw=%.3f)',
                          'occupied' if seen else 'clear', scan.header.stamp.to_sec(),
                          position.x, position.y, yaw)
            self.ready.set()

    def choose_approach(self, control, deadline):
        until = time.monotonic() + 6
        while not rospy.is_shutdown() and time.monotonic() < until:
            if control.cancelled.is_set() or control.expired(control.budget_time(), deadline):
                raise RuntimeError('Fixed QR branch selection cancelled or timed out')
            if self.ready.wait(.1):
                return self.occupied
        raise RuntimeError('Could not determine whether 2-5 was clear or occupied')


def run_fixed_qr(control, route, deadline):
    plans = control.fixed_plans(route)
    prefix = plans['prefix']
    prefix_route = plans['prefix_route']
    # Keep the original bend8 exit tangent, then stop on its straight for the scan.
    corner = next(i for i, pose in enumerate(prefix.path)
                  if abs(pose[0] + .25) < .001 and pose[1] <= 1.17)
    x, y, yaw = junction = prefix.path[corner]
    limits = prefix.acceleration, prefix.max_yaw_rate
    bend_path = PreparedPath(prefix.path[:corner + 1], max(prefix.speeds), *limits)
    qr = next(row for row in route if row[0] == 'qr')
    target = tuple(qr[1:3])
    direct = [(x, y), (x, target[1]), target]
    detour = [(x, y), (x, 1.09), (-.75, 1.09), (-.75, target[1]), target]
    approaches = {
        False: PreparedPath(rounded_path(direct, radius=.16, spacing=.0125), .32, .65, 1.4),
        True: PreparedPath(rounded_path(detour, radius=.13, spacing=.0125), .28, .65, 1.4),
    }
    observer = ConeObservation(control.navigation_frame)
    try:
        control.follow_bends([*prefix_route, ['qr_junction', *junction]], deadline,
                             prepared=bend_path, check_braking=False,
                             position_tolerance=.025, yaw_tolerance=.22)
        occupied = observer.choose_approach(control, deadline)
    finally:
        observer.subscription.unregister()
        observer.listener.unregister()
    control.report('QR_BRANCH', 'detour: 2-5 occupied' if occupied else 'direct: 2-5 clear')
    control.follow_bends([qr], deadline, prepared=approaches[occupied], check_braking=False,
                         position_tolerance=.025, yaw_tolerance=.15)
    pose = control.odom.pose.pose
    footprint = rospy.get_param('~parking_geometry/local_costmap/footprint')
    if not contained(pose.position.x, pose.position.y, yaw_from_orientation(pose.orientation),
                     footprint, margin=0., box=QR_BOX):
        raise RuntimeError('Fixed QR route stopped outside the QR box')
    return 'Fixed route stopped inside the QR box'
