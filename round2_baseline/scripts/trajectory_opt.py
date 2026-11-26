"""Time-scored continuous curve refinement and collision-checked local speed selection."""
import math

from bend_path import PreparedPath, braking_clear
from hybrid_path import heading_error


def curvature(a, b):
    return heading_error(b[2], a[2]) / math.dist(a[:2], b[:2])


def quintic(start, goal, start_curvature, end_curvature, scale):
    """Quintic Hermite interpolant with prescribed tangent and curvature."""
    distance = math.dist(start[:2], goal[:2])
    derivative = distance * scale
    axis_coefficients = []
    for axis, tangent, normal in ((0, math.cos, lambda yaw: -math.sin(yaw)),
                                  (1, math.sin, math.cos)):
        x0, x1 = start[axis], goal[axis]
        v0, v1 = derivative * tangent(start[2]), derivative * tangent(goal[2])
        a0 = derivative ** 2 * start_curvature * normal(start[2]) / 2
        a1 = derivative ** 2 * end_curvature * normal(goal[2]) / 2
        dx, dv, da = x1 - x0 - v0 - a0, v1 - v0 - 2 * a0, a1 - a0
        axis_coefficients.append((x0, v0, a0, 10*dx - 4*dv + da,
                                  -15*dx + 7*dv - 2*da, 6*dx - 3*dv + da))
    count = max(8, math.ceil(distance / .010))
    points = [start]
    for index in range(1, count):
        t = index / count
        xy = [sum(c * t**i for i, c in enumerate(axis)) for axis in axis_coefficients]
        dx, dy = [sum(i * axis[i] * t**(i-1) for i in range(1, 6))
                  for axis in axis_coefficients]
        if math.hypot(dx, dy) < .025:
            return None
        points.append((*xy, math.atan2(dy, dx)))
    return [*points, goal]


def trajectory_cost(path, limits, start_speed=0.):
    profile = PreparedPath(path, *limits, start_speed=start_speed)
    elapsed = 0.
    steering_change = 0.
    previous_rate = 0.
    for a, b, distance, v0, v1 in zip(path, path[1:], profile.steps,
                                     profile.speeds, profile.speeds[1:]):
        duration = 2 * distance / max(v0 + v1, .01)
        rate = heading_error(b[2], a[2]) / duration
        elapsed += duration
        steering_change += (rate - previous_rate)**2 / duration
        previous_rate = rate
    return elapsed + .014 * steering_change


def optimize_trajectory(path, obstacle_map, limits, cancelled, *, start_speed=0., straight_tail=0.):
    """Optimize continuous curve handles over checked sections of the search result."""
    path = list(path)
    tail_start = len(path) - 1
    tail_length = 0.
    while tail_start and tail_length + 1e-6 < straight_tail:
        tail_length += math.dist(path[tail_start - 1][:2], path[tail_start][:2])
        tail_start -= 1
    cost = trajectory_cost(path, limits, start_speed)
    index = 0
    while index + 12 < tail_start and not cancelled():
        arc = 0.
        end = index
        while end < tail_start and arc < .48:
            arc += math.dist(path[end][:2], path[end + 1][:2])
            end += 1
        a, b = path[index], path[end]
        if math.dist(a[:2], b[:2]) >= .15:
            entry = curvature(path[index - 1], a) if index else curvature(a, path[1])
            exit_turn = curvature(b, path[end + 1]) if end + 1 < len(path) else 0.
            for scale in (.8, 1., 1.25):
                curve = quintic(a, b, entry, exit_turn, scale)
                if not curve or any(math.dist(p[:2], q[:2]) < .001 or
                                    abs(curvature(p, q)) > 5.8 for p, q in zip(curve, curve[1:])):
                    continue
                candidate = [*path[:index], *curve, *path[end + 1:]]
                score = trajectory_cost(candidate, limits, start_speed)
                if score + .003 < cost and obstacle_map.sweep_clear(curve):
                    path, cost = candidate, score
                    tail_start += len(curve) - (end - index + 1)
                    break
        index += 12
    return path


def local_speed(clear, position, measured, command, acceleration):
    """Pick the fastest sampled command whose current and next braking sweeps fit."""
    vx, omega = command
    for fraction in (1., .80, .60, .40):
        candidate = (vx * fraction, 0., omega * fraction)
        if braking_clear(clear, *position, measured, candidate, acceleration):
            return candidate[0], candidate[2]
    return 0., 0.
