"""Forward Hybrid A* and swept-footprint checks for the short course trial."""
import heapq
import itertools
import math
import xml.etree.ElementTree as ET

from bend_path import rounded_path


SPAWN = (.75, 1.25, math.pi)
GATE = (5, 2)
BLUE_EXIT_CONE = (4, 2)
FREE_CELL = (5, 1)
CELLS = frozenset((c, r) for r in range(1, 5) for c in range(1, 6 if r <= 2 else 4))
CANDIDATES = CELLS - {FREE_CELL}


def center(cell):
    col, row = cell
    return -2.25 + .5 * (col - 1), .25 + .5 * (row - 1)


def legal_layouts():
    gates = ({(1, 4), (2, 4), (3, 4)}, {(4, 2), GATE})
    for cells in itertools.combinations(sorted(CANDIDATES), 5):
        occupied = frozenset(cells)
        if any(gate <= occupied for gate in gates):
            continue
        free = CELLS - occupied
        pending = [FREE_CELL]
        seen = {FREE_CELL}
        while pending:
            x, y = pending.pop()
            for adjacent in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if adjacent in free and adjacent not in seen:
                    seen.add(adjacent)
                    pending.append(adjacent)
        if seen == free:
            yield occupied


def transform(x, y, heading, px, py):
    c, s = math.cos(heading), math.sin(heading)
    return x + c * px - s * py, y + s * px + c * py


def fixed_walls(world):
    """Read only the published static wall collision boxes, in the odom frame."""
    terrain = ET.parse(world).find(".//model[@name='round2_map']")
    mx, my, _, _, _, ma = map(float, terrain.findtext('pose').split())
    for link in terrain.findall('link'):
        x, y, _, _, _, a = map(float, link.findtext('pose').split())
        x, y = transform(mx, my, ma, x, y)
        a += ma
        for box in link.findall('collision'):
            px, py, _, _, _, pa = map(float, box.findtext('pose').split())
            cx, cy = transform(x, y, a, px, py)
            width, depth, _ = map(float, box.findtext('geometry/box/size').split())
            yield cx, cy, a + pa, width / 2, depth / 2


def bounds(points):
    x, y = zip(*points)
    return min(x), min(y), max(x), max(y)


class FootprintMap:
    """Separating-axis collision tests against fixed walls and candidate cone bases."""

    def __init__(self, walls, footprint, occupied=(), margin=.014, cone_margin=None):
        self.footprint = footprint
        self.margin = margin
        self.obstacles = []
        # The mesh extends 0.99 inches to either side, scaled by five.
        wall_boxes = ((*wall, margin) for wall in walls)
        cone_boxes = ((*center(cell), 0., .126, .126, margin if cone_margin is None else cone_margin)
                      for cell in occupied)
        for x, y, a, hx, hy, obstacle_margin in itertools.chain(wall_boxes, cone_boxes):
            corners = [transform(x, y, a, px, py)
                       for px, py in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
            self.obstacles.append((corners, bounds(corners),
                                   [(math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))],
                                   obstacle_margin))

    def clear(self, x, y, yaw):
        if not (-2.47 < x < 1.49 and 0. < y < 2.96):
            return False
        c, s = math.cos(yaw), math.sin(yaw)
        hull = [(x + c * px - s * py, y + s * px + c * py) for px, py in self.footprint]
        left, bottom, right, top = bounds(hull)
        axes = None
        for box, (xmin, ymin, xmax, ymax), box_axes, margin in self.obstacles:
            if left > xmax + margin or right < xmin - margin or bottom > ymax + margin or top < ymin - margin:
                continue
            if axes is None:
                axes = []
                for (ax, ay), (bx, by) in zip(hull, hull[1:] + hull[:1]):
                    length = math.hypot(bx - ax, by - ay)
                    axes.append(((by - ay) / length, (ax - bx) / length))
            for nx, ny in (*box_axes, *axes):
                a = [nx * px + ny * py for px, py in hull]
                b = [nx * px + ny * py for px, py in box]
                if min(a) > max(b) + margin or min(b) > max(a) + margin:
                    break
            else:
                return False
        return True

    def sweep_clear(self, path):
        # With 0.01 m translation and 0.025 rad rotation samples, the margin
        # also covers the maximum footprint motion between consecutive checks.
        for a, b in zip(path, path[1:]):
            turn = heading_error(b[2], a[2])
            count = max(1, math.ceil(math.dist(a[:2], b[:2]) / .01), math.ceil(abs(turn) / .025))
            for j in range(count):
                f = j / count
                if not self.clear(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * turn):
                    return False
        return self.clear(*path[-1])


def heading_error(a, b):
    return math.remainder(a - b, 2 * math.pi)


def qr_reference(start, occupied, target):
    corner = -.75 if occupied else -.25
    return rounded_path([tuple(start[:2]), (corner, SPAWN[1]), (corner, target[1]), tuple(target[:2])],
                        radius=.18, spacing=.014)


def primitive(pose, curvature, length, step=.018):
    x, y, yaw = pose
    count = math.ceil(length / step)
    if curvature == 0:
        return [(*transform(x, y, yaw, length * i / count, 0.), yaw) for i in range(1, count + 1)]
    result = []
    for i in range(1, count + 1):
        a = yaw + curvature * length * i / count
        result.append((x + (math.sin(a) - math.sin(yaw)) / curvature,
                       y + (math.cos(yaw) - math.cos(a)) / curvature, a))
    return result


def connector(start, goal, max_curvature):
    """Sample a cubic with endpoint tangents and reject excessive curvature."""
    distance = math.dist(start[:2], goal[:2])
    if distance < .025:
        return
    for scale in (.34, .46):
        a = start[:2]
        b = transform(*start, scale * distance, 0.)
        c = transform(*goal, -scale * distance, 0.)
        d = goal[:2]
        samples = []
        count = max(5, math.ceil(distance / .012))
        for j in range(1, count + 1):
            t = j / count
            u = 1 - t
            x, y = (u**3 * p + 3 * u*u*t * q + 3 * u*t*t * r + t**3 * s for p, q, r, s in zip(a, b, c, d))
            dx, dy = (3 * (u*u*(q-p) + 2*u*t*(r-q) + t*t*(s-r)) for p, q, r, s in zip(a, b, c, d))
            ddx, ddy = (6 * (u*(r-2*q+p) + t*(s-2*r+q)) for p, q, r, s in zip(a, b, c, d))
            speed = math.hypot(dx, dy)
            if speed < 1e-5 or abs(dx*ddy - dy*ddx) > max_curvature * speed**3:
                break
            samples.append((x, y, math.atan2(dy, dx)))
        else:
            yield samples


def smooth_path(path, obstacle_map):
    """Replace sections with shorter tangent-matched curves when the entire sweep fits."""
    current = 0
    result = [path[0]]
    while current < len(path) - 1:
        selected = current + 1
        for end in range(min(current + 46, len(path) - 1), current + 5, -5):
            previous = path[current:end + 1]
            length = sum(math.dist(a[:2], b[:2]) for a, b in zip(previous, previous[1:]))
            for curve in connector(previous[0], previous[-1], 5.8):
                candidate = [previous[0], *curve]
                new_length = sum(math.dist(a[:2], b[:2]) for a, b in zip(candidate, candidate[1:]))
                if new_length < length - .002 and obstacle_map.sweep_clear(candidate):
                    selected = end
                    result.extend(curve)
                    break
            if selected == end:
                break
        if selected == current + 1:
            result.append(path[selected])
        current = selected
    return result


class NoPath(RuntimeError):
    """The forward lattice search exhausted its allowed states."""


def hybrid_astar(start, goal, obstacle_map, cancelled=lambda: False, *, limit=36000,
                 viewpoint=None, goal_headings=None, viewpoint_horizon=.66):
    """Search continuous poses with a quantized closed set and forward arc primitives."""
    goals = [goal] if goal_headings is None else [(*goal[:2], yaw) for yaw in goal_headings]
    if viewpoint is None:
        goals = [target for target in goals if obstacle_map.clear(*target)]
    if not obstacle_map.clear(*start) or not goals:
        raise NoPath('Hybrid A* start or goal footprint is blocked')
    curvature_choices = (0., -2.8, 2.8, -5.8, 5.8)
    step = .084

    def key(pose):
        x, y, a = pose
        return round(x / .045), round(y / .045), round(a * 60 / (2 * math.pi)) % 60

    def heuristic(pose):
        return max(math.dist(pose[:2], goal[:2]), min(abs(heading_error(pose[2], target[2])) for target in goals) / 5.8)

    def reconstruct(index):
        parts = []
        while index:
            _, index, segment, _, _ = nodes[index]
            parts.append(segment)
        return [start, *(point for segment in reversed(parts) for point in segment)]

    nodes = [(start, -1, [], 0., 0.)]
    queue = [(heuristic(start), 0)]
    best = {key(start): 0.}
    expanded = 0
    best_view = (0., 0)
    while queue and expanded < limit:
        _, index = heapq.heappop(queue)
        pose, _, _, cost, old_turn = nodes[index]
        if cost > best[key(pose)]:
            continue
        expanded += 1
        if expanded % 32 == 1 and cancelled():
            raise RuntimeError('Hybrid A* cancelled or timed out')
        if viewpoint is not None:
            if cost >= .18:
                rank = viewpoint(pose) - .60 * cost
                if rank > best_view[0]:
                    best_view = rank, index
            if cost >= viewpoint_horizon:
                continue
        elif math.dist(pose[:2], goal[:2]) < .70:
            for target in sorted(goals, key=lambda target: abs(heading_error(target[2], pose[2]))):
                for curve in connector(pose, target, 5.8):
                    if obstacle_map.sweep_clear([pose, *curve]):
                        path = smooth_path([*reconstruct(index), *curve], obstacle_map)
                        if not obstacle_map.sweep_clear(path):
                            raise RuntimeError('Smoothed Hybrid A* path failed revalidation')
                        return path, expanded
        for turn in curvature_choices:
            segment = primitive(pose, turn, step)
            endpoint = segment[-1]
            k = key(endpoint)
            next_cost = cost + step * (1 + .032 * abs(turn)) + .0025 * abs(turn - old_turn)
            if next_cost >= best.get(k, math.inf) or not obstacle_map.sweep_clear([pose, *segment]):
                continue
            best[k] = next_cost
            priority = next_cost if viewpoint is not None else next_cost + 1.45 * heuristic(endpoint)
            heapq.heappush(queue, (priority, len(nodes)))
            nodes.append((endpoint, index, segment, next_cost, turn))
    if viewpoint is not None and best_view[1]:
        return reconstruct(best_view[1]), expanded
    raise NoPath(f'Hybrid A* found no collision-free route after {expanded} expansions')



def docking_path(start, goal, straight_tail, obstacle_map, cancelled, *, limit=36000):
    alignment = (*transform(*goal, -straight_tail, 0.), goal[2])
    tail = primitive(alignment, 0., straight_tail, step=.014)
    if not obstacle_map.sweep_clear([alignment, *tail]):
        raise NoPath('Straight docking tail failed the footprint check')
    path, expanded = hybrid_astar(start, alignment, obstacle_map, cancelled, limit=limit)
    return [*path, *tail], expanded


def qr_pocket_exit(start, obstacle_map, cancelled):
    """Back into the QR pocket, then drive forward clockwise into the lower aisle."""
    class ReversePocket(FootprintMap):
        def clear(self, x, y, travel_yaw):
            return (-.52 < x < .43 and .18 < y < .84
                    and obstacle_map.clear(x, y, travel_yaw - math.pi))

    reverse_map = ReversePocket((), obstacle_map.footprint)
    reverse_start = (*start[:2], start[2] + math.pi)
    for x, y in ((-.25, .54), (-.25, .60), (-.28, .59), (-.22, .61)):
        pocket = (x, y, -math.pi / 2)
        try:
            backing, _ = hybrid_astar(reverse_start, (x, y, math.pi / 2), reverse_map,
                                       cancelled, limit=1000)
            forward, _ = hybrid_astar(pocket, (-.78, .25, math.pi), obstacle_map,
                                       cancelled, limit=1200)
        except NoPath:
            continue
        if all(-.86 < px < -.10 and .18 < py < .84 for px, py, _ in forward):
            return backing, forward
    raise NoPath('No checked reverse-left and forward-right QR exit')

def beam_blocked(origin, target, obstacle):
    """Test a finite beam against the oriented box using segment clipping."""
    x, y, angle, hx, hy = obstacle
    a = transform(0., 0., -angle, origin[0] - x, origin[1] - y)
    b = transform(0., 0., -angle, target[0] - x, target[1] - y)
    entering, leaving = 0., 1.
    for begin, end, half in zip(a, b, (hx, hy)):
        step = end - begin
        if abs(step) < 1e-9:
            if abs(begin) > half:
                return False
            continue
        low, high = sorted(((-half - begin) / step, (half - begin) / step))
        entering, leaving = max(entering, low), min(leaving, high)
        if entering > leaving:
            return False
    return True


def viewpoint_score(walls, layouts, visited):
    union = frozenset().union(*layouts)
    certain = frozenset.intersection(*layouts)
    blockers = (*walls, *((*center(cell), 0., .12, .12) for cell in certain))
    targets = []
    for cell in sorted(union - certain):
        fraction = sum(cell in layout for layout in layouts) / len(layouts)
        targets.append((center(cell), .65 + 1.4 * min(fraction, 1 - fraction)))

    def score(pose):
        if any(math.dist(pose[:2], previous[:2]) < .13 and
               abs(heading_error(pose[2], previous[2])) < .30 for previous in visited):
            return 0.
        sensor = transform(*pose, .148, 0.)
        value = 0.
        for target, weight in targets:
            dx, dy = target[0] - sensor[0], target[1] - sensor[1]
            distance = math.hypot(dx, dy)
            bearing = abs(heading_error(math.atan2(dy, dx), pose[2]))
            if .24 < distance < 2.35 and bearing < 1.43 and not any(
                    beam_blocked(sensor, target, obstacle) for obstacle in blockers):
                value += weight * (1 - .24 * bearing / 1.43 - .22 * distance / 2.35)
        return value

    return score

