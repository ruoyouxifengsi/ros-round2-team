"""Check the two QR envelopes and a forward plan from a measured-style reverse pose."""
import math
from pathlib import Path
from types import SimpleNamespace

import yaml

from bend_path import PreparedPath
from finish_region import contained
from hybrid_path import (BLUE_EXIT_CONE, FootprintMap, GATE, SPAWN, docking_path, fixed_walls,
                         hybrid_astar, legal_layouts, qr_pocket_exit, qr_reference)
from trajectory_opt import local_speed, optimize_trajectory, trajectory_cost


def main():
    package = Path(__file__).resolve().parents[1]
    walls = tuple(fixed_walls(package.parent/'car3/world/round2.world'))
    footprint = yaml.safe_load((package/'config/navigation.yaml').read_text())['local_costmap']['footprint']
    layouts = tuple(legal_layouts())
    assert len(layouts) == len(set(layouts)) == 611
    for occupied in (False, True):
        possible = frozenset().union(*(layout for layout in layouts if (GATE in layout) == occupied))
        course = FootprintMap(walls, footprint, possible)
        assert course.sweep_clear(qr_reference(SPAWN, occupied, (.20, .25, 0.)))
    fixture = {(3, 4), (1, 3), (1, 1), (2, 1), (3, 1)}
    course = FootprintMap(walls, footprint, fixture)
    start, goal = (-.364, .260, .038), (-1.75, 2.25, 1.57)
    path, _ = hybrid_astar(start, goal, course)
    assert path[0] == start
    assert math.dist(path[-1], goal) < 1e-9
    assert course.sweep_clear(path)
    assert not course.sweep_clear([start, goal])
    docking, _ = docking_path(start, goal, .16, course, lambda: False)
    assert course.sweep_clear(docking)
    assert all(abs(p[2] - goal[2]) < 1e-9 for p in docking[-10:])
    terminal = PreparedPath(docking, .9, 1.25, 2.2, terminal_heading_distance=.18)
    pose = docking[-4]
    _, yaw_rate, _ = terminal.command(len(docking) - 6, *pose[:2], pose[2] + .20, .14)
    assert yaw_rate < 0

    route = {name: (x, y, yaw) for name, x, y, yaw in
             yaml.safe_load((package/'config/route.yaml').read_text())}
    departure = (*route['blocks'][:2], -math.pi / 2)
    finish = route['finish']
    tail = math.dist(route['finish_approach'][:2], finish[:2])
    course = FootprintMap(walls, footprint, {(2, 4), (1, 2), (1, 1), (3, 2), (5, 2)})
    assert course.sweep_clear([route['blocks'], departure])
    path, _ = docking_path(departure, finish, tail, course, lambda: False, limit=72000)
    assert course.sweep_clear(path)
    assert contained(*path[-1], footprint)
    assert not course.clear(finish[0], 2.25, finish[2])
    limits = .90, 1.25, 2.2
    refined = optimize_trajectory(path, course, limits, lambda: False, straight_tail=tail)
    assert refined[0] == path[0] and refined[-1] == path[-1]
    assert refined[-15:] == path[-15:]
    assert course.sweep_clear(refined)
    assert trajectory_cost(refined, limits) <= trajectory_cost(path, limits)
    assert local_speed(lambda *_: True, (0., 0., 0.), (0., 0., 0.), (.62, 0.), 1.6)[0] == .62
    assert local_speed(lambda x, *_: x < .08, (0., 0., 0.), (.36, 0., 0.), (.62, 0.), 1.6)[0] < .62
    parking = PreparedPath([(0., 0., 0.), (1., 0., 0.)], *limits, parking_tolerance=.075)
    velocity = SimpleNamespace(linear=SimpleNamespace(x=.49, y=0.), angular=SimpleNamespace(z=0.))
    assert not parking.ready_to_park(.93, 0., 0., velocity)
    velocity.linear.x = .22
    assert parking.ready_to_park(.93, 0., 0., velocity)
    assert not parking.ready_to_park(1.10, 0., 0., velocity)
    blue_observations = {(4, 2): True, (2, 1): True, GATE: False, (4, 1): False,
                         (3, 1): False, (2, 2): False, (3, 2): False,
                         (1, 1): False, (1, 2): False}
    blue_layouts = [layout for layout in layouts if all(
        (cell in layout) == present for cell, present in blue_observations.items())]
    assert BLUE_EXIT_CONE in frozenset.intersection(*blue_layouts)
    pocket_course = FootprintMap(walls, footprint, frozenset.union(*blue_layouts))
    backing, departure = qr_pocket_exit((.18, .247, .02), pocket_course, lambda: False)
    assert pocket_course.sweep_clear([(x, y, yaw - math.pi) for x, y, yaw in backing])
    assert pocket_course.sweep_clear(departure)
    assert backing[-1][1] > backing[0][1] and departure[-1][0] < departure[0][0]
    print('PASS: checked Hybrid A* paths, time-scored curve refinement, and local braking limits')


if __name__ == '__main__':
    main()
