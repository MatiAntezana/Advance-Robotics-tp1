"""Consenso y geometría de la grilla de presentación."""
import copy
import numpy as np
from tp1_teach_repeat.mapping import reference_grid


def test_consensus_keeps_wall_and_rejects_isolated_hit_without_changing_route():
    wall = np.column_stack((np.full(30, 1.), np.linspace(-.5, .5, 30), np.zeros(30)))
    scans = [dict(pose=[0., 0., 0.], points=wall.tolist()),
             dict(pose=[0., .1, 0.], points=(wall-[0., .1, 0.]).tolist())]
    scans[0]['points'].append([2., 2., 0.])
    original = copy.deepcopy(scans)
    grid, origin = reference_grid(scans, .04)
    x, y = np.floor((np.array([1., 0.])-origin)/.04).astype(int)
    assert grid[y, x] == 100
    x, y = np.floor((np.array([2., 2.])-origin)/.04).astype(int)
    assert grid[y, x] != 100
    assert np.any(grid == 0) and np.any(grid == -1)
    assert scans == original


def test_single_scan_is_visible_and_empty_route_has_no_map():
    points = [[1., i*.02, 0.] for i in range(30)]
    grid, _ = reference_grid([dict(pose=[0., 0., 0.], points=points)])
    assert np.any(grid == 100)
    assert reference_grid([]) is None
