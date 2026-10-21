"""Verifica geometría, sentido de ICP, fusión temporal y convergencia."""
import math
import numpy as np
import pytest
from tp1_teach_repeat.core import (
    ICP, between, compose, inverse, interpolate, load_route,
    polar_command, save_route, transform_points, wheel_step,
)


def test_se2_and_differential_arc():
    a, b = np.array([1., -2., 1.4]), np.array([.3, .5, -.7])
    np.testing.assert_allclose(between(a, compose(a, b)), b, atol=1e-12)
    np.testing.assert_allclose(compose(a, inverse(a)), 0, atol=1e-12)
    np.testing.assert_allclose(wheel_step(1., 1., .033, .16), [.033, 0., 0.])
    angle = math.pi/2
    step = wheel_step(0., angle*.16/.033, .033, .16)
    np.testing.assert_allclose(step, [.08, .08, angle], atol=1e-12)


def test_delayed_lidar_keeps_motion_after_measurement():
    history = [(10, np.zeros(3)), (20, np.array([1., 0., 0.])),
               (30, np.array([2., 0., 0.]))]
    measured = np.array([.9, .1, 0.])
    w = interpolate(history, 20)
    correction = compose(measured, inverse(w))
    np.testing.assert_allclose(compose(correction, history[-1][1]), [1.9, .1, 0.])
    assert interpolate(history, 31) is None


def test_icp_current_to_reference_and_reject_empty():
    rng = np.random.default_rng(42)
    reference = rng.uniform(-2., 2., (500, 3))
    reference[:, 2] = 0.
    expected = np.array([.12, -.07, .08])
    current = transform_points(reference, inverse(expected))
    icp = ICP()
    np.testing.assert_allclose(icp.align(current, reference, np.zeros(3)), expected, atol=1e-5)
    assert icp.align([], reference, np.zeros(3)) is None
    assert icp.align(current+100., reference, np.zeros(3)) is None


@pytest.mark.parametrize('goal', [[1., .2, .4], [-.8, -.1, .2], [0., 0., 1.5]])
def test_closed_loop_polar_convergence(goal):
    pose = np.zeros(3)
    for _ in range(3000):
        v, w = polar_command(between(pose, goal))
        pose = compose(pose, wheel_step((v-w*.08)*.02/.033,
                                        (v+w*.08)*.02/.033, .033, .16))
    err = between(pose, goal)
    assert np.linalg.norm(err[:2]) <= .036
    assert abs(err[2]) <= .061


def test_atomic_route_roundtrip_and_validation(tmp_path):
    path = tmp_path/'route.json'
    frames = [dict(pose=[0., 0., 0.], points=[])]
    save_route(path, frames, 'encoders', [])
    assert load_route(path)['keyframes'] == frames
    with pytest.raises(ValueError, match='sin escaneo'):
        load_route(path, require_scans=True)
    with pytest.raises(ValueError):
        save_route(path, [dict(pose=[float('nan'), 0, 0])], 'encoders', [])
    assert load_route(path)['keyframes'] == frames
