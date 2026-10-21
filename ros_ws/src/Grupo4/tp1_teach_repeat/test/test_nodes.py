"""Regresiones de arranque, TF y watchdog con mensajes ROS reales."""
import math
import time
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState, LaserScan
from tp1_teach_repeat.core import compose
from tp1_teach_repeat.controller import Controller
from tp1_teach_repeat.localization import Localization, ns, yaw


@pytest.fixture(scope='module', autouse=True)
def context():
    rclpy.init()
    yield
    rclpy.shutdown()


def parameters(**kw):
    return [Parameter(k, value=v) for k, v in kw.items()]


def joint(t, left=0., right=0.):
    m = JointState()
    m.header.stamp = Time(sec=t)
    # Orden invertido para detectar dependencia accidental de índices.
    m.name, m.position = ['wheel_right_joint', 'wheel_left_joint'], [right, left]
    return m


def test_mixed_late_extrinsic_fills_first_keyframe(tmp_path):
    node = Localization(parameter_overrides=parameters(
        mode='mixed', route_file=str(tmp_path/'route.json')))
    try:
        node.on_joint(joint(1))
        node.on_joint(joint(2))
        assert len(node.frames) == 1 and node.frames[0]['points'] == []
        # Extrínseca llega después del primer keyframe.
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_link', 'base_scan'
        tf.transform.rotation.w = 1.
        tf.transform.translation.x = -.032
        node.buffer.set_transform_static(tf, 'test')
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = Time(sec=2), 'base_scan'
        scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/360
        scan.range_min, scan.range_max = .1, 5.
        scan.ranges = [2.] * 360
        node.on_scan(scan)
        assert len(node.frames[0]['points']) == 360
        assert node.frames[0]['scan']['stamp_ns'] == 2*10**9
        assert node.frames[0]['stamp_ns'] == 2*10**9
        np.testing.assert_allclose(node.pose, 0., atol=1e-12)
        # Sin más LiDAR, mixto continúa integrando las ruedas.
        node.on_joint(joint(3, 1., 1.))
        assert node.pose[0] == pytest.approx(.033)
    finally:
        node.destroy_node()


def test_repeat_tf_composition_and_measurement_stamp(tmp_path):
    from tp1_teach_repeat.core import save_route
    path = tmp_path/'route.json'
    frames = [dict(pose=[0., 0., 0.], points=[]), dict(pose=[1., .5, .4], points=[])]
    save_route(path, frames, 'encoders', [])
    node = Localization(parameter_overrides=parameters(mode='encoders', stage='repeat', route_file=str(path)))
    try:
        node.broadcaster.sendTransform = Mock()
        pose = np.array([.2, -.1, .15])
        node.publish(pose, Time(sec=10), (0., 0.))
        transforms = node.broadcaster.sendTransform.call_args.args[0]
        assert len(transforms) == 5
        assert all(ns(t.header.stamp) == 10*10**9 for t in transforms)
        tf = {t.child_frame_id: np.array([t.transform.translation.x, t.transform.translation.y,
                                         yaw(t.transform.rotation)]) for t in transforms}
        np.testing.assert_allclose(compose(pose, tf['map_current_keyframe']), frames[0]['pose'], atol=1e-12)
        np.testing.assert_allclose(compose(pose, tf['setpoint']), frames[1]['pose'], atol=1e-12)
        node.publish(pose, Time(sec=9), (0., 0.))
        assert node.broadcaster.sendTransform.call_count == 1
    finally:
        node.destroy_node()


def test_publish_rate_keeps_all_encoder_motion(tmp_path):
    node = Localization(parameter_overrides=parameters(mode='encoders', route_file=str(tmp_path/'route.json')))
    try:
        node.broadcaster.sendTransform = Mock()
        node.on_joint(joint(1))
        early = joint(1, .5, .5)
        early.header.stamp.nanosec = 5_000_000
        node.on_joint(early)
        assert node.broadcaster.sendTransform.call_count == 1
        final = joint(1, 1., 1.)
        final.header.stamp.nanosec = 20_000_000
        node.on_joint(final)
        assert node.broadcaster.sendTransform.call_count == 2
        assert node.pose[0] == pytest.approx(.033)
        assert ns(node.last_stamp) == 1_020_000_000
    finally:
        node.destroy_node()


@pytest.mark.parametrize('anchors,expected_x', [(0, .04), (3, 0.)])
def test_teach_submap_removes_scan_to_scan_drift(tmp_path, anchors, expected_x):
    node = Localization(parameter_overrides=parameters(
        mode='mixed', teach_submap_keyframes=anchors, route_file=str(tmp_path/'route.json')))
    try:
        points = np.random.default_rng(7).uniform(-2, 2, (360, 3))
        points[:, 2] = 0.
        node.on_joint(joint(1))
        node.on_joint(joint(3))
        node.frames = [dict(pose=[0., 0., 0.], points=points.tolist())]
        node.prev_cloud, node.prev_scan_pose = points, np.array([.04, 0., 0.])
        node.last_scan_ns = 2*10**9
        node.cloud = lambda msg: points.copy()
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = Time(sec=3), 'base_scan'
        scan.range_min, scan.range_max, scan.ranges = .1, 5., [2.] * 360
        node.on_scan(scan)
        assert node.correction[0] == pytest.approx(expected_x, abs=1e-6)
    finally:
        node.destroy_node()


@pytest.mark.parametrize('stamped', [True, False])
def test_controller_stale_and_wrong_frame_stop(stamped):
    node = Controller(parameter_overrides=parameters(publish_stamped=stamped))
    try:
        node.pub = Mock()
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp = node.get_clock().now().to_msg()
        target.pose.position.x, target.pose.orientation.w = 1., 1.
        node.on_target(target)
        node.tick()
        msg = node.pub.publish.call_args.args[0]
        assert (msg.twist if stamped else msg).linear.x > 0.
        # El watchdog de pared vence incluso si no cambió el reloj ROS.
        node.received = time.monotonic()-1.
        node.tick()
        msg = node.pub.publish.call_args.args[0]
        assert (msg.twist if stamped else msg).linear.x == 0.
        target.header.frame_id = 'odom'
        node.on_target(target)
        node.tick()
        msg = node.pub.publish.call_args.args[0]
        assert (msg.twist if stamped else msg).linear.x == 0.
    finally:
        node.destroy_node()
