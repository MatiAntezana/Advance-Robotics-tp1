"""Marcos y estados de los indicadores de navegación y residual medido."""
import numpy as np
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from tp1_teach_repeat import visualization as v
from tp1_teach_repeat.core import alignment_quality
from tp1_teach_repeat.localization import cloud_message


def test_keyframe_labels_follow_active_and_next_and_final():
    frames = [dict(pose=[float(i), 0., 0.]) for i in range(4)]
    markers = v.keyframes(frames, 'repeat', 1, False, Time(sec=4)).markers
    labels = [m.text for m in markers if m.type == Marker.TEXT_VIEW_FACING]
    assert labels == ['KF0', 'KF1\nACTIVO', 'KF2\nSIGUIENTE', 'KF3']
    final = v.keyframes(frames, 'repeat', 2, True, Time(sec=4)).markers
    assert any(m.text == 'KF3\nFINAL' for m in final)
    assert not any('SIGUIENTE' in m.text for m in final)


def test_setpoint_line_is_robot_to_target_in_odom_and_stale_quality():
    markers = v.navigation([1., 2., .4], [3., 4., .5], 2, False, Time(sec=7),
                           dict(rmse_m=.01, overlap=.99, stale=True)).markers
    line = next(m for m in markers if m.type == Marker.LINE_STRIP)
    assert [(p.x, p.y) for p in line.points] == [(1., 2.), (3., 4.)]
    assert all(m.header.frame_id == 'odom' and m.header.stamp.sec == 7 for m in markers)
    assert any(m.text == 'RMSE mapa:--\nSIN_SCAN' for m in markers)


def test_alignment_quality_reports_disagreement_without_moving_data():
    points = np.column_stack((np.arange(30)*.1, np.zeros(30), np.zeros(30)))
    unchanged = points.copy()
    good = alignment_quality(points, points)
    shifted = alignment_quality(points, points, (0., .08, 0.))
    assert good['rmse_m'] == 0.
    assert abs(shifted['rmse_m']-.08) < 1e-8
    assert shifted['overlap'] == 1.
    assert alignment_quality(points, points, (0., 2., 0.))['overlap'] == 0.
    np.testing.assert_array_equal(points, unchanged)


def test_localized_scan_keeps_measurement_stamp_and_metric_coordinates():
    stamp = Time(sec=11, nanosec=123)
    cloud = cloud_message([[1., 2., 0.], [-.5, .7, 0.]], stamp)
    assert cloud.header.frame_id == 'odom' and cloud.header.stamp == stamp
    assert cloud.width == 2 and cloud.row_step == 24
    np.testing.assert_allclose(np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 3),
                               [[1., 2., 0.], [-.5, .7, 0.]], atol=1e-7)
