import importlib.util
import math
from pathlib import Path

import numpy as np
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from geometry_msgs.msg import Pose


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


anchor = load_script('aruco_map_anchor')
adapter = load_script('aruco_detection_adapter')
calibrator = load_script('aruco_map_calibrator')
cmd_gate = load_script('erc_cmd_vel_gate')


def transform(yaw, xy):
    c, s = math.cos(yaw), math.sin(yaw)
    result = np.eye(3)
    result[:2, :2] = [[c, -s], [s, c]]
    result[:2, 2] = xy
    return result


def test_post_freeze_composition_recovers_map_camera_init():
    points_datum = np.array([[0.0, 0.0], [2.0, 0.0], [0.0, 3.0]])
    map_datum = transform(math.radians(30.0), [0.2, -0.1])
    expected_map_ci = transform(math.radians(10.0), [1.0, 2.0])

    homogeneous = np.c_[points_datum, np.ones(len(points_datum))].T
    points_ci = (
        np.linalg.inv(expected_map_ci) @ map_datum @ homogeneous).T[:, :2]
    cd_tx, cd_ty, cd_yaw, residual = anchor.fit_se2(
        points_datum, points_ci)
    actual = anchor.map_camera_init_from_datum_fit(
        (0.2, -0.1, math.radians(30.0)),
        (cd_tx, cd_ty, cd_yaw))

    assert residual < 1e-9
    assert np.allclose(actual[:2], [1.0, 2.0], atol=1e-9)
    assert math.isclose(actual[2], math.radians(10.0), abs_tol=1e-9)


def test_map_coordinate_landmarks_need_no_datum_transform():
    expected_map_ci = transform(math.radians(-15.0), [2.0, -1.0])
    map_points = np.array([[0.0, 0.0], [3.0, 0.5], [-1.0, 2.0]])
    homogeneous = np.c_[map_points, np.ones(len(map_points))].T
    points_ci = (np.linalg.inv(expected_map_ci) @ homogeneous).T[:, :2]

    ci_map = anchor.fit_se2(map_points, points_ci)
    actual = anchor.map_camera_init_from_datum_fit(
        (0.0, 0.0, 0.0), ci_map[:3])

    assert ci_map[3] < 1e-9
    assert np.allclose(actual[:2], [2.0, -1.0], atol=1e-9)
    assert math.isclose(actual[2], math.radians(-15.0), abs_tol=1e-9)


def test_calibrator_robust_position_rejects_large_outlier():
    samples = [
        [1.00, 2.00, 0.90],
        [1.01, 1.99, 0.91],
        [0.99, 2.01, 0.89],
        [8.00, -5.00, 3.00],
    ]

    position, kept, _, max_residual = calibrator.robust_position(
        samples, absolute_gate=0.05)

    assert kept == 3
    assert np.allclose(position, [1.0, 2.0, 0.9], atol=1e-9)
    assert max_residual < 0.02


def test_legacy_translation_is_restored_to_optical_axes():
    # Shared aruco_opencv maps optical tvec=(1,2,3) to legacy=(3,-1,-2).
    legacy = Pose()
    legacy.position.x = 3.0
    legacy.position.y = -1.0
    legacy.position.z = -2.0
    legacy.orientation.w = 1.0

    corrected = adapter.corrected_optical_pose(legacy)

    assert corrected.position.x == 1.0
    assert corrected.position.y == 2.0
    assert corrected.position.z == 3.0
    assert corrected.orientation.w == 1.0


def test_adapter_rejects_zero_quaternion_and_nonfinite_position():
    pose = Pose()
    pose.orientation.w = 0.0
    assert not adapter.pose_is_finite(pose)
    pose.orientation.w = 1.0
    assert adapter.pose_is_finite(pose)
    pose.position.x = float('nan')
    assert not adapter.pose_is_finite(pose)


def test_adapter_preserves_all_simultaneous_marker_ids():
    raw = ArucoDetection()
    raw.header.frame_id = 'camera_color_optical_frame'
    for mid, legacy_xyz in ((51, (3.0, -1.0, -2.0)),
                            (52, (6.0, -4.0, -5.0))):
        marker = MarkerPose()
        marker.marker_id = mid
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = legacy_xyz
        marker.pose.orientation.w = 1.0
        raw.markers.append(marker)

    corrected, dropped = adapter.correct_detection(raw)

    assert dropped == []
    assert [m.marker_id for m in corrected.markers] == [51, 52]
    assert corrected.header.frame_id == 'camera_color_optical_frame'
    assert [(m.pose.position.x, m.pose.position.y, m.pose.position.z)
            for m in corrected.markers] == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]


def test_pairwise_3d_distance_check_is_rigid_transform_invariant():
    known = np.array([[0.0, 0.0, 0.0], [2.0, 1.0, 0.5], [-1.0, 3.0, 1.0]])
    yaw = math.radians(25.0)
    rotation = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    observed = known @ rotation.T + np.array([4.0, -2.0, 0.7])

    absolute, relative = anchor.pairwise_distance_consistency(known, observed)

    assert absolute < 1e-12
    assert relative < 1e-12


def test_pairwise_3d_distance_check_detects_wrong_landmark_scale():
    known = np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    observed = np.array([[0.0, 0.0, 0.0], [3.4, 0.0, 0.0]])

    absolute, relative = anchor.pairwise_distance_consistency(known, observed)

    assert math.isclose(absolute, 0.4)
    assert math.isclose(relative, 0.4 / 3.0)


def test_alternating_single_marker_frames_are_not_merged():
    def detection(capture_ns, marker_id):
        point = np.zeros(3)
        return anchor.Detection(
            0.0, capture_ns, marker_id, point, point, 1.0, 90.0, point,
            np.zeros(2), 0.0)

    buffer = [
        detection(100, 51),
        detection(200, 52),
    ]
    latest = anchor.latest_capture_batch(buffer)

    assert [entry.mid for entry in latest] == [52]


def test_two_markers_from_same_frame_are_preserved():
    def detection(marker_id):
        point = np.zeros(3)
        return anchor.Detection(
            0.0, 300, marker_id, point, point, 1.0, 90.0, point,
            np.zeros(2), 0.0)

    latest = anchor.latest_capture_batch([detection(51), detection(52)])

    assert sorted(entry.mid for entry in latest) == [51, 52]


def test_single_marker_recovers_translation_when_yaw_is_already_known():
    expected = transform(math.radians(35.0), [-0.4, 1.2])
    known_map = np.array([4.0, -1.0, 0.8])
    observed_ci = (
        np.linalg.inv(expected) @ np.array([known_map[0], known_map[1], 1.0])
    )[:2]

    tx, ty = anchor.single_marker_translation_candidate(
        known_map, observed_ci, (0.0, 0.0, 0.0), math.radians(35.0))

    assert np.allclose([tx, ty], [-0.4, 1.2], atol=1e-9)


def test_single_marker_supports_legacy_datum_coordinate_landmarks():
    map_datum = transform(math.radians(20.0), [0.3, -0.2])
    expected_map_ci = transform(math.radians(-10.0), [1.0, 2.0])
    known_datum = np.array([3.0, 0.5, 1.0])
    known_map_h = map_datum @ np.array([known_datum[0], known_datum[1], 1.0])
    observed_ci = (np.linalg.inv(expected_map_ci) @ known_map_h)[:2]

    tx, ty = anchor.single_marker_translation_candidate(
        known_datum, observed_ci,
        (0.3, -0.2, math.radians(20.0)), math.radians(-10.0))

    assert np.allclose([tx, ty], [1.0, 2.0], atol=1e-9)


def test_translation_consensus_rejects_outlier_and_keeps_three_inliers():
    result = anchor.translation_consensus(
        [(1.00, 2.00), (1.03, 1.98), (0.98, 2.01), (3.0, -4.0)],
        max_spread=0.10, min_count=3)

    assert result is not None
    tx, ty, spread, count = result
    assert count == 3
    assert np.allclose([tx, ty], [1.0, 2.0], atol=0.02)
    assert spread < 0.05


def test_translation_consensus_requires_minimum_consistent_samples():
    result = anchor.translation_consensus(
        [(1.0, 2.0), (1.02, 2.01)],
        max_spread=0.10, min_count=3)

    assert result is None


def test_cmd_vel_gate_is_fail_closed_and_requires_fresh_localization():
    common = dict(
        emergency_stop=False, initialized=True, fitness=0.04,
        fitness_age=0.2, fitness_max=0.15, fitness_timeout=3.0,
        command_age=0.1, command_timeout=0.3, require_fitness=True)
    assert cmd_gate.gate_reason(**common) == 'open'

    for changed, expected in (
            ({'emergency_stop': True}, 'emergency_stop'),
            ({'initialized': False}, 'localization_not_initialized'),
            ({'fitness': None}, 'fitness_missing'),
            ({'fitness_age': 3.1}, 'fitness_stale'),
            ({'fitness': 0.2}, 'fitness_bad'),
            ({'command_age': 0.31}, 'command_timeout')):
        case = dict(common)
        case.update(changed)
        assert cmd_gate.gate_reason(**case) == expected
