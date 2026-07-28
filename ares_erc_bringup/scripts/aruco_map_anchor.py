#!/usr/bin/env python3
"""ERC Phase 3 — ArUco global anchor: correct map->datum, then map->camera_init.

Phase 1/2 gave a live pose on the prior map (`map`), but the tie between `map` and the
ERC ground-truth `datum` frame was only a hand-guessed yaw (waypoints_erc.yaml). This
node measures it: it watches ArUco detections whose datum coordinates are known, and
corrects the  map -> datum  transform so the markers land where the ERC table says
they are.

Frame authority (single, no doubling — CLAUDE.md §6.2):
  map -> camera_init : map_anchor (GICP + this node's ArUco candidate, once frozen)
  camera_init -> body: FAST-LIO                                    [untouched]
  map -> datum       : THIS node (ArUco ground-truth alignment)    [was erc_waypoints]
Run erc_waypoints with publish_datum_tf:=false so only this node owns map->datum.

Two-phase design (anti-degeneracy rationale):
  map->datum is physically a FIXED quantity (how our LiDAR happened to be oriented at
  the start, vs. the ERC coordinate axes) -- it does not change as the robot drives.
  map->camera_init is a TIME-VARYING quantity: map_anchor's GICP correction for
  accumulated FAST-LIO/scan-matching drift, which can grow badly in geometrically
  degenerate terrain (flat, featureless stretches) where LiDAR alone can't observe
  motion in some direction. A single marker sighting can't tell which of these two
  unknowns it's disagreeing with -- but at the very start (Rev.3: >=2 landmarks
  guaranteed visible from the start point), the robot hasn't moved yet, so any
  discrepancy MUST be datum error (drift hasn't had time to accumulate). That lets us
  solve datum with confidence early, FREEZE it, and then unambiguously attribute all
  LATER marker disagreement to camera_init drift instead -- publishing it as an
  independent candidate correction for map_anchor.cpp to fuse alongside its own GICP,
  precisely useful when GICP itself is degenerate and can't self-correct.

Method (§6.4 approach A — one slow rigid transform per phase, monocular safety valves):
  * per detection of a known id, transform the marker position into `map` AND into
    `camera_init` at the detection's OWN timestamp (not "latest TF", which would bias
    the estimate by velocity*pipeline-latency while moving; falls back to latest TF if
    the exact-time sample has aged out of the buffer),
  * accumulate {datum coord <-> observed map position <-> observed camera_init position
    <-> range <-> viewing angle} over a short window,
  * PRE-FREEZE phase (fit map->datum): >=3 markers, or 2 markers with a datum-frame
    separation >= min_baseline_2pt -> full SE(2) Procrustes (yaw + xy), each marker
    weighted ~1/range^2 (closer = more trusted); 2 markers with a short (near-collinear-
    from-here) baseline -> fall back to the single-marker path on the closer of the two
    (a 2-point Procrustes yaw is fragile there); 1 marker -> yaw about the known origin
    only when close (max_range) AND roughly frontal (min_view_angle_deg off grazing --
    planar-pose ambiguity), translation stays at the seed. Freezes after
    freeze.min_multimarker_updates accepted >=2-marker fits, or freeze.max_wait_sec
    regardless.
  * POST-FREEZE phase (fit map->camera_init candidate): >=2 markers on a trustworthy
    baseline produce a full planar x/y/yaw candidate. Once map localization has already
    been initialized, one known marker may produce an x/y-only candidate while yaw stays
    constrained by FAST-LIO/GICP. Several same-ID candidates must agree before one is
    published; a one-marker candidate is never allowed to initialize global yaw.
    Candidates are PoseWithCovarianceStamped messages (not TF -- map_anchor owns that
    edge and fuses them alongside its own GICP correction).
  * low-pass the accepted map->datum correction (never snap), gated by known-id-only /
    max-range / min-view-angle / max-jump / residual. Publishes /erc/aruco_anchor_residual,
    /erc/aruco_camera_init_candidate (post-freeze), and RViz viz markers.
"""
import math
import os
from collections import defaultdict, deque, namedtuple

import numpy as np
import rclpy
import yaml
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, \
    ExtrapolationException
from visualization_msgs.msg import Marker, MarkerArray

Detection = namedtuple(
    'Detection',
    'stamp capture_ns mid p_datum p_map rng view_deg p_camera_init '
    'map_camera_init_xy map_camera_init_yaw')

# PoseWithCovarianceStamped is retained for compatibility with the existing C++ fusion
# node. covariance[35] >= this value means "translation-only; yaw is unconstrained".
# Full multi-marker candidates leave covariance[35] at zero.
TRANSLATION_ONLY_YAW_VARIANCE = 1.0e6


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_rotmat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def fit_se2(src, dst, weights=None):
    """Weighted least-squares R(yaw), t with dst_i ~= R@src_i + t (weighted Kabsch/Procrustes,
    no scale). weights=None -> uniform (reduces to the plain unweighted fit).

    src, dst: (N,2). weights: (N,) or None. Returns (tx, ty, yaw, weighted_rms_residual).
    """
    w = np.ones(len(src)) if weights is None else np.asarray(weights, dtype=float)
    w = w / w.sum()
    cs = (w[:, None] * src).sum(axis=0)
    cd = (w[:, None] * dst).sum(axis=0)
    H = (src - cs).T @ (w[:, None] * (dst - cd))     # weighted 2x2
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T     # rotates src -> dst
    t = cd - R @ cs
    res = math.sqrt(float((w * np.sum((src @ R.T + t - dst) ** 2, axis=1)).sum()))
    return float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0]), res


def pairwise_distance_consistency(src, dst):
    """Compare all pair distances before fitting a rigid transform.

    Distances are invariant to the unknown camera/robot pose and to a rigid
    camera-LiDAR extrinsic. A mismatch therefore catches wrong landmark coordinates,
    marker size, or an unmodelled marker-face-to-pole offset before it can corrupt the
    global pose. Returns (maximum absolute error [m], maximum relative error).
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    max_abs = 0.0
    max_rel = 0.0
    for i in range(len(src)):
        for j in range(i + 1, len(src)):
            known = float(np.linalg.norm(src[i] - src[j]))
            observed = float(np.linalg.norm(dst[i] - dst[j]))
            error = abs(observed - known)
            max_abs = max(max_abs, error)
            if known > 1e-6:
                max_rel = max(max_rel, error / known)
    return max_abs, max_rel


def latest_capture_batch(buffer):
    """Return one latest observation per ID, all from the exact same image."""
    if not buffer:
        return []
    latest_capture = max(entry.capture_ns for entry in buffer)
    latest = {}
    for entry in buffer:
        if entry.capture_ns == latest_capture:
            latest[entry.mid] = entry
    return list(latest.values())


def map_camera_init_from_datum_fit(map_datum, camera_init_datum):
    """Compose T_map_camera_init = T_map_datum * inverse(T_camera_init_datum).

    Each tuple is ``(tx, ty, yaw)`` and follows the ROS TF convention: it maps a
    point expressed in the child frame into the parent frame.
    """
    md_tx, md_ty, md_yaw = map_datum
    cd_tx, cd_ty, cd_yaw = camera_init_datum
    map_ci_yaw = wrap(md_yaw - cd_yaw)
    R_map_ci = np.array([
        [math.cos(map_ci_yaw), -math.sin(map_ci_yaw)],
        [math.sin(map_ci_yaw), math.cos(map_ci_yaw)],
    ])
    # T_md * inv(T_cd): t_mci = t_md - R_mci * t_cd
    map_ci_t = np.array([md_tx, md_ty]) - R_map_ci @ np.array([cd_tx, cd_ty])
    return float(map_ci_t[0]), float(map_ci_t[1]), map_ci_yaw


def transform_planar_point(transform, point):
    """Apply planar (tx, ty, yaw) to the xy components of a point."""
    tx, ty, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = float(point[0]), float(point[1])
    return np.array([tx + c * x - s * y, ty + s * x + c * y])


def single_marker_translation_candidate(
        known_coordinate_point, observed_camera_init_point,
        map_to_coordinate, current_map_camera_init_yaw):
    """Compute map->camera_init xy from one landmark while holding yaw fixed.

    ``map_to_coordinate`` is identity for map-coordinate anchors, or the frozen
    map->datum transform for legacy datum-coordinate anchors.
    """
    known_map_xy = transform_planar_point(
        map_to_coordinate, known_coordinate_point)
    rotated_observation = transform_planar_point(
        (0.0, 0.0, current_map_camera_init_yaw),
        observed_camera_init_point)
    candidate = known_map_xy - rotated_observation
    return float(candidate[0]), float(candidate[1])


def translation_consensus(points, max_spread, min_count):
    """Return robust median xy and max inlier spread, or None if not consistent."""
    if len(points) < min_count:
        return None
    values = np.asarray(points, dtype=float)
    center = np.median(values, axis=0)
    distance = np.linalg.norm(values - center, axis=1)
    inliers = values[distance <= max_spread]
    if len(inliers) < min_count:
        return None
    center = np.median(inliers, axis=0)
    spread = float(np.max(np.linalg.norm(inliers - center, axis=1)))
    return float(center[0]), float(center[1]), spread, len(inliers)


class ArucoMapAnchor(Node):
    def __init__(self):
        super().__init__('aruco_map_anchor')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.datum_frame = self.declare_parameter('datum_frame', 'datum').value
        self.odom_frame = self.declare_parameter('odom_frame', 'camera_init').value
        det_topic = self.declare_parameter('detections_topic', '/aruco_detections').value
        anchors_file = self.declare_parameter('anchors_file', '').value

        if not anchors_file or not os.path.isfile(anchors_file):
            raise RuntimeError(f'anchors_file not found: {anchors_file!r}')
        with open(anchors_file) as f:
            cfg = yaml.safe_load(f) or {}

        d = cfg.get('datum', {})
        # Localization landmarks should normally be surveyed directly in the prior
        # map. Legacy datum-coordinate tables remain supported when map->datum is
        # independently calibrated.
        self.coordinate_frame = str(
            cfg.get('coordinate_frame', self.datum_frame)).strip()
        if self.coordinate_frame not in (self.map_frame, self.datum_frame):
            raise RuntimeError(
                f'coordinate_frame must be {self.map_frame!r} or '
                f'{self.datum_frame!r}, got {self.coordinate_frame!r}')
        self.seed_xy = np.array(d.get('xy_offset', [0.0, 0.0]), dtype=float)
        seed_yaw = math.radians(float(d.get('yaw_offset_deg', 0.0)))
        # A calibrated map->datum is required for arbitrary-start global localization.
        # If false, the legacy same-start calibration phase remains available.
        self.datum_calibrated = bool(d.get('calibrated', False))
        # known markers: id -> coordinate_frame xyz (correction is planar SE(2))
        self.known = {}
        for m in (cfg.get('markers', []) or []):
            self.known[int(m['id'])] = np.array(
                [float(m.get('x', 0.0)), float(m.get('y', 0.0)), float(m.get('z', 0.0))])
        if not self.known:
            self.get_logger().warn('no known markers in anchors_file — anchor will idle')

        g = cfg.get('gates', {})
        self.max_range = float(g.get('max_range', 4.0))
        self.max_jump_trans = float(g.get('max_jump_trans', 1.0))
        self.max_jump_yaw = float(g.get('max_jump_yaw', 0.5))
        self.residual_max = float(g.get('residual_max', 1.0))
        # A rigid transform cannot change the 3-D distance between landmarks. This
        # catches bad hand-measured YAML coordinates / wrong marker scale before the
        # lower-dimensional SE(2) fit can look deceptively plausible.
        self.max_pair_distance_error = float(
            g.get('max_pair_distance_error', 0.20))
        self.max_pair_distance_relative_error = float(
            g.get('max_pair_distance_relative_error', 0.10))
        # planar-pose ambiguity floor (TagSLAM-style grazing-angle gate): degrees off
        # grazing incidence, 90=dead-on frontal, 0=edge-on. Below this, a lone marker's
        # yaw is untrustworthy even if close.
        self.min_view_angle_deg = float(g.get('min_view_angle_deg', 20.0))
        # 2-marker datum-frame separation [m] below which the Procrustes yaw fit is
        # numerically fragile (near-collinear from the camera's perspective) -> fall
        # back to the single-marker path (datum phase) or skip (camera_init phase).
        self.min_baseline_2pt = float(g.get('min_baseline_2pt', 1.5))
        landmark = cfg.get('landmark', {})
        # ERC coordinates refer to the pole axis, while solvePnP observes a box face.
        # Zero is correct for flat indoor test tags. Set to half the box width for the
        # real four-faced landmark after confirming its pole/box geometry.
        self.face_to_pole_depth = float(
            landmark.get('face_to_pole_depth', 0.0))
        u = cfg.get('update', {})
        self.accum_window = float(u.get('accum_window_sec', 2.0))
        self.alpha = float(u.get('lowpass_alpha', 0.2))
        update_period = float(u.get('update_period', 0.5))
        tf_rate = float(u.get('tf_publish_rate', 20.0))
        # Camera images often arrive a few tens of milliseconds ahead of the latest
        # dynamic TF. Queue them briefly instead of using "latest TF", which creates a
        # velocity * latency pose bias while the rover is moving.
        self.max_tf_wait = float(u.get('max_tf_wait_sec', 0.25))
        self.yaw_range = float(u.get('yaw_update_max_range', 1.5))
        self.yaw_weight = float(u.get('single_marker_yaw_weight', 0.3))
        self.single_tracking_enabled = bool(
            u.get('single_marker_tracking_enabled', True))
        self.single_min_consistent = int(
            u.get('single_marker_min_consistent', 3))
        self.single_consistency_trans = float(
            u.get('single_marker_consistency_trans', 0.15))
        self.single_history_sec = float(
            u.get('single_marker_history_sec', 2.0))

        # Freeze criteria: once map->datum is trusted, stop updating it and start
        # publishing map->camera_init candidates from later marker sightings instead
        # (see module docstring for the anti-degeneracy rationale).
        fz = cfg.get('freeze', {})
        self.freeze_min_multimarker = int(fz.get('min_multimarker_updates', 3))
        self.freeze_max_wait_sec = float(fz.get('max_wait_sec', 20.0))
        # Map-coordinate anchors need no online map->datum calibration phase.
        self.frozen = self.datum_calibrated or self.coordinate_frame == self.map_frame
        self.multimarker_count = 0
        self.start_time_sec = self.get_clock().now().nanoseconds * 1e-9

        # current map->datum estimate (SE(2): translation xy + yaw)
        self.tx, self.ty = float(self.seed_xy[0]), float(self.seed_xy[1])
        self.yaw = seed_yaw

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        from tf2_ros import TransformBroadcaster
        self.bc = TransformBroadcaster(self)

        self.buffer = deque()  # rolling window of Detection tuples
        self.pending = deque()  # (arrival monotonic ROS seconds, ArucoDetection)
        self.single_candidate_history = defaultdict(deque)
        self.localization_initialized = False

        self.residual_pub = self.create_publisher(Float32, '/erc/aruco_anchor_residual', 10)
        self.candidate_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/erc/aruco_camera_init_candidate', 10)
        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.known_pub = self.create_publisher(MarkerArray, '/erc/aruco_known_markers', latched)
        self.obs_pub = self.create_publisher(MarkerArray, '/erc/aruco_observed', 10)
        self._publish_known_markers()

        self.sub = self.create_subscription(ArucoDetection, det_topic, self.det_cb, 10)
        self.create_subscription(
            Bool, '/erc/localization_initialized',
            self._localization_initialized_cb, latched)
        self.create_timer(0.02, self._process_pending)
        self.create_timer(1.0 / max(1.0, tf_rate), self.broadcast_tf)
        self.create_timer(update_period, self.update)
        self.get_logger().info(
            f'aruco_map_anchor up: {len(self.known)} known markers in '
            f'{self.coordinate_frame} on {det_topic}; map->datum seed yaw='
            f'{math.degrees(self.yaw):.1f} deg. '
            + (f'Datum is pre-calibrated; estimating {self.map_frame}->{self.odom_frame} '
               'global pose immediately.'
               if self.frozen else
               f'Will freeze after {self.freeze_min_multimarker} multi-marker fit(s) or '
               f'{self.freeze_max_wait_sec:.0f}s, then estimate {self.map_frame}->'
               f'{self.odom_frame} corrections instead.'))

    def _localization_initialized_cb(self, msg):
        self.localization_initialized = bool(msg.data)
        if not self.localization_initialized:
            self.single_candidate_history.clear()

    # ---- detections -> correspondences (in both map and camera_init frames) ----
    def _lookup_exact(self, target_frame, source_frame, detect_time):
        """Return the timestamped transform, or None until TF catches up."""
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, detect_time)
            return self._tf_to_Rt(tf)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def det_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.pending.append((now, msg))
        # Bound memory even if TF never becomes connected.
        while len(self.pending) > 30:
            self.pending.popleft()

    def _process_pending(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.pending:
            arrival, msg = self.pending[0]
            if self._process_detection(msg):
                self.pending.popleft()
                continue
            age = now - arrival
            if age <= self.max_tf_wait:
                break
            self.pending.popleft()
            self.get_logger().warn(
                f'dropping ArUco frame after waiting {age:.3f}s for timestamped TF; '
                'latest-TF fallback is disabled to avoid motion bias',
                throttle_duration_sec=3.0)

    def _process_detection(self, msg):
        frame = msg.header.frame_id
        # Look up TF at the detection's OWN capture time, not "latest" -- while moving,
        # latest-TF silently biases the estimate by velocity * pipeline latency.
        detect_time = rclpy.time.Time.from_msg(msg.header.stamp)
        rt_map = self._lookup_exact(self.map_frame, frame, detect_time)
        if rt_map is None:
            return False
        R_map, t_map = rt_map
        # camera_init<-frame is a STRICT SUBSET of the map<-frame chain (skips
        # map_anchor's own map->camera_init hop) -- needed so the post-freeze
        # camera_init candidate doesn't circularly depend on map_anchor's current
        # (possibly-degenerate) estimate of that very transform.
        rt_ci = self._lookup_exact(self.odom_frame, frame, detect_time)
        if rt_ci is None:
            return False
        R_ci, t_ci = rt_ci
        # T_map_ci = T_map_frame * inverse(T_ci_frame). This current yaw is the
        # independent FAST-LIO/GICP yaw constraint used by the one-marker translation
        # path; the marker itself is not trusted to define yaw.
        R_map_ci = R_map @ R_ci.T
        t_map_ci = t_map - R_map_ci @ t_ci
        map_ci_yaw = math.atan2(R_map_ci[1, 0], R_map_ci[0, 0])
        now = self.get_clock().now().nanoseconds * 1e-9
        for mk in msg.markers:
            mid = int(mk.marker_id)
            if mid not in self.known:
                continue
            p_face_cam = np.array(
                [mk.pose.position.x, mk.pose.position.y, mk.pose.position.z])
            rng = float(np.linalg.norm(p_face_cam))     # camera->marker-face distance
            if rng > self.max_range:
                continue
            # Grazing-angle (planar-pose ambiguity) estimate: angle between the camera
            # ray and the marker plane, from the marker's own orientation. 90=dead-on
            # frontal (best), 0=edge-on (worst, ambiguous like TagSLAM's rpp check).
            # Rev.3 confirms each landmark shows an identical marker on all 4 faces, so
            # this is only ever used as a quality gate, never as an absolute heading.
            q = (mk.pose.orientation.x, mk.pose.orientation.y,
                 mk.pose.orientation.z, mk.pose.orientation.w)
            marker_normal_cam = quat_to_rotmat(q)[:, 2]
            ray = p_face_cam / rng if rng > 1e-6 else np.zeros(3)
            cos_theta = abs(float(np.dot(marker_normal_cam, ray)))
            view_deg = 90.0 - math.degrees(math.acos(min(1.0, max(0.0, cos_theta))))
            # Rules define the landmark coordinate on the pole axis, not on the visible
            # face. Marker +z points outward toward a front-facing camera, so the pole is
            # behind the face along -z.
            p_pole_cam = p_face_cam - self.face_to_pole_depth * marker_normal_cam
            p_obs = R_map @ p_pole_cam + t_map             # pole position in map
            p_ci = R_ci @ p_pole_cam + t_ci                # pole position in camera_init
            capture_ns = (
                int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec))
            self.buffer.append(Detection(
                now, capture_ns, mid, self.known[mid], p_obs, rng, view_deg, p_ci,
                t_map_ci[:2], map_ci_yaw))
            self.get_logger().info(
                f'detected known marker id{mid} at {rng:.2f} m, {view_deg:.0f} deg off-grazing '
                f'-> map({p_obs[0]:.2f}, {p_obs[1]:.2f})', throttle_duration_sec=2.0)
        # prune old
        while self.buffer and (now - self.buffer[0].stamp) > self.accum_window:
            self.buffer.popleft()
        return True

    def _tf_to_Rt(self, tf):
        q = (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w)
        t = np.array([tf.transform.translation.x, tf.transform.translation.y,
                      tf.transform.translation.z])
        return quat_to_rotmat(q), t

    # ---- fit + low-pass ----
    def update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        # Expire observations even when the camera/detector stops publishing. Previously
        # pruning happened only in det_cb, so one stale frame could be reused forever.
        while self.buffer and (now - self.buffer[0].stamp) > self.accum_window:
            self.buffer.popleft()
        if not self.buffer:
            return
        # A global pose must come from markers detected in the SAME camera image.
        # Combining alternating single-marker messages over the rolling window creates
        # a fictitious multi-marker observation and can initialize from a moving rig.
        items = latest_capture_batch(self.buffer)
        self._publish_observed(items)

        if not self.frozen:
            self._update_datum(items)
            self._maybe_freeze()
        else:
            self._update_camera_init_candidate(items)

    def _update_datum(self, items):
        if len(items) == 2:
            baseline = float(np.linalg.norm(items[0].p_datum[:2] - items[1].p_datum[:2]))
            if baseline < self.min_baseline_2pt:
                # Near-collinear-from-here pair: a 2-point Procrustes yaw is fragile here
                # (small per-marker noise -> large yaw swing) -- fall back to the closer
                # marker's single-marker path instead of trusting the 2-point fit.
                best = min(items, key=lambda e: e.rng)
                self.get_logger().info(
                    f'2 markers {sorted(i.mid for i in items)} but datum baseline '
                    f'{baseline:.2f} m < {self.min_baseline_2pt:.2f} m: using single-marker '
                    f'path on the closer one (id{best.mid}) instead of the 2-point fit.',
                    throttle_duration_sec=2.0)
                result = self._single_marker_correction(best)
            else:
                result = self._weighted_fit(items, dst_attr='p_map')
        elif len(items) >= 3:
            result = self._weighted_fit(items, dst_attr='p_map')
        else:
            result = self._single_marker_correction(items[0])

        if result is None:
            return
        tx, ty, yaw, res, a_yaw, n_used = result

        # jump gate (bad match / discontinuity) — §6.4 safety valve
        djump = math.hypot(tx - self.tx, ty - self.ty)
        dyaw = abs(wrap(yaw - self.yaw))
        if djump > self.max_jump_trans or dyaw > self.max_jump_yaw:
            self.get_logger().warn(
                f'correction jump too large (dt={djump:.2f} m, dyaw={dyaw:.2f} rad) — rejected',
                throttle_duration_sec=3.0)
            return

        # low-pass (never snap)
        self.tx += self.alpha * (tx - self.tx)
        self.ty += self.alpha * (ty - self.ty)
        self.yaw = wrap(self.yaw + a_yaw * wrap(yaw - self.yaw))
        self.residual_pub.publish(Float32(data=res))
        self.get_logger().info(
            f'corrected map->datum from {n_used} marker(s): '
            f'yaw={math.degrees(self.yaw):.1f} deg, '
            f't=({self.tx:.2f}, {self.ty:.2f}), residual={res:.3f} m',
            throttle_duration_sec=2.0)
        if n_used >= 2:
            self.multimarker_count += 1

    def _maybe_freeze(self):
        if self.frozen:
            return
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.start_time_sec
        if (self.multimarker_count >= self.freeze_min_multimarker
                or elapsed >= self.freeze_max_wait_sec):
            self.frozen = True
            self.get_logger().info(
                f'map->datum FROZEN at yaw={math.degrees(self.yaw):.1f} deg, '
                f't=({self.tx:.2f}, {self.ty:.2f}) after {self.multimarker_count} '
                f'multi-marker fit(s), {elapsed:.1f}s since start. Now estimating '
                f'{self.map_frame}->{self.odom_frame} corrections from ArUco instead '
                f'(published on /erc/aruco_camera_init_candidate).')

    def _update_camera_init_candidate(self, items):
        """Publish one-marker xy-only or multi-marker full planar candidates."""
        if not items:
            return
        if len(items) == 1:
            self._update_single_marker_camera_init_candidate(items[0])
            return
        if len(items) == 2:
            baseline = float(np.linalg.norm(items[0].p_datum[:2] - items[1].p_datum[:2]))
            if baseline < self.min_baseline_2pt:
                best = min(items, key=lambda e: e.rng)
                self.get_logger().info(
                    f'2 markers {sorted(i.mid for i in items)} but datum baseline '
                    f'{baseline:.2f} m < {self.min_baseline_2pt:.2f} m: too fragile for a '
                    f'full yaw candidate; using closer id{best.mid} for translation-only '
                    'tracking.',
                    throttle_duration_sec=2.0)
                self._update_single_marker_camera_init_candidate(best)
                return
        # A trustworthy full multi-marker observation supersedes one-marker history.
        self.single_candidate_history.clear()
        result = self._weighted_fit(items, dst_attr='p_camera_init')
        if result is None:
            return
        cd_tx, cd_ty, cd_yaw, res, _, n_used = result
        map_to_coordinates = (
            (0.0, 0.0, 0.0) if self.coordinate_frame == self.map_frame
            else (self.tx, self.ty, self.yaw))
        tx, ty, yaw = map_camera_init_from_datum_fit(
            map_to_coordinates, (cd_tx, cd_ty, cd_yaw))

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = tx
        msg.pose.pose.position.y = ty
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Not a real covariance -- reusing the field to carry the fit's RMS residual
        # [m] as a simple quality scalar (diagonal x/y slots) for map_anchor to gate on.
        msg.pose.covariance[0] = res
        msg.pose.covariance[7] = res
        self.candidate_pub.publish(msg)
        self.get_logger().info(
            f'{self.map_frame}->{self.odom_frame} candidate from {n_used} marker(s): '
            f'yaw={math.degrees(yaw):.1f} deg t=({tx:.2f}, {ty:.2f}) residual={res:.3f} m',
            throttle_duration_sec=2.0)

    def _update_single_marker_camera_init_candidate(self, entry):
        """After global initialization, constrain x/y from one known marker.

        One point cannot independently observe planar yaw. We therefore hold the
        timestamped GICP/FAST-LIO map->camera_init yaw and require several consistent
        same-ID translation estimates before publishing an explicitly translation-only
        candidate. map_anchor.cpp preserves the complete current rotation for it.
        """
        if not self.single_tracking_enabled:
            return
        if not self.localization_initialized:
            self.get_logger().info(
                f'one marker id{entry.mid} visible, but global localization is not '
                'initialized; waiting for a multi-marker initializer',
                throttle_duration_sec=2.0)
            self.single_candidate_history.clear()
            return
        if entry.view_deg < self.min_view_angle_deg:
            self.get_logger().warn(
                f'one-marker tracking id{entry.mid} viewed {entry.view_deg:.0f} deg '
                f'off grazing (< {self.min_view_angle_deg:.0f} deg floor); rejected',
                throttle_duration_sec=2.0)
            return

        map_to_coordinates = (
            (0.0, 0.0, 0.0) if self.coordinate_frame == self.map_frame
            else (self.tx, self.ty, self.yaw))
        tx, ty = single_marker_translation_candidate(
            entry.p_datum, entry.p_camera_init, map_to_coordinates,
            entry.map_camera_init_yaw)

        history = self.single_candidate_history[entry.mid]
        # update() can run more than once while the same camera frame remains the
        # newest entry in the rolling buffer. Never count one image as multiple
        # independent confirmations.
        if not history or history[-1][1] != entry.capture_ns:
            history.append((
                entry.stamp, entry.capture_ns, tx, ty,
                entry.map_camera_init_yaw))
        while history and entry.stamp - history[0][0] > self.single_history_sec:
            history.popleft()
        # Bound memory if timestamps stop advancing in a synthetic/replayed stream.
        while len(history) > max(10, 3 * self.single_min_consistent):
            history.popleft()

        result = translation_consensus(
            [(row[2], row[3]) for row in history],
            self.single_consistency_trans, self.single_min_consistent)
        if result is None:
            self.get_logger().info(
                f'one-marker tracking id{entry.mid}: collecting consistent candidates '
                f'({len(history)}/{self.single_min_consistent})',
                throttle_duration_sec=2.0)
            return
        tx, ty, spread, n_used = result
        # Use the newest timestamped global yaw only as a placeholder in the message;
        # covariance[35] tells the fusion node not to apply this yaw.
        candidate_yaw = history[-1][4]

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = tx
        msg.pose.pose.position.y = ty
        qx, qy, qz, qw = yaw_to_quat(candidate_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = spread
        msg.pose.covariance[7] = spread
        msg.pose.covariance[35] = TRANSLATION_ONLY_YAW_VARIANCE
        self.candidate_pub.publish(msg)
        innovation = float(np.linalg.norm(
            np.array([tx, ty]) - np.asarray(entry.map_camera_init_xy)))
        self.get_logger().info(
            f'{self.map_frame}->{self.odom_frame} translation-only candidate from '
            f'id{entry.mid}: t=({tx:.2f}, {ty:.2f}), yaw held at '
            f'{math.degrees(candidate_yaw):.1f} deg, spread={spread:.3f} m '
            f'({n_used} samples), innovation={innovation:.3f} m',
            throttle_duration_sec=2.0)

    def _weighted_fit(self, items, dst_attr='p_map'):
        """>=2 markers with a trustworthy baseline: SE(2) Procrustes weighted ~1/range^2
        (closer/more-trusted observations count for more). dst_attr selects which
        observed-frame field to fit against ('p_map' for the datum phase, 'p_camera_init'
        for the post-freeze phase). Returns None if rejected."""
        src3 = np.array([e.p_datum for e in items])
        dst3 = np.array([getattr(e, dst_attr) for e in items])
        distance_error, relative_error = pairwise_distance_consistency(src3, dst3)
        if ((self.max_pair_distance_error > 0.0
             and distance_error > self.max_pair_distance_error)
                or (self.max_pair_distance_relative_error > 0.0
                    and relative_error > self.max_pair_distance_relative_error)):
            self.get_logger().warn(
                f'known/observed 3-D landmark spacing disagrees by '
                f'{distance_error:.3f} m ({100.0 * relative_error:.1f}%); '
                f'limits are {self.max_pair_distance_error:.3f} m / '
                f'{100.0 * self.max_pair_distance_relative_error:.1f}%. '
                'Check YAML coordinates, actual printed marker size, and '
                'face_to_pole_depth. Pose candidate rejected.',
                throttle_duration_sec=3.0)
            return None

        src = src3[:, :2]        # datum xy (known, fixed); rover pose is planar
        dst = dst3[:, :2]
        weights = np.array([1.0 / (e.rng ** 2 + 1e-3) for e in items])
        tx, ty, yaw, res = fit_se2(src, dst, weights)
        if res > self.residual_max:
            self.get_logger().warn(
                f'fit residual {res:.2f} > {self.residual_max} (rejected)',
                throttle_duration_sec=3.0)
            return None
        return tx, ty, yaw, res, self.alpha, len(items)

    def _single_marker_correction(self, e):
        """Derive a map->datum correction from ONE trusted marker (datum phase only --
        the post-freeze camera_init phase has no equivalent, see its docstring). Keeps
        translation at the seed (monocular translation is unreliable); only yaw is
        corrected, and only when the marker is close (max range) AND roughly frontal
        (min view angle) — §6.4 safety valve against planar-pose ambiguity. Returns None
        if gated out."""
        if e.rng > self.yaw_range:
            self.get_logger().warn(
                f'1 usable marker (id{e.mid}) at {e.rng:.2f} m > yaw_update_max_range '
                f'{self.yaw_range:.2f} m: too far to correct yaw from a lone marker. '
                'Bring it closer, add a 2nd (well-separated) marker, or raise '
                'yaw_update_max_range.', throttle_duration_sec=2.0)
            return None
        if e.view_deg < self.min_view_angle_deg:
            self.get_logger().warn(
                f'1 usable marker (id{e.mid}) viewed {e.view_deg:.0f} deg off grazing '
                f'(< {self.min_view_angle_deg:.0f} deg floor): too oblique to trust a '
                'lone marker\'s yaw (planar-pose ambiguity). Face it more directly.',
                throttle_duration_sec=2.0)
            return None
        tx, ty = float(self.seed_xy[0]), float(self.seed_xy[1])
        p_d, p_o = e.p_datum[:2], e.p_map[:2]
        yaw = wrap(math.atan2(p_o[1] - ty, p_o[0] - tx) - math.atan2(p_d[1], p_d[0]))
        res = float(np.linalg.norm(p_o - (self._R2(yaw) @ p_d + np.array([tx, ty]))))
        return tx, ty, yaw, res, self.alpha * self.yaw_weight, 1

    def _R2(self, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s], [s, c]])

    # ---- outputs ----
    def broadcast_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.datum_frame
        tf.transform.translation.x = self.tx
        tf.transform.translation.y = self.ty
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.bc.sendTransform(tf)

    def _publish_known_markers(self):
        arr = MarkerArray()
        for i, (mid, p) in enumerate(sorted(self.known.items())):
            m = Marker()
            m.header.frame_id = self.coordinate_frame
            m.ns = 'aruco_known'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.9, 0.1, 0.9
            arr.markers.append(m)
            tm = Marker()
            tm.header.frame_id = self.coordinate_frame
            tm.ns = 'aruco_known_label'
            tm.id = i
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x, tm.pose.position.y, tm.pose.position.z = p[0], p[1], p[2] + 0.3
            tm.pose.orientation.w = 1.0
            tm.scale.z = 0.25
            tm.color.r = tm.color.g = tm.color.b = tm.color.a = 1.0
            tm.text = f'id{mid}'
            arr.markers.append(tm)
        self.known_pub.publish(arr)

    def _publish_observed(self, items):
        arr = MarkerArray()
        for i, e in enumerate(items):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'aruco_observed'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = e.p_map
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.6, 1.0, 0.9
            arr.markers.append(m)
        self.obs_pub.publish(arr)


def main():
    rclpy.init()
    node = ArucoMapAnchor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
