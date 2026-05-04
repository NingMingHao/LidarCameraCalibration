#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Multi-timestamp fisheye lidar-camera manual pairing tool.

This tool reads a rosbag directly, builds synchronized lidar/image anchors
using lidar timestamps as the reference, downsamples those anchors for
manageable browsing, republishes selected data under /calibration/*, and
collects 3D-2D correspondences across multiple timestamps.
"""

import argparse
import bisect
import json
import math
import os
import sys
import threading
from copy import deepcopy
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from PyQt5 import QtCore, QtGui, QtWidgets

try:
    import cv2
except ImportError as exc:
    raise RuntimeError("OpenCV/cv2 is required for this tool.") from exc

try:
    from scipy.optimize import least_squares
except ImportError as exc:
    raise RuntimeError("scipy is required for fisheye extrinsic refinement.") from exc


CALIB_PREFIX = "/calibration"
DEFAULT_RESULT_FILENAME = "fisheye_lidar_camera_calibration_result.yaml"


@dataclass
class SyncFrame:
    sync_id: int
    cloud_idx: int
    cloud_time: float
    image_time: Optional[float]
    cam_info_time: Optional[float]
    image_dt: Optional[float]
    cam_info_dt: Optional[float]
    valid: bool


@dataclass
class CameraModel:
    width: int
    height: int
    frame_id: str
    distortion_model: str
    K: np.ndarray
    D: np.ndarray
    R: np.ndarray
    P: np.ndarray
    source: str


class StaticTFTree:
    def __init__(self):
        self.transforms: Dict[str, Tuple[str, np.ndarray]] = {}

    def add_transform_quat(self, parent: str, child: str, trans, rot):
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = tf.transformations.quaternion_matrix(rot)[:3, :3]
        T[:3, 3] = np.array(trans, dtype=np.float64)
        self.transforms[child] = (parent, T)

    def lookup(self, target_frame: str, source_frame: str) -> Optional[np.ndarray]:
        if not target_frame or not source_frame:
            return None
        if target_frame == source_frame:
            return np.eye(4, dtype=np.float64)

        def root_map(frame: str):
            out = {frame: np.eye(4, dtype=np.float64)}
            curr = frame
            T_root_curr = np.eye(4, dtype=np.float64)
            while curr in self.transforms:
                parent, T_parent_curr = self.transforms[curr]
                T_root_curr = T_parent_curr @ T_root_curr
                curr = parent
                out[curr] = T_root_curr
            return out

        src = root_map(source_frame)
        tgt = root_map(target_frame)
        common = set(src) & set(tgt)
        if not common:
            return None
        ancestor = next(iter(common))
        return np.linalg.inv(tgt[ancestor]) @ src[ancestor]


def nearest(cache: List[Tuple[float, object]], t_ref: float) -> Tuple[Optional[object], Optional[float], Optional[float]]:
    if not cache:
        return None, None, None
    times = [t for t, _ in cache]
    idx = bisect.bisect_left(times, t_ref)
    candidates = []
    if idx < len(cache):
        candidates.append(cache[idx])
    if idx > 0:
        candidates.append(cache[idx - 1])
    t_msg, msg = min(candidates, key=lambda p: abs(p[0] - t_ref))
    return msg, t_msg, t_msg - t_ref


def prefer_topic(topics: List[str], requested: Optional[str], contains: str) -> str:
    if requested:
        if requested not in topics:
            raise RuntimeError(f"Requested topic not found in bag: {requested}")
        return requested
    preferred = [t for t in topics if contains in t]
    if preferred:
        return sorted(preferred)[0]
    if not topics:
        raise RuntimeError(f"No topic found matching {contains}")
    return sorted(topics)[0]


def read_bag_data(args):
    bag = rosbag.Bag(args.bag, "r")
    info = bag.get_type_and_topic_info()[1]

    image_topics = []
    cam_info_topics = []
    cloud_topics = []
    for topic, topic_info in info.items():
        msg_type = topic_info.msg_type
        if msg_type in ("sensor_msgs/Image", "sensor_msgs/CompressedImage"):
            image_topics.append(topic)
        elif msg_type == "sensor_msgs/CameraInfo":
            cam_info_topics.append(topic)
        elif msg_type == "sensor_msgs/PointCloud2":
            cloud_topics.append(topic)

    image_topic = prefer_topic(image_topics, args.image_topic, "image_raw")
    cloud_topic = prefer_topic(cloud_topics, args.lidar_topic, "rslidar_points_front")

    if args.camera_info_topic:
        cam_info_topic = prefer_topic(cam_info_topics, args.camera_info_topic, "camera_info")
    else:
        image_ns = image_topic.rsplit("/", 2)[0]
        matching = [t for t in cam_info_topics if t.rsplit("/", 1)[0] == image_ns]
        cam_info_topic = sorted(matching)[0] if matching else prefer_topic(cam_info_topics, None, "camera_info")

    caches = {image_topic: [], cam_info_topic: [], cloud_topic: []}
    tf_tree = StaticTFTree()
    tf_pairs: Dict[Tuple[str, str], np.ndarray] = {}

    for topic, msg, stamp in bag.read_messages():
        t_sec = stamp.to_sec()
        if topic in caches:
            caches[topic].append((t_sec, msg))
        elif topic == "/tf":
            for tr in msg.transforms:
                parent = tr.header.frame_id
                child = tr.child_frame_id
                trans = (tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z)
                rot = (tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w)
                tf_tree.add_transform_quat(parent, child, trans, rot)
                key = (parent, child)
                if key not in tf_pairs:
                    T = np.eye(4, dtype=np.float64)
                    T[:3, :3] = tf.transformations.quaternion_matrix(rot)[:3, :3]
                    T[:3, 3] = np.array(trans, dtype=np.float64)
                    tf_pairs[key] = T

    bag.close()
    for cache in caches.values():
        cache.sort(key=lambda x: x[0])

    return {
        "image_topic": image_topic,
        "camera_info_topic": cam_info_topic,
        "cloud_topic": cloud_topic,
        "image_cache": caches[image_topic],
        "cam_info_cache": caches[cam_info_topic],
        "cloud_cache": caches[cloud_topic],
        "tf_tree": tf_tree,
        "tf_pairs": tf_pairs,
    }


def build_sync_frames(cloud_cache, image_cache, cam_info_cache, tolerance: float) -> Tuple[List[SyncFrame], List[SyncFrame]]:
    all_frames = []
    for i, (cloud_time, _) in enumerate(cloud_cache):
        _, img_t, img_dt = nearest(image_cache, cloud_time)
        _, cam_t, cam_dt = nearest(cam_info_cache, cloud_time)
        valid = (
            img_t is not None and cam_t is not None
            and abs(img_dt) <= tolerance and abs(cam_dt) <= tolerance
        )
        all_frames.append(SyncFrame(
            sync_id=i,
            cloud_idx=i,
            cloud_time=cloud_time,
            image_time=img_t,
            cam_info_time=cam_t,
            image_dt=img_dt,
            cam_info_dt=cam_dt,
            valid=valid,
        ))
    return all_frames, [f for f in all_frames if f.valid]


def downsample_frames(valid_frames: List[SyncFrame], frame_step: int, target_hz: Optional[float], max_frames: Optional[int]) -> List[SyncFrame]:
    if not valid_frames:
        return []
    selected = valid_frames
    if target_hz is not None and target_hz > 0 and len(valid_frames) > 1:
        min_dt = 1.0 / target_hz
        out = []
        last_t = -float("inf")
        for frame in valid_frames:
            if frame.cloud_time - last_t >= min_dt - 1e-9:
                out.append(frame)
                last_t = frame.cloud_time
        selected = out
    else:
        selected = valid_frames[::max(1, frame_step)]

    if max_frames is not None and max_frames > 0 and len(selected) > max_frames:
        idx = np.linspace(0, len(selected) - 1, max_frames).round().astype(int)
        selected = [selected[int(i)] for i in idx]
    return selected


def camera_model_from_msg(msg: CameraInfo, source: str) -> CameraModel:
    K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
    D = np.array(msg.D, dtype=np.float64).reshape(-1)
    R = np.array(msg.R, dtype=np.float64).reshape(3, 3)
    P = np.array(msg.P, dtype=np.float64).reshape(3, 4)
    return CameraModel(msg.width, msg.height, msg.header.frame_id, msg.distortion_model, K, D, R, P, source)


def camera_model_from_yaml(path: str, frame_id: str) -> CameraModel:
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
    R = np.array(data.get("rectification_matrix", {}).get("data", np.eye(3).reshape(-1)), dtype=np.float64).reshape(3, 3)
    P = np.array(data.get("projection_matrix", {}).get("data", [
        K[0, 0], 0, K[0, 2], 0,
        0, K[1, 1], K[1, 2], 0,
        0, 0, 1, 0,
    ]), dtype=np.float64).reshape(3, 4)
    return CameraModel(
        int(data["image_width"]),
        int(data["image_height"]),
        frame_id,
        data.get("distortion_model", "equidistant"),
        K,
        D,
        R,
        P,
        path,
    )


def make_camera_info(model: CameraModel, stamp: rospy.Time) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = model.frame_id
    msg.width = int(model.width)
    msg.height = int(model.height)
    msg.distortion_model = model.distortion_model
    msg.K = model.K.reshape(-1).tolist()
    msg.D = model.D.reshape(-1).tolist()
    msg.R = model.R.reshape(-1).tolist()
    msg.P = model.P.reshape(-1).tolist()
    return msg


def ros_now_for_live_publish() -> rospy.Time:
    now = rospy.Time.now()
    if now.to_sec() == 0.0 and rospy.get_param("/use_sim_time", False):
        rospy.logwarn_throttle(
            5.0,
            "[PairTool] /use_sim_time is true but ROS time is 0. "
            "Start /clock or set /use_sim_time false for this live calibration tool.",
        )
    return now


def decode_image(bridge: CvBridge, msg) -> np.ndarray:
    if isinstance(msg, CompressedImage) or getattr(msg, "_type", "") == "sensor_msgs/CompressedImage":
        return bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def rectify_image(img_bgr: np.ndarray, model: CameraModel) -> np.ndarray:
    if model.distortion_model.lower() in ("equidistant", "fisheye"):
        D = model.D[:4].reshape(4, 1)
        return cv2.fisheye.undistortImage(img_bgr, model.K, D=D, Knew=model.P[:3, :3])
    return cv2.undistort(img_bgr, model.K, model.D, None, model.P[:3, :3])


def default_result_path(args) -> str:
    if getattr(args, "result_yaml", ""):
        return os.path.abspath(os.path.expanduser(args.result_yaml))
    if getattr(args, "camera_yaml", ""):
        return os.path.join(os.path.dirname(os.path.abspath(args.camera_yaml)), DEFAULT_RESULT_FILENAME)
    if getattr(args, "bag", ""):
        return os.path.join(os.path.dirname(os.path.abspath(args.bag)), DEFAULT_RESULT_FILENAME)
    return os.path.abspath(DEFAULT_RESULT_FILENAME)


class CalibrationBackend(QtCore.QObject):
    frame_changed = QtCore.pyqtSignal(object, object)
    pairs_changed = QtCore.pyqtSignal()
    tf_guess_changed = QtCore.pyqtSignal()
    calibration_finished = QtCore.pyqtSignal(bool, str)

    def __init__(self, args, bag_data, all_frames: List[SyncFrame], valid_frames: List[SyncFrame], display_frames: List[SyncFrame]):
        super().__init__()
        self.args = args
        self.bag_data = bag_data
        self.all_frames = all_frames
        self.valid_frames = valid_frames
        self.display_frames = display_frames
        self.show_all_frames = False
        self.current_sync_id: Optional[int] = display_frames[0].sync_id if display_frames else None

        self.bridge = CvBridge()
        self.tf_tree: StaticTFTree = bag_data["tf_tree"]
        self.tf_pairs = bag_data["tf_pairs"]
        self.pairs: List[Dict] = []
        self.next_pair_index = 0
        self.last_marker_ids = set()
        self.latest_T_cam_lidar: Optional[np.ndarray] = None
        self.tf_guess_source = "unset"
        self.result_path = default_result_path(args)
        self.last_result_summary = "No calibration result saved yet."

        self.active_camera_model: Optional[CameraModel] = None
        self.current_raw_image: Optional[np.ndarray] = None
        self.current_rect_image: Optional[np.ndarray] = None
        self.current_cloud: Optional[PointCloud2] = None
        self.lidar_frame: Optional[str] = None
        self.camera_frame: Optional[str] = None

        self.pub_cloud = rospy.Publisher(f"{CALIB_PREFIX}/rslidar_points_front", PointCloud2, queue_size=1, latch=True)
        self.pub_raw = rospy.Publisher(f"{CALIB_PREFIX}/camera/image_raw", Image, queue_size=1, latch=True)
        self.pub_rect = rospy.Publisher(f"{CALIB_PREFIX}/camera/image_rect", Image, queue_size=1, latch=True)
        self.pub_info = rospy.Publisher(f"{CALIB_PREFIX}/camera/camera_info", CameraInfo, queue_size=1, latch=True)
        self.pub_markers = rospy.Publisher(f"{CALIB_PREFIX}/pairs_markers", MarkerArray, queue_size=1, latch=True)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.clicked_sub = rospy.Subscriber("/clicked_point", PointStamped, self.on_clicked_point, queue_size=10)

        self.stop_tf = threading.Event()
        self.tf_thread = threading.Thread(target=self.tf_loop)
        self.tf_thread.daemon = True
        self.tf_thread.start()

    def ensure_tf_guess(self):
        if self.latest_T_cam_lidar is not None or not self.camera_frame or not self.lidar_frame:
            return
        T = self.tf_tree.lookup(self.camera_frame, self.lidar_frame)
        if T is not None:
            self.latest_T_cam_lidar = T.copy()
            self.tf_guess_source = "bag /tf"
        else:
            self.latest_T_cam_lidar = np.eye(4, dtype=np.float64)
            self.tf_guess_source = "identity fallback"
        self.tf_guess_changed.emit()

    def frames_for_ui(self) -> List[SyncFrame]:
        return self.valid_frames if self.show_all_frames else self.display_frames

    def frame_by_id(self, sync_id: int) -> Optional[SyncFrame]:
        for frame in self.all_frames:
            if frame.sync_id == sync_id:
                return frame
        return None

    def set_show_all_frames(self, show: bool):
        self.show_all_frames = show

    def pair_count_for_frame(self, sync_id: int) -> int:
        return sum(1 for p in self.pairs if p["sync_id"] == sync_id)

    def load_frame(self, sync_id: int):
        frame = self.frame_by_id(sync_id)
        if frame is None or not frame.valid:
            return
        self.current_sync_id = sync_id
        cloud_msg, _, _ = nearest(self.bag_data["cloud_cache"], frame.cloud_time)
        image_msg, _, _ = nearest(self.bag_data["image_cache"], frame.image_time)
        cam_msg, _, _ = nearest(self.bag_data["cam_info_cache"], frame.cam_info_time)
        if cloud_msg is None or image_msg is None or cam_msg is None:
            return

        stamp = ros_now_for_live_publish()
        raw = decode_image(self.bridge, image_msg)
        bag_model = camera_model_from_msg(cam_msg, "bag camera_info")
        self.camera_frame = bag_model.frame_id
        self.lidar_frame = cloud_msg.header.frame_id
        if self.args.camera_yaml:
            model = camera_model_from_yaml(self.args.camera_yaml, bag_model.frame_id)
            if raw.shape[1] != model.width or raw.shape[0] != model.height:
                raw = cv2.resize(raw, (model.width, model.height), interpolation=cv2.INTER_LINEAR)
        else:
            model = bag_model

        rect = rectify_image(raw, model)
        self.active_camera_model = model
        self.current_raw_image = raw
        self.current_rect_image = rect
        self.current_cloud = deepcopy(cloud_msg)
        self.current_cloud.header.stamp = stamp

        raw_msg = self.bridge.cv2_to_imgmsg(raw, encoding="bgr8")
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = model.frame_id
        rect_msg = self.bridge.cv2_to_imgmsg(rect, encoding="bgr8")
        rect_msg.header.stamp = stamp
        rect_msg.header.frame_id = model.frame_id

        self.pub_cloud.publish(self.current_cloud)
        self.pub_raw.publish(raw_msg)
        self.pub_rect.publish(rect_msg)
        self.pub_info.publish(make_camera_info(model, stamp))
        self.ensure_tf_guess()
        self.publish_markers()
        self.frame_changed.emit(raw, rect)

    def republish_current_frame(self):
        if self.current_cloud is None or self.current_raw_image is None or \
           self.current_rect_image is None or self.active_camera_model is None:
            return

        stamp = ros_now_for_live_publish()
        self.current_cloud.header.stamp = stamp
        self.broadcast_current_tf(stamp)

        raw_msg = self.bridge.cv2_to_imgmsg(self.current_raw_image, encoding="bgr8")
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.active_camera_model.frame_id

        rect_msg = self.bridge.cv2_to_imgmsg(self.current_rect_image, encoding="bgr8")
        rect_msg.header.stamp = stamp
        rect_msg.header.frame_id = self.active_camera_model.frame_id

        self.pub_cloud.publish(self.current_cloud)
        self.pub_raw.publish(raw_msg)
        self.pub_rect.publish(rect_msg)
        self.pub_info.publish(make_camera_info(self.active_camera_model, stamp))
        self.publish_markers()

    def on_clicked_point(self, msg: PointStamped):
        if self.current_sync_id is None or self.lidar_frame is None:
            rospy.logwarn("[PairTool] Select a synchronized frame before picking 3D points.")
            return
        T_lidar_src = self.tf_tree.lookup(self.lidar_frame, msg.header.frame_id)
        if T_lidar_src is None:
            if msg.header.frame_id == self.lidar_frame:
                T_lidar_src = np.eye(4)
            else:
                rospy.logwarn(f"[PairTool] No TF from {msg.header.frame_id} to {self.lidar_frame}")
                return
        pt_src = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0], dtype=np.float64)
        pt_lidar = (T_lidar_src @ pt_src)[:3]
        pair = {
            "index": self.next_pair_index,
            "sync_id": self.current_sync_id,
            "pt3d": pt_lidar,
            "pt2d": None,
            "lidar_frame": self.lidar_frame,
            "camera_frame": self.camera_frame,
            "error": None,
        }
        self.next_pair_index += 1
        self.pairs.append(pair)
        self.pairs_changed.emit()
        self.publish_markers()

    def assign_pixel(self, pair_index: Optional[int], u: float, v: float):
        target = None
        if pair_index is not None:
            target = next((p for p in self.pairs if p["index"] == pair_index), None)
        if target is None:
            target = next((p for p in self.pairs if p["sync_id"] == self.current_sync_id and p["pt2d"] is None), None)
        if target is None:
            return False
        target["pt2d"] = np.array([u, v], dtype=np.float64)
        self.pairs_changed.emit()
        return True

    def remove_pair(self, index: int):
        self.pairs = [p for p in self.pairs if p["index"] != index]
        self.pairs_changed.emit()
        self.publish_markers()

    def visible_pairs(self, current_only: bool) -> List[Dict]:
        if current_only:
            return [p for p in self.pairs if p["sync_id"] == self.current_sync_id]
        return list(self.pairs)

    def publish_markers(self):
        ma = MarkerArray()
        active_ids = set()
        stamp = ros_now_for_live_publish()
        for p in self.pairs:
            pt = p["pt3d"]
            current = p["sync_id"] == self.current_sync_id
            sphere_id = p["index"] * 2
            text_id = p["index"] * 2 + 1
            active_ids.update([sphere_id, text_id])

            color = (0.0, 1.0, 0.1, 1.0) if current else (0.55, 0.55, 0.55, 0.28)
            scale = 0.35 if current else 0.18
            for marker_id, marker_type, dz in [(sphere_id, Marker.SPHERE, 0.0), (text_id, Marker.TEXT_VIEW_FACING, 0.45)]:
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = p["lidar_frame"]
                m.ns = "calib_pairs_current" if current else "calib_pairs_ghost"
                m.id = marker_id
                m.type = marker_type
                m.action = Marker.ADD
                m.pose.position.x = float(pt[0])
                m.pose.position.y = float(pt[1])
                m.pose.position.z = float(pt[2] + dz)
                m.pose.orientation.w = 1.0
                m.scale.x = scale
                m.scale.y = scale
                m.scale.z = scale if marker_type == Marker.SPHERE else 0.35
                m.color.r, m.color.g, m.color.b, m.color.a = color
                if marker_type == Marker.TEXT_VIEW_FACING:
                    m.text = f"{p['index']}@{p['sync_id']}"
                ma.markers.append(m)

        for old_id in self.last_marker_ids - active_ids:
            for ns in ("calib_pairs_current", "calib_pairs_ghost"):
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = self.lidar_frame or ""
                m.ns = ns
                m.id = old_id
                m.action = Marker.DELETE
                ma.markers.append(m)
        self.last_marker_ids = active_ids
        self.pub_markers.publish(ma)

    def tf_loop(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and not self.stop_tf.is_set():
            now = ros_now_for_live_publish()
            self.broadcast_current_tf(now)
            rate.sleep()

    def broadcast_current_tf(self, stamp: Optional[rospy.Time] = None):
        if stamp is None:
            stamp = ros_now_for_live_publish()
        for (parent, child), T in self.tf_pairs.items():
            if self.latest_T_cam_lidar is not None and child == self.camera_frame:
                continue
            t = T[:3, 3]
            q = tf.transformations.quaternion_from_matrix(T)
            self.tf_broadcaster.sendTransform(tuple(t), tuple(q), stamp, child, parent)
        if self.latest_T_cam_lidar is not None and self.lidar_frame and self.camera_frame:
            T_lidar_cam = np.linalg.inv(self.latest_T_cam_lidar)
            t = T_lidar_cam[:3, 3]
            q = tf.transformations.quaternion_from_matrix(T_lidar_cam)
            self.tf_broadcaster.sendTransform(tuple(t), tuple(q), stamp, self.camera_frame, self.lidar_frame)

    def current_T_lidar_cam(self) -> Optional[np.ndarray]:
        if self.latest_T_cam_lidar is None:
            return None
        return np.linalg.inv(self.latest_T_cam_lidar)

    def current_static_tf_values(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        T_lidar_cam = self.current_T_lidar_cam()
        if T_lidar_cam is None:
            return None
        x, y, z = [float(v) for v in T_lidar_cam[:3, 3]]
        roll, pitch, yaw = [float(v) for v in tf.transformations.euler_from_matrix(T_lidar_cam, axes="sxyz")]
        return x, y, z, yaw, pitch, roll

    def current_static_tf_string(self) -> str:
        vals = self.current_static_tf_values()
        if vals is None:
            return "No TF guess available yet. Select a synchronized frame first."
        x, y, z, yaw, pitch, roll = vals
        return (
            f"{x:.6f} {y:.6f} {z:.6f} "
            f"{yaw:.6f} {pitch:.6f} {roll:.6f} "
            f"{self.lidar_frame or '<lidar_frame>'} {self.camera_frame or '<camera_frame>'}"
        )

    def set_static_tf_values(self, x: float, y: float, z: float, yaw: float, pitch: float, roll: float, source: str):
        T_lidar_cam = tf.transformations.euler_matrix(roll, pitch, yaw, axes="sxyz")
        T_lidar_cam[:3, 3] = [x, y, z]
        self.latest_T_cam_lidar = np.linalg.inv(T_lidar_cam)
        self.tf_guess_source = source
        self.republish_current_frame()
        self.tf_guess_changed.emit()

    def reset_tf_guess_from_bag(self) -> Tuple[bool, str]:
        if not self.camera_frame or not self.lidar_frame:
            return False, "Select a synchronized frame first."
        T = self.tf_tree.lookup(self.camera_frame, self.lidar_frame)
        if T is None:
            return False, f"No bag TF from {self.lidar_frame} to {self.camera_frame}."
        self.latest_T_cam_lidar = T.copy()
        self.tf_guess_source = "bag /tf"
        self.republish_current_frame()
        self.tf_guess_changed.emit()
        return True, "Loaded current TF guess from bag /tf."

    def evaluate_current_tf(self) -> Tuple[bool, str]:
        complete = [p for p in self.pairs if p["pt2d"] is not None]
        if len(complete) < 1:
            return False, "No complete pairs to evaluate."
        if self.latest_T_cam_lidar is None or self.active_camera_model is None:
            return False, "Select a frame and set a TF guess first."
        err = self.compute_reprojection_errors(self.latest_T_cam_lidar, complete)
        for p, e in zip(complete, err):
            p["error"] = float(e)
        self.pairs_changed.emit()
        return True, (
            f"Current TF reprojection: RMS={np.sqrt(np.mean(err ** 2)):.3f}px, "
            f"p95={np.percentile(err, 95):.3f}px, max={np.max(err):.3f}px"
        )

    def save_pairs(self, path: str) -> Tuple[bool, str]:
        path = os.path.abspath(os.path.expanduser(path))
        data = self.export_data(include_result=False)
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            with open(path, "w") as f:
                if path.lower().endswith((".yaml", ".yml")):
                    yaml.safe_dump(data, f, sort_keys=False)
                else:
                    json.dump(data, f, indent=2)
            return True, f"Saved {len(self.pairs)} pairs."
        except Exception as exc:
            return False, str(exc)

    def load_pairs(self, path: str) -> Tuple[bool, str]:
        path = os.path.abspath(os.path.expanduser(path))
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) if path.lower().endswith((".yaml", ".yml")) else json.load(f)
            if data is None:
                data = {}
            self.pairs = []
            for item in data.get("pairs", []):
                pair = {
                    "index": int(item["index"]),
                    "sync_id": int(item["sync_id"]),
                    "pt3d": np.array(item["pt3d_lidar"], dtype=np.float64),
                    "pt2d": np.array(item["pt2d_raw"], dtype=np.float64) if item.get("pt2d_raw") is not None else None,
                    "lidar_frame": item.get("lidar_frame", self.lidar_frame),
                    "camera_frame": item.get("camera_frame", self.camera_frame),
                    "error": item.get("error_px"),
                }
                self.pairs.append(pair)
            self.next_pair_index = max([p["index"] for p in self.pairs], default=-1) + 1
            loaded_tf = self.load_result_tf_from_data(data, path)
            self.pairs_changed.emit()
            self.publish_markers()
            suffix = " and calibration result TF." if loaded_tf else "."
            return True, f"Loaded {len(self.pairs)} pairs{suffix}"
        except Exception as exc:
            return False, str(exc)

    def load_result_tf_from_data(self, data: Dict, path: str) -> bool:
        result = data.get("result") or {}
        matrix = result.get("T_cam_lidar") or (data.get("tf_guess") or {}).get("T_cam_lidar")
        if matrix is None:
            return False
        T = np.array(matrix, dtype=np.float64).reshape(4, 4)
        self.latest_T_cam_lidar = T
        self.tf_guess_source = f"loaded result: {os.path.basename(path)}"
        self.result_path = path
        reproj = result.get("reprojection") or {}
        if reproj:
            self.last_result_summary = (
                f"Loaded {os.path.basename(path)}: "
                f"RMS={float(reproj.get('rms_px', 0.0)):.3f}px, "
                f"p95={float(reproj.get('p95_px', 0.0)):.3f}px"
            )
        else:
            self.last_result_summary = f"Loaded calibration result TF from {path}"
        self.republish_current_frame()
        self.tf_guess_changed.emit()
        return True

    def export_data(self, include_result=True) -> Dict:
        model = self.active_camera_model
        tf_values = self.current_static_tf_values()
        return {
            "version": 1,
            "bag_path": self.args.bag,
            "topics": {
                "lidar": self.bag_data["cloud_topic"],
                "image": self.bag_data["image_topic"],
                "camera_info": self.bag_data["camera_info_topic"],
            },
            "sync": {
                "reference": "cloud",
                "tolerance_sec": self.args.sync_tolerance,
                "frame_step": self.args.frame_step,
                "target_sync_hz": self.args.target_sync_hz,
                "max_sync_frames": self.args.max_sync_frames,
                "valid_count": len(self.valid_frames),
                "display_count": len(self.display_frames),
            },
            "tf_guess": None if tf_values is None else {
                "source": self.tf_guess_source,
                "convention": "static_transform_publisher x y z yaw pitch roll parent=lidar child=camera",
                "lidar_frame": self.lidar_frame,
                "camera_frame": self.camera_frame,
                "x_y_z_yaw_pitch_roll": [float(v) for v in tf_values],
                "T_cam_lidar": None if self.latest_T_cam_lidar is None else self.latest_T_cam_lidar.reshape(-1).tolist(),
            },
            "camera": None if model is None else {
                "source": model.source,
                "frame_id": model.frame_id,
                "width": model.width,
                "height": model.height,
                "distortion_model": model.distortion_model,
                "K": model.K.reshape(-1).tolist(),
                "D": model.D.reshape(-1).tolist(),
                "P": model.P.reshape(-1).tolist(),
            },
            "pairs": [
                {
                    "index": int(p["index"]),
                    "sync_id": int(p["sync_id"]),
                    "lidar_frame": p["lidar_frame"],
                    "camera_frame": p["camera_frame"],
                    "pt3d_lidar": np.asarray(p["pt3d"], dtype=float).tolist(),
                    "pt2d_raw": None if p["pt2d"] is None else np.asarray(p["pt2d"], dtype=float).tolist(),
                    "error_px": p.get("error"),
                }
                for p in self.pairs
            ],
        }

    def initial_guess(self):
        self.ensure_tf_guess()
        if self.latest_T_cam_lidar is not None:
            rvec, _ = cv2.Rodrigues(self.latest_T_cam_lidar[:3, :3])
            return rvec.reshape(3), self.latest_T_cam_lidar[:3, 3].reshape(3)
        return np.zeros(3), np.zeros(3)

    def compute_reprojection_errors(self, T_cam_lidar: np.ndarray, pairs: List[Dict]) -> np.ndarray:
        model = self.active_camera_model
        if model is None:
            raise RuntimeError("No active camera model.")
        obj = np.array([p["pt3d"] for p in pairs], dtype=np.float64).reshape(-1, 1, 3)
        img = np.array([p["pt2d"] for p in pairs], dtype=np.float64).reshape(-1, 2)
        rvec, _ = cv2.Rodrigues(T_cam_lidar[:3, :3])
        tvec = T_cam_lidar[:3, 3].reshape(3, 1)

        if model.distortion_model.lower() in ("equidistant", "fisheye"):
            D = np.zeros(4, dtype=np.float64)
            D[:min(4, model.D.size)] = model.D[:min(4, model.D.size)]
            proj, _ = cv2.fisheye.projectPoints(obj, rvec, tvec, model.K, D.reshape(4, 1))
        else:
            proj, _ = cv2.projectPoints(obj, rvec, tvec, model.K, model.D)
        return np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)

    def run_calibration(self, save_path: Optional[str] = None):
        complete = [p for p in self.pairs if p["pt2d"] is not None]
        if len(complete) < 6:
            return False, "Need at least 6 complete 3D-2D pairs."
        if self.active_camera_model is None:
            return False, "No active camera model. Select a frame first."

        model = self.active_camera_model
        obj = np.array([p["pt3d"] for p in complete], dtype=np.float64).reshape(-1, 1, 3)
        img = np.array([p["pt2d"] for p in complete], dtype=np.float64).reshape(-1, 1, 2)
        r0, t0 = self.initial_guess()
        x0 = np.concatenate([r0, t0])

        if model.distortion_model.lower() in ("equidistant", "fisheye"):
            D = np.zeros(4, dtype=np.float64)
            D[:min(4, model.D.size)] = model.D[:min(4, model.D.size)]

            def project(x):
                pts, _ = cv2.fisheye.projectPoints(obj, x[:3].reshape(3, 1), x[3:].reshape(3, 1), model.K, D.reshape(4, 1))
                return pts.reshape(-1, 2)
        else:
            D = model.D

            def project(x):
                pts, _ = cv2.projectPoints(obj, x[:3].reshape(3, 1), x[3:].reshape(3, 1), model.K, D)
                return pts.reshape(-1, 2)

        def residual(x):
            return (project(x) - img.reshape(-1, 2)).reshape(-1)

        result = least_squares(residual, x0, loss="soft_l1", f_scale=3.0, max_nfev=2000)
        rvec = result.x[:3].reshape(3, 1)
        tvec = result.x[3:].reshape(3, 1)
        R, _ = cv2.Rodrigues(rvec)
        T_cam_lidar = np.eye(4, dtype=np.float64)
        T_cam_lidar[:3, :3] = R
        T_cam_lidar[:3, 3] = tvec.reshape(3)
        self.latest_T_cam_lidar = T_cam_lidar
        self.tf_guess_source = "calibration result"
        self.republish_current_frame()

        reproj = project(result.x)
        err = np.linalg.norm(reproj - img.reshape(-1, 2), axis=1)
        for p, e in zip(complete, err):
            p["error"] = float(e)
        stats = {
            "count": len(complete),
            "rms_px": float(np.sqrt(np.mean(err ** 2))),
            "mean_px": float(np.mean(err)),
            "p95_px": float(np.percentile(err, 95)),
            "max_px": float(np.max(err)),
        }

        T_lidar_cam = np.linalg.inv(T_cam_lidar)
        q = [float(v) for v in tf.transformations.quaternion_from_matrix(T_lidar_cam)]
        roll, pitch, yaw = [float(v) for v in tf.transformations.euler_from_matrix(T_lidar_cam, axes="sxyz")]
        t = T_lidar_cam[:3, 3]
        static_args = (
            f"{t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
            f"{yaw:.6f} {pitch:.6f} {roll:.6f} "
            f"{self.lidar_frame} {self.camera_frame}"
        )
        rospy.loginfo("[PairTool] static_transform_publisher args:")
        rospy.loginfo(static_args)

        data = self.export_data()
        data["result"] = {
            "T_cam_lidar": T_cam_lidar.reshape(-1).tolist(),
            "T_lidar_cam": T_lidar_cam.reshape(-1).tolist(),
            "translation_lidar_to_camera_parent_lidar": [float(v) for v in t],
            "quaternion_lidar_to_camera_parent_lidar": q,
            "rpy_lidar_to_camera_parent_lidar": [float(roll), float(pitch), float(yaw)],
            "static_transform_publisher_args": static_args,
            "reprojection": stats,
        }
        if save_path:
            save_path = os.path.abspath(os.path.expanduser(save_path))
            try:
                os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
                with open(save_path, "w") as f:
                    yaml.safe_dump(data, f, sort_keys=False)
                self.result_path = save_path
                self.last_result_summary = (
                    f"Saved {len(complete)} pairs to {save_path}\n"
                    f"RMS={stats['rms_px']:.3f}px, p95={stats['p95_px']:.3f}px"
                )
            except Exception as exc:
                return False, f"Calibration solved, but saving result YAML failed: {exc}\n{static_args}"
        self.pairs_changed.emit()
        self.tf_guess_changed.emit()
        return True, (
            f"Calibration OK: RMS={stats['rms_px']:.3f}px, p95={stats['p95_px']:.3f}px\n"
            f"Saved: {self.result_path}\n{static_args}"
        )


class ImageView(QtWidgets.QGraphicsView):
    image_clicked = QtCore.pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
        self.scene_obj = QtWidgets.QGraphicsScene(self)
        self.setScene(self.scene_obj)
        self.pixmap_item = None
        self.overlays = []
        self.current_image = None
        self.panning = False
        self.last_pan = None

    def set_image(self, img_bgr: Optional[np.ndarray]):
        self.scene_obj.clear()
        self.overlays = []
        self.pixmap_item = None
        self.current_image = img_bgr
        if img_bgr is None:
            return
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = img_rgb.shape
        qimg = QtGui.QImage(img_rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
        self.pixmap_item = self.scene_obj.addPixmap(QtGui.QPixmap.fromImage(qimg))
        self.scene_obj.setSceneRect(0, 0, w, h)
        self.fitInView(self.scene_obj.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def draw_overlays(self, pairs: List[Dict], current_sync_id: Optional[int], show_ghosts: bool):
        if self.pixmap_item is None:
            return
        for p in pairs:
            if p["pt2d"] is None:
                continue
            current = p["sync_id"] == current_sync_id
            if not current and not show_ghosts:
                continue
            u, v = p["pt2d"]
            radius = 6.0 if current else 4.0
            color = QtGui.QColor(255, 32, 32, 230) if current else QtGui.QColor(150, 150, 150, 90)
            fill = QtGui.QColor(color.red(), color.green(), color.blue(), 80 if current else 35)
            self.scene_obj.addEllipse(float(u) - radius, float(v) - radius, 2 * radius, 2 * radius,
                                      QtGui.QPen(color, 2), QtGui.QBrush(fill))
            text = self.scene_obj.addSimpleText(str(p["index"]))
            text.setBrush(QtGui.QBrush(QtGui.QColor(255, 255, 0, 230) if current else QtGui.QColor(180, 180, 180, 120)))
            text.setPos(float(u) + radius, float(v) + radius)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self.pixmap_item is not None:
            pos = self.mapToScene(event.pos())
            self.image_clicked.emit(pos.x(), pos.y())
            event.accept()
            return
        if event.button() in (QtCore.Qt.MiddleButton, QtCore.Qt.RightButton):
            self.panning = True
            self.last_pan = event.pos()
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.panning and self.last_pan is not None:
            delta = event.pos() - self.last_pan
            self.last_pan = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() in (QtCore.Qt.MiddleButton, QtCore.Qt.RightButton):
            self.panning = False
            self.last_pan = None
            self.setCursor(QtCore.Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event):
        self.scale(1.25 if event.angleDelta().y() > 0 else 0.8, 1.25 if event.angleDelta().y() > 0 else 0.8)


class CalibUI(QtWidgets.QWidget):
    def __init__(self, backend: CalibrationBackend):
        super().__init__()
        self.backend = backend
        self._updating_tf_editor = False
        self.setWindowTitle("Fisheye Lidar-Camera Pair Tool")
        self.backend.frame_changed.connect(self.on_frame_changed)
        self.backend.pairs_changed.connect(self.refresh_all)
        self.backend.tf_guess_changed.connect(self.refresh_tf_editor)
        self.build_ui()
        self.refresh_frames()
        self.refresh_result_info()
        if self.backend.current_sync_id is not None:
            self.backend.load_frame(self.backend.current_sync_id)

    def build_ui(self):
        root = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QVBoxLayout()
        right = QtWidgets.QVBoxLayout()
        root.addLayout(left, 3)
        root.addLayout(right, 2)

        controls = QtWidgets.QHBoxLayout()
        self.btn_prev = QtWidgets.QPushButton("Prev")
        self.btn_next = QtWidgets.QPushButton("Next")
        self.chk_all_frames = QtWidgets.QCheckBox("Show all sync frames")
        self.chk_ghost_pixels = QtWidgets.QCheckBox("Ghost all pixels")
        self.chk_all_pairs = QtWidgets.QCheckBox("Show all pairs")
        controls.addWidget(self.btn_prev)
        controls.addWidget(self.btn_next)
        controls.addWidget(self.chk_all_frames)
        controls.addWidget(self.chk_ghost_pixels)
        controls.addWidget(self.chk_all_pairs)
        left.addLayout(controls)

        self.tabs = QtWidgets.QTabWidget()
        self.raw_view = ImageView()
        self.rect_view = ImageView()
        self.tf_tab = self.build_tf_tab()
        self.tabs.addTab(self.raw_view, "Raw image")
        self.tabs.addTab(self.rect_view, "Rectified preview")
        self.tabs.addTab(self.tf_tab, "TF guess")
        left.addWidget(self.tabs)

        self.frame_table = QtWidgets.QTableWidget(0, 6)
        self.frame_table.setHorizontalHeaderLabels(["sync_id", "cloud t", "image t", "dt ms", "pairs", "valid"])
        self.frame_table.horizontalHeader().setStretchLastSection(True)
        right.addWidget(QtWidgets.QLabel("Synchronized frames"))
        right.addWidget(self.frame_table, 2)

        self.pair_table = QtWidgets.QTableWidget(0, 8)
        self.pair_table.setHorizontalHeaderLabels(["idx", "sync", "X", "Y", "Z", "u", "v", "err"])
        self.pair_table.horizontalHeader().setStretchLastSection(True)
        right.addWidget(QtWidgets.QLabel("Pairs"))
        right.addWidget(self.pair_table, 2)

        result_box = QtWidgets.QGroupBox("Calibration result YAML")
        result_layout = QtWidgets.QVBoxLayout(result_box)
        self.result_path_label = QtWidgets.QLabel("")
        self.result_path_label.setWordWrap(True)
        self.result_status_label = QtWidgets.QLabel("")
        self.result_status_label.setWordWrap(True)
        result_layout.addWidget(self.result_path_label)
        result_layout.addWidget(self.result_status_label)
        right.addWidget(result_box)

        buttons = QtWidgets.QHBoxLayout()
        self.btn_remove = QtWidgets.QPushButton("Remove")
        self.btn_save = QtWidgets.QPushButton("Save pairs")
        self.btn_load = QtWidgets.QPushButton("Load pairs")
        self.btn_calib = QtWidgets.QPushButton("Run calibration")
        self.btn_copy_tf = QtWidgets.QPushButton("Copy static TF")
        buttons.addWidget(self.btn_remove)
        buttons.addWidget(self.btn_save)
        buttons.addWidget(self.btn_load)
        buttons.addWidget(self.btn_calib)
        buttons.addWidget(self.btn_copy_tf)
        right.addLayout(buttons)

        self.raw_view.image_clicked.connect(self.on_image_clicked)
        self.frame_table.itemSelectionChanged.connect(self.on_frame_selected)
        self.btn_prev.clicked.connect(self.prev_frame)
        self.btn_next.clicked.connect(self.next_frame)
        self.chk_all_frames.stateChanged.connect(self.on_toggle_all_frames)
        self.chk_ghost_pixels.stateChanged.connect(self.refresh_images)
        self.chk_all_pairs.stateChanged.connect(self.refresh_pairs)
        self.btn_remove.clicked.connect(self.remove_selected_pair)
        self.btn_save.clicked.connect(self.save_pairs)
        self.btn_load.clicked.connect(self.load_pairs)
        self.btn_calib.clicked.connect(self.run_calibration)
        self.btn_copy_tf.clicked.connect(self.copy_static_tf)

    def build_tf_tab(self):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        info = QtWidgets.QLabel(
            "Editable initial guess, shown as ROS static_transform_publisher args:\n"
            "x y z yaw pitch roll lidar_frame camera_frame"
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        grid = QtWidgets.QGridLayout()
        self.tf_spin = {}
        specs = [
            ("x", "x [m]", -1000.0, 1000.0, 0.001, 4),
            ("y", "y [m]", -1000.0, 1000.0, 0.001, 4),
            ("z", "z [m]", -1000.0, 1000.0, 0.001, 4),
            ("yaw", "yaw [deg]", -360.0, 360.0, 0.05, 4),
            ("pitch", "pitch [deg]", -360.0, 360.0, 0.05, 4),
            ("roll", "roll [deg]", -360.0, 360.0, 0.05, 4),
        ]
        for row, (key, label, min_v, max_v, step, decimals) in enumerate(specs):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(min_v, max_v)
            spin.setDecimals(decimals)
            spin.setSingleStep(step)
            spin.setKeyboardTracking(False)
            self.tf_spin[key] = spin
            grid.addWidget(QtWidgets.QLabel(label), row, 0)
            grid.addWidget(spin, row, 1)
        layout.addLayout(grid)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_apply_tf = QtWidgets.QPushButton("Apply TF guess")
        self.btn_reset_tf = QtWidgets.QPushButton("Reset from bag TF")
        self.btn_eval_tf = QtWidgets.QPushButton("Evaluate current TF")
        btn_row.addWidget(self.btn_apply_tf)
        btn_row.addWidget(self.btn_reset_tf)
        btn_row.addWidget(self.btn_eval_tf)
        layout.addLayout(btn_row)

        self.tf_source_label = QtWidgets.QLabel("Source: unset")
        layout.addWidget(self.tf_source_label)

        self.tf_static_text = QtWidgets.QPlainTextEdit()
        self.tf_static_text.setReadOnly(True)
        self.tf_static_text.setMaximumHeight(90)
        layout.addWidget(self.tf_static_text)

        self.tf_eval_label = QtWidgets.QLabel("")
        self.tf_eval_label.setWordWrap(True)
        layout.addWidget(self.tf_eval_label)
        layout.addStretch(1)

        self.btn_apply_tf.clicked.connect(self.apply_tf_editor)
        self.btn_reset_tf.clicked.connect(self.reset_tf_from_bag)
        self.btn_eval_tf.clicked.connect(self.evaluate_current_tf)
        return widget

    def refresh_all(self):
        self.refresh_frames()
        self.refresh_pairs()
        self.refresh_images()
        self.refresh_tf_editor()
        self.refresh_result_info()

    def refresh_result_info(self):
        if not hasattr(self, "result_path_label"):
            return
        self.result_path_label.setText(f"Path: {self.backend.result_path}")
        self.result_status_label.setText(self.backend.last_result_summary)

    def refresh_frames(self):
        frames = self.backend.frames_for_ui()
        self.frame_table.blockSignals(True)
        self.frame_table.setRowCount(0)
        for frame in frames:
            row = self.frame_table.rowCount()
            self.frame_table.insertRow(row)
            vals = [
                str(frame.sync_id),
                f"{frame.cloud_time:.3f}",
                f"{frame.image_time:.3f}" if frame.image_time else "",
                f"{1000.0 * frame.image_dt:.1f}" if frame.image_dt is not None else "",
                str(self.backend.pair_count_for_frame(frame.sync_id)),
                "yes" if frame.valid else "no",
            ]
            for col, val in enumerate(vals):
                self.frame_table.setItem(row, col, QtWidgets.QTableWidgetItem(val))
            if frame.sync_id == self.backend.current_sync_id:
                self.frame_table.selectRow(row)
        self.frame_table.blockSignals(False)

    def refresh_pairs(self):
        current_only = not self.chk_all_pairs.isChecked()
        pairs = self.backend.visible_pairs(current_only)
        self.pair_table.setRowCount(0)
        for p in pairs:
            row = self.pair_table.rowCount()
            self.pair_table.insertRow(row)
            pt = p["pt3d"]
            uv = p["pt2d"]
            vals = [
                str(p["index"]), str(p["sync_id"]),
                f"{pt[0]:.3f}", f"{pt[1]:.3f}", f"{pt[2]:.3f}",
                "" if uv is None else f"{uv[0]:.1f}",
                "" if uv is None else f"{uv[1]:.1f}",
                "" if p.get("error") is None else f"{p['error']:.2f}",
            ]
            for col, val in enumerate(vals):
                self.pair_table.setItem(row, col, QtWidgets.QTableWidgetItem(val))

    def refresh_images(self):
        if self.backend.current_raw_image is not None:
            self.raw_view.set_image(self.backend.current_raw_image)
            self.raw_view.draw_overlays(self.backend.pairs, self.backend.current_sync_id, self.chk_ghost_pixels.isChecked())
        if self.backend.current_rect_image is not None:
            self.rect_view.set_image(self.backend.current_rect_image)

    def on_frame_changed(self, raw, rect):
        self.refresh_images()
        self.refresh_pairs()
        self.refresh_tf_editor()

    def on_frame_selected(self):
        rows = self.frame_table.selectionModel().selectedRows()
        if not rows:
            return
        item = self.frame_table.item(rows[0].row(), 0)
        if item:
            self.backend.load_frame(int(item.text()))

    def current_frame_row(self):
        rows = self.frame_table.selectionModel().selectedRows()
        return rows[0].row() if rows else -1

    def prev_frame(self):
        row = max(0, self.current_frame_row() - 1)
        self.frame_table.selectRow(row)

    def next_frame(self):
        row = min(self.frame_table.rowCount() - 1, self.current_frame_row() + 1)
        self.frame_table.selectRow(row)

    def on_toggle_all_frames(self):
        self.backend.set_show_all_frames(self.chk_all_frames.isChecked())
        self.refresh_frames()

    def on_image_clicked(self, u, v):
        pair_index = None
        rows = self.pair_table.selectionModel().selectedRows()
        if rows:
            item = self.pair_table.item(rows[0].row(), 0)
            if item:
                pair_index = int(item.text())
        if not self.backend.assign_pixel(pair_index, u, v):
            QtWidgets.QMessageBox.warning(self, "No 3D point", "Pick a 3D point in RViz first.")

    def remove_selected_pair(self):
        rows = self.pair_table.selectionModel().selectedRows()
        for row in sorted([r.row() for r in rows], reverse=True):
            item = self.pair_table.item(row, 0)
            if item:
                self.backend.remove_pair(int(item.text()))

    def save_pairs(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save pairs", "", "YAML (*.yaml *.yml);;JSON (*.json)")
        if path:
            ok, msg = self.backend.save_pairs(path)
            (QtWidgets.QMessageBox.information if ok else QtWidgets.QMessageBox.warning)(self, "Save pairs", msg)

    def load_pairs(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load pairs", "", "YAML/JSON (*.yaml *.yml *.json)")
        if path:
            ok, msg = self.backend.load_pairs(path)
            self.refresh_result_info()
            (QtWidgets.QMessageBox.information if ok else QtWidgets.QMessageBox.warning)(self, "Load pairs", msg)

    def refresh_tf_editor(self):
        if not hasattr(self, "tf_spin"):
            return
        vals = self.backend.current_static_tf_values()
        self._updating_tf_editor = True
        try:
            if vals is not None:
                x, y, z, yaw, pitch, roll = vals
                self.tf_spin["x"].setValue(x)
                self.tf_spin["y"].setValue(y)
                self.tf_spin["z"].setValue(z)
                self.tf_spin["yaw"].setValue(math.degrees(yaw))
                self.tf_spin["pitch"].setValue(math.degrees(pitch))
                self.tf_spin["roll"].setValue(math.degrees(roll))
            self.tf_source_label.setText(f"Source: {self.backend.tf_guess_source}")
            self.tf_static_text.setPlainText(self.backend.current_static_tf_string())
        finally:
            self._updating_tf_editor = False

    def apply_tf_editor(self):
        if self._updating_tf_editor:
            return
        x = self.tf_spin["x"].value()
        y = self.tf_spin["y"].value()
        z = self.tf_spin["z"].value()
        yaw = math.radians(self.tf_spin["yaw"].value())
        pitch = math.radians(self.tf_spin["pitch"].value())
        roll = math.radians(self.tf_spin["roll"].value())
        self.backend.set_static_tf_values(x, y, z, yaw, pitch, roll, "manual UI")
        self.tf_eval_label.setText("Applied manual TF guess and broadcasting it live.")

    def reset_tf_from_bag(self):
        ok, msg = self.backend.reset_tf_guess_from_bag()
        self.tf_eval_label.setText(msg)
        if not ok:
            QtWidgets.QMessageBox.warning(self, "TF guess", msg)

    def evaluate_current_tf(self):
        self.apply_tf_editor()
        ok, msg = self.backend.evaluate_current_tf()
        self.tf_eval_label.setText(msg)
        if not ok:
            QtWidgets.QMessageBox.warning(self, "TF evaluation", msg)

    def run_calibration(self):
        self.apply_tf_editor()
        default = self.backend.result_path
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save calibration result", default, "YAML (*.yaml *.yml)")
        if not path:
            return
        ok, msg = self.backend.run_calibration(path)
        self.refresh_result_info()
        (QtWidgets.QMessageBox.information if ok else QtWidgets.QMessageBox.warning)(self, "Calibration", msg)

    def copy_static_tf(self):
        text = self.backend.current_static_tf_string()
        QtWidgets.QApplication.clipboard().setText(text)
        self.result_status_label.setText("Copied static_transform_publisher args to clipboard.")


def parse_args():
    parser = argparse.ArgumentParser(description="Multi-timestamp fisheye lidar-camera pair selection tool")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--camera-yaml", default="")
    parser.add_argument("--lidar-topic", default="")
    parser.add_argument("--image-topic", default="")
    parser.add_argument("--camera-info-topic", default="")
    parser.add_argument("--sync-tolerance", type=float, default=0.05)
    parser.add_argument("--frame-step", type=int, default=10)
    parser.add_argument("--target-sync-hz", type=float, default=None)
    parser.add_argument("--max-sync-frames", type=int, default=None)
    parser.add_argument(
        "--result-yaml",
        default="",
        help=(
            "Default path for calibration result YAML. If omitted, the tool saves next to "
            "--camera-yaml when provided, otherwise next to the bag."
        ),
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    args = parse_args()
    args.bag = os.path.expanduser(args.bag)
    args.camera_yaml = os.path.expanduser(args.camera_yaml) if args.camera_yaml else ""
    args.result_yaml = os.path.expanduser(args.result_yaml) if args.result_yaml else ""
    rospy.init_node("fisheye_lidar_cam_pair_tool", anonymous=True, disable_signals=True)
    bag_data = read_bag_data(args)
    all_frames, valid_frames = build_sync_frames(
        bag_data["cloud_cache"], bag_data["image_cache"], bag_data["cam_info_cache"], args.sync_tolerance
    )
    display_frames = downsample_frames(valid_frames, args.frame_step, args.target_sync_hz, args.max_sync_frames)
    rospy.loginfo(
        f"[PairTool] topics: lidar={bag_data['cloud_topic']}, image={bag_data['image_topic']}, "
        f"camera_info={bag_data['camera_info_topic']}"
    )
    rospy.loginfo(f"[PairTool] sync frames: all={len(all_frames)}, valid={len(valid_frames)}, display={len(display_frames)}")
    if not display_frames:
        raise RuntimeError("No synchronized frames found. Check topics and sync tolerance.")

    app = QtWidgets.QApplication(sys.argv)
    backend = CalibrationBackend(args, bag_data, all_frames, valid_frames, display_frames)
    ui = CalibUI(backend)
    ui.resize(1500, 850)
    ui.show()
    try:
        sys.exit(app.exec_())
    finally:
        backend.stop_tf.set()


if __name__ == "__main__":
    main()
