"""ROS1 adapters for the stable locator interfaces.

Imports of ROS and model dependencies are intentionally delayed until construction,
so the core and replay tests remain runnable on a machine without ROS.
"""

from __future__ import annotations

import json
import math
import queue
import statistics
from dataclasses import dataclass
from typing import Any, Optional, Sequence, Tuple

try:
    from .contracts import (
        Detection2D, Point3, SynchronizedFrame, TargetQuery, Update,
    )
except ImportError:
    from contracts import Detection2D, Point3, SynchronizedFrame, TargetQuery, Update


@dataclass(frozen=True)
class RosTopics:
    rgb: str = "/camera/color/image_raw"
    depth: str = "/camera/aligned_depth_to_color/image_raw"
    camera_info: str = "/camera/color/camera_info"
    query: str = "/semantic/target_query"
    status: str = "/semantic/target_status"
    map_frame: str = "map"


class RosSynchronizedFrameSource:
    def __init__(self, topics: RosTopics, queue_size: int = 5, slop_s: float = 0.05) -> None:
        import message_filters
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image

        self._queue: "queue.Queue[SynchronizedFrame]" = queue.Queue(maxsize=1)
        self._bridge = CvBridge()
        rgb_sub = message_filters.Subscriber(topics.rgb, Image)
        depth_sub = message_filters.Subscriber(topics.depth, Image)
        info_sub = message_filters.Subscriber(topics.camera_info, CameraInfo)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size, slop_s
        )
        self._sync.registerCallback(self._callback)

    def _callback(self, rgb_msg: Any, depth_msg: Any, info_msg: Any) -> None:
        frame = SynchronizedFrame(
            stamp=float(rgb_msg.header.stamp.to_sec()),
            source_frame=str(rgb_msg.header.frame_id),
            width=int(rgb_msg.width),
            height=int(rgb_msg.height),
            rgb=self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8"),
            depth=self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough"),
            camera_info=info_msg,
            metadata={"depth_encoding": str(depth_msg.encoding)},
        )
        try:
            self._queue.get_nowait()
        except queue.Empty:
            pass
        self._queue.put_nowait(frame)

    def next_frame(self, timeout_s: float) -> Optional[SynchronizedFrame]:
        try:
            return self._queue.get(timeout=timeout_s)
        except queue.Empty:
            return None


class RosVisionLanguageBackend:
    """Adapter around the existing YOLO and CLIP HTTP sidecars."""

    def __init__(self, yolo_port: int = 12184, clip_port: int = 12182) -> None:
        from vlfm.vlm.yolov7 import YOLOv7Client

        self._detector = YOLOv7Client(port=yolo_port)
        self._clip_url = "http://localhost:{}/blip2itm".format(clip_port)

    def detect(self, frame: SynchronizedFrame, query: TargetQuery) -> Sequence[Detection2D]:
        import cv2
        from vlfm.vlm.server_wrapper import send_request

        detections = self._detector.predict(frame.rgb)
        detections.filter_by_class([query.base_class])
        detections.filter_by_conf(0.01)  # Core applies the configured threshold.
        results = []
        for index in range(detections.num_detections):
            x1, y1, x2, y2 = (float(value) for value in detections.boxes[index].tolist())
            px1 = max(0, min(frame.width - 1, int(x1 * frame.width)))
            py1 = max(0, min(frame.height - 1, int(y1 * frame.height)))
            px2 = max(px1 + 1, min(frame.width, int(x2 * frame.width)))
            py2 = max(py1 + 1, min(frame.height, int(y2 * frame.height)))
            crop_rgb = cv2.cvtColor(frame.rgb[py1:py2, px1:px2], cv2.COLOR_BGR2RGB)
            response = send_request(
                self._clip_url,
                image=crop_rgb,
                txt="a photo of a {}".format(query.target_text),
            )
            results.append(Detection2D(
                class_name=query.base_class,
                box=(x1, y1, x2, y2),
                detector_confidence=float(detections.logits[index]),
                text_similarity=float(response["response"]),
            ))
        return results


class AlignedDepthProvider:
    """Back-project aligned depth pixels inside a detection box."""

    def __init__(
        self,
        depth_scale_to_m: float = 0.001,
        min_depth_m: float = 0.2,
        max_depth_m: float = 8.0,
        pixel_stride: int = 4,
        min_points: int = 20,
    ) -> None:
        self.depth_scale_to_m = depth_scale_to_m
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.pixel_stride = pixel_stride
        self.min_points = min_points

    def position_in_sensor(
        self, frame: SynchronizedFrame, detection: Detection2D
    ) -> Tuple[Point3, int]:
        if frame.depth is None or frame.camera_info is None:
            raise ValueError("depth or CameraInfo is missing")
        fx, fy = float(frame.camera_info.K[0]), float(frame.camera_info.K[4])
        cx, cy = float(frame.camera_info.K[2]), float(frame.camera_info.K[5])
        x1, y1, x2, y2 = detection.box
        px1, px2 = max(0, int(x1 * frame.width)), min(frame.width, int(x2 * frame.width))
        py1, py2 = max(0, int(y1 * frame.height)), min(frame.height, int(y2 * frame.height))
        points = []
        for v in range(py1, py2, self.pixel_stride):
            for u in range(px1, px2, self.pixel_stride):
                depth_m = float(frame.depth[v, u]) * self.depth_scale_to_m
                if not math.isfinite(depth_m) or not self.min_depth_m <= depth_m <= self.max_depth_m:
                    continue
                points.append(((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m))
        if len(points) < self.min_points:
            raise ValueError(
                "not enough aligned depth points: {} < {}".format(len(points), self.min_points)
            )
        median = tuple(statistics.median(axis) for axis in zip(*points))
        return median, len(points)  # type: ignore[return-value]


class RosTfProvider:
    def __init__(self, topics: RosTopics, timeout_s: float = 0.2) -> None:
        import rospy
        import tf2_ros

        self._rospy = rospy
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._map_frame = topics.map_frame
        self._timeout_s = timeout_s

    def to_map(self, frame: SynchronizedFrame, point_sensor: Point3) -> Point3:
        transform = self._buffer.lookup_transform(
            self._map_frame,
            frame.source_frame,
            self._rospy.Time.from_sec(frame.stamp),
            self._rospy.Duration(self._timeout_s),
        ).transform
        q = transform.rotation
        p = _rotate_by_quaternion(point_sensor, (q.x, q.y, q.z, q.w))
        t = transform.translation
        return p[0] + t.x, p[1] + t.y, p[2] + t.z


def _rotate_by_quaternion(point: Point3, quaternion: Tuple[float, float, float, float]) -> Point3:
    x, y, z = point
    qx, qy, qz, qw = quaternion
    # R(q) * point, expanded to avoid another runtime dependency.
    return (
        (1 - 2 * (qy * qy + qz * qz)) * x + 2 * (qx * qy - qz * qw) * y + 2 * (qx * qz + qy * qw) * z,
        2 * (qx * qy + qz * qw) * x + (1 - 2 * (qx * qx + qz * qz)) * y + 2 * (qy * qz - qx * qw) * z,
        2 * (qx * qz - qy * qw) * x + 2 * (qy * qz + qx * qw) * y + (1 - 2 * (qx * qx + qy * qy)) * z,
    )


class RosJsonResultSink:
    """Initial wire format; replace only this adapter when g1_msgs is finalized."""

    def __init__(self, topics: RosTopics) -> None:
        import rospy
        from std_msgs.msg import String

        self._message_type = String
        self._publisher = rospy.Publisher(topics.status, String, queue_size=10, latch=True)

    def publish(self, update: Update) -> None:
        self._publisher.publish(self._message_type(data=json.dumps(update.to_dict(), ensure_ascii=False)))


class RosQuerySource:
    """Accept JSON {target_text, base_class}; plain text falls back to its last word."""

    def __init__(self, topics: RosTopics) -> None:
        import rospy
        from std_msgs.msg import String

        self._queue: "queue.Queue[TargetQuery]" = queue.Queue()
        self._subscriber = rospy.Subscriber(topics.query, String, self._callback, queue_size=1)

    def _callback(self, message: Any) -> None:
        text = str(message.data).strip()
        try:
            value = json.loads(text)
            query = TargetQuery(str(value["target_text"]), str(value["base_class"]))
        except (json.JSONDecodeError, KeyError, TypeError):
            query = TargetQuery(text, text.split()[-1])
        self._queue.put(query)

    def next_query(self, timeout_s: float = 0.5) -> Optional[TargetQuery]:
        try:
            return self._queue.get(timeout=timeout_s)
        except queue.Empty:
            return None
