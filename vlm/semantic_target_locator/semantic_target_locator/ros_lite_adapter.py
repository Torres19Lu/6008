"""ROS1 adapters used only by the lightweight YOLO + color mode."""

from __future__ import annotations

import json
import queue
from typing import Any, Optional, Sequence

try:
    from .contracts import SynchronizedFrame, TargetQuery
    from .lite import RawDetection, parse_target_query
except ImportError:
    from contracts import SynchronizedFrame, TargetQuery
    from lite import RawDetection, parse_target_query


class YoloV7ObjectDetector:
    """Convert the existing YOLOv7 sidecar result to the lite detector contract."""

    def __init__(self, port: int = 12184) -> None:
        from vlfm.vlm.yolov7 import YOLOv7Client

        self._detector = YOLOv7Client(port=port)

    def detect(self, frame: SynchronizedFrame, base_class: str) -> Sequence[RawDetection]:
        detections = self._detector.predict(frame.rgb)
        detections.filter_by_class([base_class])
        detections.filter_by_conf(0.01)
        return [
            RawDetection(
                class_name=base_class,
                box=tuple(float(value) for value in detections.boxes[index].tolist()),
                confidence=float(detections.logits[index]),
            )
            for index in range(detections.num_detections)
        ]


class RosLiteQuerySource:
    """Accept natural fixed queries or JSON with an explicit detector class."""

    def __init__(self, query_topic: str) -> None:
        import rospy
        from std_msgs.msg import String

        self._rospy = rospy
        self._queue: "queue.Queue[TargetQuery]" = queue.Queue()
        self._subscriber = rospy.Subscriber(query_topic, String, self._callback, queue_size=1)

    def _callback(self, message: Any) -> None:
        text = str(message.data).strip()
        try:
            value = json.loads(text)
            query = parse_target_query(
                str(value["target_text"]), str(value.get("base_class", "")) or None
            )
        except (json.JSONDecodeError, TypeError):
            try:
                query = parse_target_query(text)
            except ValueError as error:
                self._rospy.logwarn("lightweight target query rejected: %s", error)
                return
        except (KeyError, ValueError) as error:
            self._rospy.logwarn("lightweight target query rejected: %s", error)
            return
        self._queue.put(query)

    def next_query(self, timeout_s: float = 0.5) -> Optional[TargetQuery]:
        try:
            return self._queue.get(timeout=timeout_s)
        except queue.Empty:
            return None
