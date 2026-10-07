#!/usr/bin/env python3
"""Realtime RGB-D semantic localizer for ROS Noetic.

The neural networks stay in the lightweight VLFM HTTP servers. This node uses
the system ROS Python, synchronizes live RGB and aligned-depth ROS images from
Gazebo or a real camera, queries the two model servers, transforms the detected
point into ``map``, and publishes a stable target pose.
"""

import base64
import json
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import message_filters
import numpy as np
import requests
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped conversions
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from vlm_sensor_utils import depth_image_to_meters


class RealtimeVLM:
    def __init__(self):
        self.target = rospy.get_param("~target", "red chair").strip().lower()
        self.base_class = rospy.get_param("~base_class", "chair").strip().lower()
        self.map_frame = rospy.get_param("~map_frame", "map")
        # Topic defaults preserve the Gazebo contract. A real camera can use a
        # namespace or remapped topics without changing this node again.
        self.rgb_topic = rospy.get_param(
            "~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info")
        # Gazebo publishes 32FC1 metres. RealSense normally publishes 16UC1
        # millimetres; this scale converts one integer unit to metres.
        self.uint16_depth_scale = float(
            rospy.get_param("~uint16_depth_scale", 0.001))
        self.yolo_url = rospy.get_param(
            "~yolo_url", "http://localhost:12184/yolov7")
        self.clip_url = rospy.get_param(
            "~clip_url", "http://localhost:12182/blip2itm")
        self.process_rate = float(rospy.get_param("~process_rate", 1.0))
        self.yolo_threshold = float(rospy.get_param("~yolo_threshold", 0.25))
        self.clip_threshold = float(rospy.get_param("~clip_threshold", 0.20))
        self.min_observations = int(rospy.get_param("~min_observations", 3))
        self.memory_size = int(rospy.get_param("~memory_size", 20))
        self.output_path = Path(
            rospy.get_param("~output", "semantic_memory_realtime.json")
        ).expanduser()

        self.bridge = CvBridge()
        self.camera_info = None
        self.observations = deque(maxlen=self.memory_size)
        self.busy = False
        self.last_process_wall = 0.0
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_pub = rospy.Publisher("/vlm/target_pose", PoseStamped, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher("/vlm/semantic_target", String, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/vlm/target_marker", Marker, queue_size=1, latch=True)
        self.image_pub = rospy.Publisher("/vlm/detection_image", Image, queue_size=1)

        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback,
            queue_size=1,
        )
        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(
            self.depth_topic, Image
        )
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.08
        )
        sync.registerCallback(self.synced_callback)
        self.sync = sync

        rospy.loginfo(
            "Realtime VLM ready: target='%s', processing %.2f Hz, RGB=%s, depth=%s",
            self.target,
            self.process_rate,
            self.rgb_topic,
            self.depth_topic,
        )

    def camera_info_callback(self, message):
        if self.camera_info is None:
            self.camera_info = {
                "fx": float(message.K[0]),
                "fy": float(message.K[4]),
                "cx": float(message.K[2]),
                "cy": float(message.K[5]),
            }
            rospy.loginfo(
                "CameraInfo received: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                self.camera_info["fx"], self.camera_info["fy"],
                self.camera_info["cx"], self.camera_info["cy"],
            )

    def synced_callback(self, rgb_message, depth_message):
        now = time.monotonic()
        interval = 1.0 / max(self.process_rate, 0.01)
        with self.lock:
            if self.busy or self.camera_info is None or now - self.last_process_wall < interval:
                return
            self.busy = True
            self.last_process_wall = now

        thread = threading.Thread(
            target=self.process_pair, args=(rgb_message, depth_message), daemon=True
        )
        thread.start()

    @staticmethod
    def encode_image(image):
        ok, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        return base64.b64encode(data.tobytes()).decode("utf-8")

    def request(self, url, payload):
        response = requests.post(url, json=payload, timeout=60)
        response.raise_for_status()
        return response.json()

    @staticmethod
    def pixel_box(box, width, height):
        x1, y1, x2, y2 = box
        return (
            max(0, min(width - 1, int(x1 * width))),
            max(0, min(height - 1, int(y1 * height))),
            max(1, min(width, int(x2 * width))),
            max(1, min(height, int(y2 * height))),
        )

    @staticmethod
    def box_depth(depth, box):
        x1, y1, x2, y2 = box
        width, height = x2 - x1, y2 - y1
        ix1, ix2 = x1 + int(width * 0.25), x2 - int(width * 0.25)
        iy1, iy2 = y1 + int(height * 0.25), y2 - int(height * 0.25)
        if ix2 <= ix1 or iy2 <= iy1:
            return None
        region = depth[iy1:iy2, ix1:ix2]
        valid = region[np.isfinite(region) & (region > 0.1) & (region < 20.0)]
        return float(np.median(valid)) if valid.size >= 20 else None

    def process_pair(self, rgb_message, depth_message):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="rgb8")
            # Use passthrough so the ROS encoding remains available. Conversion
            # to metres is explicit and works for both Gazebo and RealSense.
            raw_depth = self.bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough")
            depth = depth_image_to_meters(
                raw_depth,
                depth_message.encoding,
                uint16_scale=self.uint16_depth_scale,
            )
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            encoded = self.encode_image(bgr)

            detections = self.request(
                self.yolo_url, {"image": encoded}
            )
            candidates = []
            height, width = rgb.shape[:2]
            for box, confidence, phrase in zip(
                detections["boxes"], detections["logits"], detections["phrases"]
            ):
                if phrase != self.base_class or float(confidence) < self.yolo_threshold:
                    continue
                pixels = self.pixel_box(box, width, height)
                x1, y1, x2, y2 = pixels
                crop = rgb[y1:y2, x1:x2]
                if crop.size == 0:
                    continue
                # cv2.imencode expects BGR channel order. Without this conversion
                # red and blue are swapped before the CLIP service sees the crop.
                crop_bgr = cv2.cvtColor(crop, cv2.COLOR_RGB2BGR)
                clip = self.request(
                    self.clip_url,
                    {"image": self.encode_image(crop_bgr),
                     "txt": "a photo of a " + self.target},
                )
                similarity = float(clip["response"])
                distance = self.box_depth(depth, pixels)
                if distance is not None:
                    candidates.append((similarity, float(confidence), distance, pixels))

            if not candidates:
                rospy.logwarn_throttle(5.0, "VLM: no chair with valid depth")
                return
            similarity, confidence, distance, pixels = max(candidates, key=lambda x: x[0])
            if similarity < self.clip_threshold:
                rospy.logwarn("VLM: best CLIP score %.3f is below threshold", similarity)
                return

            x1, y1, x2, y2 = pixels
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            info = self.camera_info
            point = PointStamped()
            point.header = rgb_message.header
            point.point.x = (u - info["cx"]) * distance / info["fx"]
            point.point.y = (v - info["cy"]) * distance / info["fy"]
            point.point.z = distance
            mapped = self.tf_buffer.transform(
                point, self.map_frame, timeout=rospy.Duration(0.25)
            )
            self.observations.append(
                (mapped.point.x, mapped.point.y, mapped.point.z,
                 distance, confidence, similarity)
            )
            self.publish_result(rgb_message, rgb, pixels)
        except requests.RequestException as error:
            rospy.logerr_throttle(5.0, "VLM model service error: %s", error)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(5.0, "VLM TF unavailable: %s", error)
        except Exception as error:
            rospy.logerr_throttle(5.0, "VLM processing failed: %s", error)
        finally:
            with self.lock:
                self.busy = False

    def publish_result(self, source_message, rgb, pixels):
        values = np.asarray(self.observations, dtype=np.float64)
        count = len(values)
        position = np.median(values[:, :3], axis=0)
        result = {
            "target_text": self.target,
            "base_class": self.base_class,
            "detected": True,
            "stable": count >= self.min_observations,
            "frame_id": self.map_frame,
            "object_position": {"x": round(float(position[0]), 3),
                                "y": round(float(position[1]), 3),
                                "z": round(float(position[2]), 3)},
            "depth_m": round(float(np.median(values[:, 3])), 3),
            "yolo_confidence": round(float(np.median(values[:, 4])), 3),
            "clip_similarity": round(float(np.median(values[:, 5])), 3),
            "observation_count": count,
            "source": "live_ros_topics",
        }
        self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        self.output_path.write_text(
            json.dumps({self.target: result}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        if count >= self.min_observations:
            pose = PoseStamped()
            pose.header.stamp = source_message.header.stamp
            pose.header.frame_id = self.map_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)

            marker = Marker()
            marker.header = pose.header
            marker.ns, marker.id, marker.type, marker.action = "vlm", 0, Marker.SPHERE, Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = marker.scale.y = marker.scale.z = 0.35
            marker.color.r, marker.color.a = 1.0, 1.0
            marker.lifetime = rospy.Duration(0)
            self.marker_pub.publish(marker)

        annotated = rgb.copy()
        x1, y1, x2, y2 = pixels
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 0), 3)
        cv2.putText(annotated, self.target, (x1, max(25, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        image_message = self.bridge.cv2_to_imgmsg(annotated, encoding="rgb8")
        image_message.header = source_message.header
        self.image_pub.publish(image_message)
        rospy.loginfo(
            "VLM %s: map=(%.3f, %.3f, %.3f), depth=%.2f, observations=%d%s",
            self.target, position[0], position[1], position[2],
            np.median(values[:, 3]), count,
            " [STABLE]" if count >= self.min_observations else "",
        )


if __name__ == "__main__":
    rospy.init_node("ros_vlm_node")
    RealtimeVLM()
    rospy.spin()
