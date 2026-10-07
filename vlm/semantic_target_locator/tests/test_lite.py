import unittest

import cv2
import numpy as np

from contracts import SynchronizedFrame
from lite import (
    LiteVisionLanguageBackend, RawDetection, color_score_bgr,
    extract_color, parse_target_query,
)


class FakeDetector:
    def __init__(self, detections):
        self.detections = detections

    def detect(self, frame, base_class):
        del frame, base_class
        return self.detections


class LiteTargetTest(unittest.TestCase):
    def test_parses_chinese_color_object(self):
        query = parse_target_query("红色的椅子")
        self.assertEqual(query.target_text, "红色的椅子")
        self.assertEqual(query.base_class, "chair")
        self.assertEqual(extract_color(query.target_text), "red")

    def test_explicit_base_class_supports_unlisted_object(self):
        query = parse_target_query("蓝色工具箱", "toolbox")
        self.assertEqual(query.base_class, "toolbox")
        self.assertEqual(extract_color(query.target_text), "blue")

    def test_red_candidate_wins_without_clip(self):
        image = np.zeros((100, 300, 3), dtype=np.uint8)
        image[:, 0:100] = (0, 0, 255)      # BGR red
        image[:, 100:200] = (255, 0, 0)    # BGR blue
        image[:, 200:300] = (128, 128, 128)
        detections = [
            RawDetection("chair", (0.0, 0.0, 1 / 3, 1.0), 0.70),
            RawDetection("chair", (1 / 3, 0.0, 2 / 3, 1.0), 0.95),
            RawDetection("chair", (2 / 3, 0.0, 1.0, 1.0), 0.99),
        ]
        backend = LiteVisionLanguageBackend(FakeDetector(detections))
        frame = SynchronizedFrame(1.0, "camera", 300, 100, rgb=image)
        results = backend.detect(frame, parse_target_query("红色的椅子"))
        winner = max(results, key=lambda item: item.joint_confidence)
        self.assertEqual(winner.box, detections[0].box)
        self.assertGreater(winner.text_similarity, 0.95)

    def test_uncolored_query_uses_detector_confidence(self):
        image = np.zeros((10, 20, 3), dtype=np.uint8)
        detections = [
            RawDetection("bottle", (0.0, 0.0, 0.5, 1.0), 0.60),
            RawDetection("bottle", (0.5, 0.0, 1.0, 1.0), 0.90),
        ]
        backend = LiteVisionLanguageBackend(FakeDetector(detections))
        frame = SynchronizedFrame(1.0, "camera", 20, 10, rgb=image)
        results = backend.detect(frame, parse_target_query("瓶子"))
        winner = max(results, key=lambda item: item.joint_confidence)
        self.assertEqual(winner.box, detections[1].box)
        self.assertEqual(winner.text_similarity, 1.0)

    def test_color_score_rejects_wrong_color(self):
        red = np.full((20, 20, 3), (0, 0, 255), dtype=np.uint8)
        self.assertGreater(color_score_bgr(red, "red"), 0.99)
        self.assertLess(color_score_bgr(red, "blue"), 0.01)


if __name__ == "__main__":
    unittest.main()
