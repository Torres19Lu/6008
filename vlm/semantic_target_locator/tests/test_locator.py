import json
import unittest
from pathlib import Path

from contracts import Status, TargetQuery
from geometry import MatrixTransformProvider, RegisteredPointDepthProvider
from locator import LocatorConfig, SemanticTargetLocator
from replay_adapters import CollectingSink, ReplayFrameSource, ReplayVlmBackend, load_replay


HERE = Path(__file__).resolve().parent


def build_locator(data, sink, **overrides):
    config = LocatorConfig(max_frames=len(data["frames"]), **overrides)
    return SemanticTargetLocator(
        frames=ReplayFrameSource(data["frames"]),
        vlm=ReplayVlmBackend(),
        depth=RegisteredPointDepthProvider(min_points=3),
        transforms=MatrixTransformProvider(),
        sink=sink,
        config=config,
    )


class LocatorTest(unittest.TestCase):
    def setUp(self):
        self.data = load_replay(HERE / "fixtures" / "mock_replay.json")

    def test_replay_locks_map_coordinate(self):
        sink = CollectingSink()
        estimate = build_locator(self.data, sink).locate(
            TargetQuery(self.data["target_text"], self.data["base_class"])
        )
        self.assertIsNotNone(estimate)
        self.assertEqual(estimate.to_dict()["position"], {"x": 2.732, "y": 3.0, "z": 0.42})
        self.assertEqual(estimate.observation_count, 3)
        self.assertEqual(sink.updates[-1].status, Status.TARGET_LOCKED)

    def test_no_detection_times_out_without_fake_coordinate(self):
        data = json.loads(json.dumps(self.data))
        for frame in data["frames"]:
            frame["detections"] = []
        sink = CollectingSink()
        estimate = build_locator(data, sink).locate(
            TargetQuery(data["target_text"], data["base_class"])
        )
        self.assertIsNone(estimate)
        self.assertEqual(sink.updates[-1].status, Status.TIMEOUT)
        self.assertTrue(all(update.target is None for update in sink.updates))

    def test_missing_transform_is_reported(self):
        data = json.loads(json.dumps(self.data))
        for frame in data["frames"]:
            frame.pop("sensor_to_map")
        sink = CollectingSink()
        estimate = build_locator(data, sink).locate(
            TargetQuery(data["target_text"], data["base_class"])
        )
        self.assertIsNone(estimate)
        self.assertIn(Status.TF_UNAVAILABLE, [update.status for update in sink.updates])


if __name__ == "__main__":
    unittest.main()
