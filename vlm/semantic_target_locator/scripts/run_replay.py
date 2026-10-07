import argparse
import json
from pathlib import Path

from contracts import TargetQuery
from geometry import MatrixTransformProvider, RegisteredPointDepthProvider
from locator import LocatorConfig, SemanticTargetLocator
from replay_adapters import CollectingSink, ReplayFrameSource, ReplayVlmBackend, load_replay


parser = argparse.ArgumentParser(description="Replay semantic target localization")
parser.add_argument("replay", type=Path)
args = parser.parse_args()

data = load_replay(args.replay)
sink = CollectingSink()
locator = SemanticTargetLocator(
    frames=ReplayFrameSource(data["frames"]),
    vlm=ReplayVlmBackend(),
    depth=RegisteredPointDepthProvider(min_points=3),
    transforms=MatrixTransformProvider(),
    sink=sink,
    config=LocatorConfig(max_frames=len(data["frames"])),
)
locator.locate(TargetQuery(data["target_text"], data["base_class"]))
print(json.dumps([update.to_dict() for update in sink.updates], ensure_ascii=False, indent=2))
