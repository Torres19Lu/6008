
from typing import Any, Optional

import numpy as np
import open_clip
import torch
import torch.nn.functional as F
from PIL import Image

from .server_wrapper import ServerMixin, host_model, str_to_image


class CLIPITM:
    def __init__(self, device: Optional[Any] = None) -> None:
        if device is None:
            device = torch.device(
                "cuda" if torch.cuda.is_available() else "cpu"
            )

        self.device = device

        self.model, _, self.preprocess = (
            open_clip.create_model_and_transforms(
                "ViT-B-32",
                pretrained="laion2b_s34b_b79k",
            )
        )

        self.tokenizer = open_clip.get_tokenizer("ViT-B-32")
        self.model = self.model.to(self.device).eval()

    def cosine(self, image: np.ndarray, txt: str) -> float:
        pil_image = Image.fromarray(image)

        image_tensor = (
            self.preprocess(pil_image)
            .unsqueeze(0)
            .to(self.device)
        )

        text_tensor = self.tokenizer([txt]).to(self.device)

        with torch.inference_mode():
            image_features = self.model.encode_image(image_tensor)
            text_features = self.model.encode_text(text_tensor)

            image_features = F.normalize(image_features, dim=-1)
            text_features = F.normalize(text_features, dim=-1)

            score = image_features @ text_features.T

        return float(score.item())


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=12182)
    args = parser.parse_args()

    class CLIPITMServer(ServerMixin, CLIPITM):
        def process_payload(self, payload: dict) -> dict:
            image = str_to_image(payload["image"])
            score = self.cosine(image, payload["txt"])
            return {"response": score}

    print("Loading CLIP model...")
    model = CLIPITMServer()
    print("CLIP model loaded.")
    print(f"Hosting on port {args.port}...")

    host_model(model, name="blip2itm", port=args.port)
