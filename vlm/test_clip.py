import cv2
from vlfm.vlm.server_wrapper import send_request

image_path = "/mnt/c/Users/Lenovo/Downloads/chair.jpg"
image = cv2.imread(image_path)

if image is None:
    raise RuntimeError("图片读取失败")

texts = [
    "a photo of a chair",
    "a photo of a dog",
    "a photo of a bed",
]

for text in texts:
    result = send_request(
        "http://localhost:12182/blip2itm",
        image=image,
        txt=text,
    )
    print(text, "=>", result)
