import sys
import os
import json
import cv2
from PIL import Image, ImageDraw #pip install pillow

#make sure the uw_llm library is in the same directory
from uw_llm import *

def resize_image(input_path, output_path, scale=0.25):
    with Image.open(input_path) as img:
        new_size = (int(img.width * scale), int(img.height * scale))
        resized_img = img.resize(new_size)
        resized_img.save(output_path)

identify_prompt = """
You are picture with an orange block as well as ArUco markers on it. Output a bounding box around the orange block in JSON format. Use the label "orange". Output ONLY the bounding box in the format:

  {"bbox_2d": [x0, y0, x1, y1], "label": "orange"}

"""

input_path = "."
resized_path = "."
resized_img_name = "objects_resized.JPG"
img = "objects.JPG"

"""
We need to resize the image because the GPU we are running the model on doesn't QUITE have enough memory
to run on a full resolution image of our cameras. 0.75x scale is fine. HOWEVER, this means that you need to 
move the bounding box to the right place - since all pixel coordinates are scaled by 0.75, you need to re-map
to the actual pixel coordinates before trying to pick stuff up or you will be in the wrong place.
"""

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Camera error")

# Step 1: Capture frame
ret, frame = cap.read()
if not ret:
    print("Frame error")

# Optional: Save for LLM vision
cv2.imwrite("objects.JPG", frame)

resize_image(img,resized_img_name,0.75)
resized_img_path = os.path.join(resized_path, resized_img_name)

response = generate_vision(identify_prompt, resized_img_path, fast=False)
print(response)
bbox_obj = json.loads(response.strip("`\n json"))  # Now expects a single dict, not a list

# --- Draw bounding box on the image ---
with Image.open(resized_img_path) as image:
    draw = ImageDraw.Draw(image)
    box = bbox_obj["bbox_2d"]
    label = bbox_obj["label"]
    draw.rectangle(box, outline="red", width=3)
    draw.text((box[0] + 10, box[1] + 10), label, fill="red")

    output_img_path = "boxed_" + img
    image.save(output_img_path)
    print(f"Saved final image with box to {output_img_path}")

# Save the sub-image
base_name, ext = os.path.splitext(img)
with Image.open(resized_img_path) as image:
    out_name = f"{label}_0_{base_name}{ext}"
    out_path = os.path.join(resized_path, out_name)
    region = image.crop(box)
    region.save(out_path)
    print(f"Saved {label} region to {out_path}")
