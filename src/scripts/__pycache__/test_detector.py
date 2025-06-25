#!/usr/bin/env python

import cv2
import os
from inference_sdk import InferenceHTTPClient

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="ebkmiDQxyVJR0shdty5T"
)

# Read image using OpenCV
img = cv2.imread('src/scripts/hammer.jpg')

# Save image temporarily
temp_path = 'temp_image.jpg'
cv2.imwrite(temp_path, img)

# Infer from temporary image
result = CLIENT.infer(temp_path, model_id="mechanical-tools-10000/3")
print(result)

# Delete temporary image
os.remove(temp_path)