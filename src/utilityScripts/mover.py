import shutil
from glob import glob
import cv2
import os

emotions = ["happy", "neutral"]
dir = "training"
emotion = "happy"

images = glob(f"{dir}/{emotion}/*.jpg")
count = 0

for img_path in images:
    image = cv2.imread(img_path)
    cv2.imshow('Safe?', image)
    print(img_path)
    print("1 - keep, 2 - move to manual")
    pressed = cv2.waitKey(0)

    parts = img_path.split("/")
    img_name = parts[len(parts) - 1]
    if img_name[0] == 'x':
        img_name = img_name[1:]

    if pressed == 49: # 1
        os.replace(img_path, f"verified/{emotion}/{img_name}")
    elif pressed == 50:
        os.replace(f"uncropped/{emotion}/{img_name}", f"manual/{emotion}/{img_name}")
        os.replace(img_path, f"garbage/{emotion}/{img_name}")
    
    count+=1
    if count == 100:
        break