import cv2
import matplotlib.pyplot as plt
from glob import glob
import os
import dlib

emotions = ["happy", "angry", "sad", "neutral", "other"]
imageType = "training"

# DLIB HoG
hog_detector = dlib.get_frontal_face_detector()

for emotion in emotions:
	allImages = glob(f"{imageType}/{emotion}/*.jpg")
	total = 0
	success = 0

	for imagePath in allImages:
		name = imagePath.split("/")[2]
		image = cv2.imread(imagePath)
		gray_frame= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		faces = hog_detector(gray_frame)
		count = len(faces)
		if count == 1:
			face = faces[0]
			print(face)
			x, y, w, h = face.left(), face.top(), face.width(), face.height()
			face_image = image[y:y+h, x:x+w]
			print(f"training/{emotion}/{name}")
			# cv2.imwrite(f"training/{emotion}/{name}", face_image)
		else:
			print(f"manual/{emotion}/{name}")
			# cv2.imwrite(f"manual/{emotion}/{name}", image)

	print("-----------------------------")
	print(f"Found faces in {success}/{total} images in {emotion} set")
	print("-----------------------------")