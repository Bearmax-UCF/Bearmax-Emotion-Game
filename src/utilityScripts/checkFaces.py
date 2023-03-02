import cv2
import matplotlib.pyplot as plt
from glob import glob
import os
import dlib

emotions = ["happy", "angry", "sad", "neutral", "other"]
imageType = "uncropped"

# DLIB HoG
hog_detector = dlib.get_frontal_face_detector()

for emotion in emotions:
	allImages = glob(f"{imageType}/{emotion}/*.jpg")

	for imagePath in allImages:
		name = imagePath.split("/")[2]
		image = cv2.imread(imagePath)
		gray_frame= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		faces = hog_detector(gray_frame)
		count = len(faces)
		if count == 1:
			face = faces[0]
			x, y, w, h = face.left(), face.top(), face.width(), face.height()
			face_image = image[y:y+h, x:x+w]
			try:
				cv2.imwrite(f"training/{emotion}/x{name}", face_image)
			except:
				cv2.imwrite(f"manual/{emotion}/x{name}", image)
		else:
			cv2.imwrite(f"manual/{emotion}/x{name}", image)