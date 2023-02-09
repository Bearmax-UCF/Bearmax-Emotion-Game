from retinaface import RetinaFace
import matplotlib.pyplot as plt
from glob import glob
import os

autisticImages = glob("autismDataset/test/autistic/*.jpg")
nonAutisticImages = glob("autismDataset/test/non_autistic/*.jpg")

total = 0
success = 0

for img_path in autisticImages:
	faces = RetinaFace.extract_faces(img_path=img_path, align=True)
	if len(faces) != 0:
		success += 1
		print(f"Found face in {img_path}")
		for face in faces:
			plt.imshow(face)
			plt.show()
	else:
		print(f"No face in {img_path}")

	total += 1

print("-----------------------------")
print(f"Found faces in {success}/{total} images of autistic children")
print("-----------------------------")

total = 0
success = 0

for img_path in nonAutisticImages:
	faces = RetinaFace.extract_faces(img_path=img_path, align=True)
	if len(faces) != 0:
		success += 1
		print(f"Found face in {img_path}")
		for face in faces:
			plt.imshow(face)
			plt.show()
	else:
		print(f"No face in {img_path}")
		
	total += 1
print("-----------------------------")
print(f"Found faces in {success}/{total} images of non_autistic children")
print("-----------------------------")

# https://pypi.org/project/retina-face/
# https://github.com/serengil/deepface
# Other option: https://machinelearningmastery.com/how-to-perform-face-detection-with-classical-and-deep-learning-methods-in-python-with-keras/

'''
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

def main():

	pipeline = rs.pipeline()
	config = rs.config()

	config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
	pipeline.start(config)

	while True:
		# Get RealSense frame first so we can guarantee we have one
		# The result is an array of two frames for one instance in time, one depth and one color
		frames = pipeline.wait_for_frames()
		color_frame = frames.get_color_frame()

		# As far as I can tell this never happens, but just safety
		if not color_frame:
			continue
		
		# Convert images to numpy arrays
		color_image = np.asanyarray(color_frame.get_data())

		# Render our findings to the screen
		cv2.imshow("color_image preview", color_image)
		if cv2.waitKey(1) == 27:
			break

	pipeline.stop()

if __name__ == '__main__':
	main()
'''