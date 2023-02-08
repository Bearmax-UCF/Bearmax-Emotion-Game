# from retinaface import RetinaFace
# from deepface import DeepFace
# import matplotlib.pyplot as plt
# import os

# img_path = os.getcwd() + "/testImages/test0.jpg"
# faces = RetinaFace.extract_faces(img_path=img_path, align=True)

# for face in faces:
#   plt.imshow(face)
#   plt.show()

# https://pypi.org/project/retina-face/
# https://github.com/serengil/deepface
# Other option: https://machinelearningmastery.com/how-to-perform-face-detection-with-classical-and-deep-learning-methods-in-python-with-keras/

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
