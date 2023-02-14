import cv2
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
