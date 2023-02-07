from retinaface import RetinaFace
from deepface import DeepFace
import matplotlib.pyplot as plt
import os

img_path = os.getcwd() + "/testImages/test0.jpg"
faces = RetinaFace.extract_faces(img_path=img_path, align=True)

for face in faces:
  plt.imshow(face)
  plt.show()

from deepface import DeepFace
# https://pypi.org/project/retina-face/
# https://github.com/serengil/deepface


# Other option: https://machinelearningmastery.com/how-to-perform-face-detection-with-classical-and-deep-learning-methods-in-python-with-keras/
