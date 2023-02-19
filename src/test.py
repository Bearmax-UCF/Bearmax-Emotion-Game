"""
dlib with CUDA testing
"""

import cv2
import os
import numpy as np
import dlib
import face_recognition
import time

def invalidCamera(capture):
    return capture is None or not capture.isOpened()

def run():
    for i in range(0, 5):
        capture = cv2.VideoCapture(i)

        if invalidCamera(capture):
            continue

        print(f"Found webcam at index {i}")
        break

    if invalidCamera(capture):
        raise Exception("Could not find webcam to use for video capture")

    frameCount = 0
    startTime = time.time()
    while True:
        frameCount += 1
        face_locations = []
        face_encodings = []

        ret, img = capture.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        face_detector = dlib.get_frontal_face_detector()

        faces = face_detector(img, 0)
        for face in faces:
            x,y,w,h = face.left(), face.top(), face.right(), face.bottom()
            face_locations.append((x,y,w,h))

        # TODO: Figure out what this is
        # face_encodings = face_recognition.face_encodings(img, known_face_locations = face_locations, num_jitters = 1)

        # Currently stable around 1.93 FPS, which is usable but low
        count = 1
        for (left, top, bottom, right) in face_locations:
            cv2.rectangle(img, (left,top), (right,bottom), (0,0,255), 2)
            cutImg = img[top:bottom, left:right]
            cv2.imshow("Face" + str(count), cutImg)

        thisTime = time.time()
        newFPS = frameCount / (thisTime - startTime)
        print(newFPS)
        cv2.putText(img, str(newFPS), (10,25), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2)

        cv2.imshow("Full", img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    if capture != None:
        capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run()