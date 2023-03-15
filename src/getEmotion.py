import cv2
import dlib
import time
import joblib
import argparse
import numpy as np
from imutils.face_utils import rect_to_bb
from tensorflow.keras.models import load_model

import utils

modelName = "CNNModel_feraligned+ck_5emo"
detector = "dlib"
hist_eq = True

hog_detector = dlib.get_frontal_face_detector()

# Just face detection: 15FPS
# With emotion model: 3.8FPS
def dlib_detector(frame):
    gray_frame= cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    offset = 15
    x_pos,y_pos = 10,40

    faces = hog_detector(gray_frame)
    for idx, face in enumerate(faces):
        if hist_eq:
            gray_frame = cv2.equalizeHist(gray_frame)

        img_arr = gray_frame
        img_arr = utils.align_face(gray_frame, face, desiredLeftEye)
        img_arr = utils.preprocess_img(img_arr, resize=False)

        predicted_proba = model.predict(img_arr)
        predicted_label = np.argmax(predicted_proba[0])

        x,y,w,h = rect_to_bb(face)
        cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0), 2)
        text = f"Person {idx+1}: {label2text[predicted_label]}"
        utils.draw_text_with_backgroud(frame, text, x + 5, y, font_scale=0.4)

        text = f"Person {idx+1} :  "
        y_pos = y_pos + 2*offset
        utils.draw_text_with_backgroud(frame, text, x_pos, y_pos, font_scale=0.3, box_coords_2=(2,-2))
        for k,v in label2text.items():
            text = f"{v}: {round(predicted_proba[0][k]*100, 3)}%"
            y_pos = y_pos + offset
            utils.draw_text_with_backgroud(frame, text, x_pos, y_pos, font_scale=0.3, box_coords_2=(2,-2))

desiredLeftEye=(0.32, 0.32)
model = load_model("misc/" + modelName + ".h5")
label2text = joblib.load("misc/label2text_" + modelName + ".pkl")

if __name__ == "__main__":
    iswebcam = True
    vidcap=cv2.VideoCapture(0)

    frame_count = 0
    tt = 0
    while True:
        status, frame = vidcap.read()
        if not status:
            break

        frame_count += 1

        if iswebcam:
            frame=cv2.flip(frame,1,0)

        # try:
        tik = time.time()

        if detector == "dlib":
            out = dlib_detector(frame)
    
        tt += time.time() - tik
        fps = frame_count / tt
        label = f"Detector: {detector} ; Model: {modelName}; FPS: {round(fps, 2)}"
        utils.draw_text_with_backgroud(frame, label, 10, 20, font_scale=0.35)

        cv2.imshow("Face Detection Comparison", frame)
        if cv2.waitKey(10) == ord('q'):
            break

    cv2.destroyAllWindows()
    vidcap.release()
