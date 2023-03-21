'''
Performs facial detection -> emotion recognition pipeline on camera node input
When confident a new emotion has been recognized, publishes it to the game node
Receives instructions from game node to pause publishing changes for recalibration or a new round delay
'''


import cv2
import dlib
import time
import joblib
import argparse
import numpy as np
from imutils.face_utils import rect_to_bb
from tensorflow.keras.models import load_model
from std_msgs.msg import String
import bearmax_emotion.emotion_lib.src.utils as utils
from importlib.resources import path

ALL_EMOTIONS = utils.ALL_EMOTIONS

NEW_ROUND_PAUSE = 1000
RECAL_PAUSE = 5000

currentEmotion = ""
pubToController = None

modelName = "CNNModel_feraligned+ck_5emo"
detector = "dlib"
hist_eq = True
hog_detector = dlib.get_frontal_face_detector()

'''
TODO below:
1. Track the user's face each frame and crop/test that specifically (not just first item)
2. Update the current emotion and, if changed, publish to the game node
3. When receive a newRound message, don't publish for NEW_ROUND_PAUSE ms and then send whatever the current emotion is
4. When receive a recalibration message, don't publish for RECAL_PAUSE seconds and then reset the tracked face to be the central most face and send whatever the current emotion is
'''

# Just face detection: 15FPS
# With emotion model: 3.8FPS
def dlib_detector(frame):
    gray_frame= cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    offset = 15
    x_pos,y_pos = 10,40
    detected_emotion = "invalid"
    detected_face_pos = (0,0,0)
    frame_h, frame_w, _ = frame.shape

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

        detected_emotion = str(label2text[predicted_label]).lower()
        detected_face_pos = (
            round((x+(w/2))/frame_w, 3), # x
            round((y+(h/2))/frame_h, 3), # y
            round((w*h)/(frame_w*frame_h), 3) # z
        )

        text = f"Person {idx+1} :  "
        y_pos = y_pos + 2*offset
        utils.draw_text_with_backgroud(frame, text, x_pos, y_pos, font_scale=0.3, box_coords_2=(2,-2))
        for k,v in label2text.items():
            text = f"{v}: {round(predicted_proba[0][k]*100, 3)}%"
            y_pos = y_pos + offset
            utils.draw_text_with_backgroud(frame, text, x_pos, y_pos, font_scale=0.3, box_coords_2=(2,-2))

    return detected_emotion, detected_face_pos

desiredLeftEye=(0.32, 0.32)

MODEL_PATH = str(path("bearmax_emotion.emotion_lib.src.misc", modelName + ".h5"))
LABEL2TEXT_PATH = str(path("bearmax_emotion.emotion_lib.src.misc", "label2text_" + modelName + ".pkl"))

model = load_model(MODEL_PATH)
label2text = joblib.load(LABEL2TEXT_PATH)

def runCamera():
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
            dlib_detector(frame)
    
        tt += time.time() - tik
        fps = frame_count / tt
        label = f"Detector: {detector} ; Model: {modelName}; FPS: {round(fps, 2)}"
        utils.draw_text_with_backgroud(frame, label, 10, 20, font_scale=0.35)

        cv2.imshow("Face Detection Comparison", frame)
        if cv2.waitKey(10) == ord('q'):
            break

    cv2.destroyAllWindows()
    vidcap.release()


def run_pipeline(frame, frame_count, tt):
    """This function is called by ros2 node with the frame to process"""

    frame = cv2.flip(frame, 1, 0)
    tik = time.time()

    emo, fpos = dlib_detector(frame)
    
    tt += time.time() - tik
    fps = frame_count / tt
    label = f"Detector: {detector} ; Model: {modelName}; FPS: {round(fps, 2)}"
    utils.draw_text_with_backgroud(frame, label, 10, 20, font_scale=0.35)

    return tt, frame, fpos, emo
