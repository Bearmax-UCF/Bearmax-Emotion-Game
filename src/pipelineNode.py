'''
Performs facial detection -> emotion recognition pipeline on camera node input
When confident a new emotion has been recognized, publishes it to the game node
Receives instructions from game node to pause publishing changes for recalibration or a new round delay
'''

from tensorflow.keras.models import load_model
import cv2
import dlib
import time
import joblib
import numpy as np
import math
from imutils.face_utils import rect_to_bb
from std_msgs.msg import String
import bearmax_emotion.emotion_lib.src.utils as utils
from importlib.resources import path

ALL_EMOTIONS = utils.ALL_EMOTIONS

RECAL_DUR = 3 #sec
IGNORE_THRESH = 250 #px

# Emotion recognition setup
current_emotion = "invalid"
model_name = "CNNModel_feraligned+ck_5emo"
hog_detector = dlib.get_frontal_face_detector()

desiredLeftEye = (0.32, 0.32)
MODEL_PATH = path("bearmax_emotion.emotion_lib.src.misc", model_name + ".h5")
LABEL2TEXT_PATH = path("bearmax_emotion.emotion_lib.src.misc",
                       "label2text_" + model_name + ".pkl")

model = None
label2text = None
with MODEL_PATH as MODEL_PATH_OBJ, LABEL2TEXT_PATH as LABEL2TEXT_PATH_OBJ:
    model = load_model(MODEL_PATH_OBJ)
    label2text = joblib.load(LABEL2TEXT_PATH_OBJ)

# ROS setup
pause_end = None

# Center of face bounding box
last_coords = None
last_face = None


def getEmotion(frame):
    global last_coords, last_face
    fwidth, fheight, _ = frame.shape

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = hog_detector(gray_frame)

    # Center of the last frame we tracked a face
    compare_center = last_coords
    # If last_coords == None, coming out of recalibration and want most central face
    if last_coords == None:
        compare_center = (fwidth / 2, fheight / 2)

    closest = None
    closest_face = None
    closest_dist = None

    # print(len(faces))
    for _i, face in enumerate(faces):
        x, y, w, h = rect_to_bb(face)
        # Blue = Candidate faces in the frame
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        this_center = (x + .5 * w, y + .5 * h)
        distance = math.dist(compare_center, this_center)

        # Always take first face or take if it's closer to the point we care about
        if closest == None or distance <= closest_dist:
            closest = this_center
            closest_face = face
            closest_dist = distance

    # Didn't find any faces or closest face is too far of a jump from last face location
    if closest == None or (last_coords != None and closest_dist >= IGNORE_THRESH):
        if last_face != None:
            x, y, w, h = rect_to_bb(last_face)
            # Green = last face we saw that was valid in a previous frame
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        return "invalid", (0, 0, 0)

    last_coords = closest
    last_face = closest_face
    x, y, w, h = rect_to_bb(closest_face)

    # Red = new face this frame
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
    return dlib_detector(frame, gray_frame, last_face)


def dlib_detector(frame, gray_frame, face):
    gray_frame = cv2.equalizeHist(gray_frame)
    fwidth, fheight, _ = frame.shape

    img_arr = gray_frame
    img_arr = utils.align_face(gray_frame, face, desiredLeftEye)
    img_arr = utils.preprocess_img(img_arr, resize=False)

    predicted_proba = model.predict(img_arr)
    predicted_label = np.argmax(predicted_proba[0])

    x, y, w, h = rect_to_bb(face)
    text = f"Prediction: {label2text[predicted_label]}"
    utils.draw_text_with_backgroud(frame, text, x + 5, y, font_scale=0.4)

    detected_emotion = str(label2text[predicted_label]).lower()
    detected_face_pos = (
        round((x+(w/2))/fwidth, 3),  # x
        round((y+(h/2))/fheight, 3),  # y
        round((w*h)/(fwidth*fheight), 3)  # z
    )

    return detected_emotion, detected_face_pos

# TODO: Figure out how to use (low prio until we get something working)


def recalibrate():
    global pause_end, last_coords, last_dims
    pause_end = time.time() + RECAL_DUR
    last_coords = None
    last_dims = (0, 0, 0, 0)


# How I was handling recalibration, which was triggered by a key press
# on the CV2 window that called recalibrate()
# Feel free to use or discard
'''
def runCamera():
    global pause_end

    in_pause = pause_end != None and tik < pause_end
    if not in_pause:
        # Reset if we passed the pause end
        if pause_end != None:
            pause_end = None
            frame_count += 1

            update_target_face_coords(frame)
        else:
            time_left = round(pause_end - tik, 2)
            label = f"Recal in {time_left}s"
            cv2.putText(frame, label, (int(fwidth - 200), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, thickness=3, color=(255,255,0))
'''


def run_pipeline(frame, frame_count, tt):
    """This function is called by ros2 node with the frame to process"""

    frame = cv2.flip(frame, 1, 0)
    tik = time.time()

    emo, fpos = getEmotion(frame)

    tt += time.time() - tik
    fps = frame_count / tt
    label = f"Model: {model_name}; FPS: {round(fps, 2)}"
    utils.draw_text_with_backgroud(frame, label, 10, 20, font_scale=0.35)

    return tt, frame, fpos, emo
