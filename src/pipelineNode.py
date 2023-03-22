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
import math
from imutils.face_utils import rect_to_bb
from tensorflow.keras.models import load_model
# from std_msgs.msg import String
import utils

ALL_EMOTIONS = utils.ALL_EMOTIONS

RECAL_DUR = 3 #sec
NEWROUND_DUR = 1 #sec
IGNORE_THRESH = 250 #px

# Emotion recognition setup
current_emotion = ""
model_name = "CNNModel_feraligned+ck_5emo"
hog_detector = dlib.get_frontal_face_detector()
hist_eq = True

desiredLeftEye=(0.32, 0.32)
model = load_model("misc/" + model_name + ".h5")
label2text = joblib.load("misc/label2text_" + model_name + ".pkl")

# Frame and ROS setup
fwidth = 0
fheight = 0
publisherToGame = None
pause_end = None

# Center of face bounding box
last_coords = None
last_dims = (0, 0, 0, 0) # (x, y, w, h)
last_face = None

def update_target_face_coords(frame):
    global last_coords, last_dims
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
        x,y,w,h = rect_to_bb(face)
        # Blue = Candidate faces in the frame
        cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0), 2)
        this_center = (x + .5 * w, y + .5 * h)
        distance = math.dist(compare_center, this_center)

        # Always take first face or take if it's closer to the point we care about
        if closest == None or distance <= closest_dist:
            closest = this_center
            closest_face = face
            closest_dist = distance

    # Didn't find any faces or closest face is too far of a jump from last face location
    if closest == None or (last_coords != None and closest_dist >= IGNORE_THRESH):
        x, y, w, h = last_dims
        # Green = last face we saw that was valid in a previous frame
        print("No new frame")
        # TODO: print works but this rectangle doesn't draw
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
        return
    
    last_coords = closest
    last_face = closest_face
    x, y, w, h = rect_to_bb(closest_face)
    # Red = new face this frame
    # cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)
    dlib_detector(frame, gray_frame, last_face)

def dlib_detector(frame, gray_frame, face):
    offset = 15
    x_pos,y_pos = 10,40

    if hist_eq:
        gray_frame = cv2.equalizeHist(gray_frame)

    img_arr = gray_frame
    img_arr = utils.align_face(gray_frame, face, desiredLeftEye)
    img_arr = utils.preprocess_img(img_arr, resize=False)

    predicted_proba = model.predict(img_arr)
    predicted_label = np.argmax(predicted_proba[0])

    x,y,w,h = rect_to_bb(face)
    cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)
    text = f"Prediction: {label2text[predicted_label]}"
    utils.draw_text_with_backgroud(frame, text, x + 5, y, font_scale=0.4)

    current_emotion = label2text[predicted_label]
    print(current_emotion)

    # for k,v in label2text.items():
    #     text = f"{v}: {round(predicted_proba[0][k]*100, 3)}%"
    #     y_pos = y_pos + offset
    #     utils.draw_text_with_backgroud(frame, text, x_pos, y_pos, font_scale=0.3, box_coords_2=(2,-2))

def bb_to_rect(bb):
    x, y, w, h = bb
    return [(x, y), (x + w, y + h)]

def recalibrate():
    global pause_end, last_coords, last_dims
    pause_end = time.time() + RECAL_DUR
    last_coords = None
    last_dims = (0, 0, 0, 0)

def runCamera():
    global pause_end
    iswebcam = True
    vidcap=cv2.VideoCapture(0)
    fwidth = vidcap.get(cv2.CAP_PROP_FRAME_WIDTH)
    fheight = vidcap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    frame_count = 0
    tt = 0

    while True:
        status, frame = vidcap.read()
        if not status:
            break

        frame=cv2.flip(frame,1,0)
        tik = time.time()
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
        
        tt += time.time() - tik
        fps = frame_count / tt
        label = f"FPS: {round(fps, 2)}"
        cv2.putText(frame, label, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, thickness=3, color=(255,255,0))

        cv2.imshow("Face Detection", frame)
        key = cv2.waitKey(10)
        if key == ord('q'):
            break
        elif key == ord('r'):
            recalibrate()

    cv2.destroyAllWindows()
    vidcap.release()

def handleNewPause():
    return # TODO

if __name__ == '__main__':
    # try:
        # rospy.init_node('gamePipeline')

        # # Subscribes to pausePipeline topic, which either sends 'newRound' and 'recalibrate'
        # rospy.Subscriber("pausePipeline", String, handleNewPause)

        # # When we confidently find a new emotion, publish to the game controller
        # publisherToGame = rospy.Publisher("changeEmotion", String)

        runCamera()
    # except rospy.ROSInterruptException:
    #     pass