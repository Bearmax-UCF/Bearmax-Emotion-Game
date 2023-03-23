import cv2
import dlib
import time
import math
from imutils.face_utils import rect_to_bb

RECAL_DUR = 3 #sec
IGNORE_THRESH = 250 #px

detector = "dlib"
hog_detector = dlib.get_frontal_face_detector()
fwidth = 0
fheight = 0

# Center of face bounding box
last_coords = None
last_dims = (0, 0, 0, 0)
pause_end = None

# Just face detection: 15FPS
# With emotion model: 3.8FPS
def getTargetFace(frame):
    global last_coords, last_dims
    gray_frame= cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = hog_detector(gray_frame)

    # Center of the last frame we tracked a face
    compare_center = last_coords
    # If last_coords == None, coming out of recalibration and want most central face
    if last_coords == None:
        compare_center = (fwidth / 2, fheight / 2)

    closest = None
    closest_dist = None
    closest_dims = (0, 0, 0, 0)

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
            closest_dist = distance
            closest_dims = (x, y, w, h)
    
    print(closest)

    # Didn't find any faces or closest face is too far of a jump from last face location
    if closest == None or (last_coords != None and closest_dist >= IGNORE_THRESH):
        x, y, w, h = last_dims
        # Green = last face we saw that was valid in a previous frame
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
        return
    
    last_coords = closest
    last_dims = closest_dims
    x, y, w, h = closest_dims
    # Red = new face this frame
    cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)

def recalibrate():
    global pause_end, last_coords, last_dims
    pause_end = time.time() + RECAL_DUR
    last_coords = None
    last_dims = (0, 0, 0, 0)

if __name__ == "__main__":
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

            if detector == "dlib":
                dlib_detector(frame)
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
