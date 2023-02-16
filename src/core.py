"""
Extracts faces and emotions from video capture
"""

import cv2

'''
Required Data:
Correct: Array
Wrong: Array
TotalPlays: int
NumCorrect: int
LastPlayed: TimeStamp
UserID: String

Need clarification with Rachel before we use this, which seems to mix individual 
and overall stats into each table record
'''

def pushRunStats(): # TODO: send current data to API and clear stats
    return

def invalidCamera(capture):
    return capture is None or not capture.isOpened()

def run():
    # Need a better face recognizer, this is only frontal which isn't good enough when you are looking not directly at camera
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    capture = None

    for i in range(0, 5):
        capture = cv2.VideoCapture(i)

        if invalidCamera(capture):
            continue

        print(f"Found webcam at index {i}")
        break

    if invalidCamera(capture):
        raise Exception("Could not find webcam to use for video capture")

    while True: # TODO: Better condition, run without blocking
        ret, img = capture.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = img[y:y+h, x:x+w]
            cv2.imshow("Face", roi_color)
            
        cv2.imshow("Full", img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
        
    pushRunStats()
    if capture != None:
        capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run()