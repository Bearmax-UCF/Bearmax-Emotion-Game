"""
Extracts faces and emotions from video capture
"""

import cv2

class EmotionGame:
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

    def __init__(self, video):
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.capture = cv2.VideoCapture(0)

    def run(self): # TODO: needs to run without blocking
        while True: # TODO: Better condition
            ret, img = capture.read()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)

            for (x, y, w, h) in faces:
                cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                roi_color = img[y:y+h, x:x+w]
                cv2.imshow("Face", roi_color)
                
            cv2.imshow('img', img)
            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break

    def stop(self):
        # TODO: stop the non-blocking process and push stats to API
        return

    def pushRunStats(self): # TODO: send current data to API and clear stats
        return

    def __del__(self):
        # TODO: Send data to API
        capture.release()
        cv2.destroyAllWindows()

def createGameInstance(video=0):
    gameInstance = EmotionGame(video)

def startGame(gameInstance):
    if not __isGameInstance(gameInstance):
        return
    gameInstance.run()

def terminateGame(gameInstance):
    if not __isGameInstance(gameInstance):
        return
    

def __isGameInstance(gameInstance):
    if gameInstance == None or not type(gameInstance) is EmotionGame:
        print("Error: provided parameter is not of type EmotionGame")
        return False
    return True

if __name__ == "__main__":
    instance = createGameInstance()
    startGame(instance)