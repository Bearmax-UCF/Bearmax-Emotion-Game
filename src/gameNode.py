'''
Handles communication with outside system and operates 
all nodes involved in the emotion recognition game
'''

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from random import randint
import datetime
import requests
from utils import ALL_EMOTIONS

class EmotionGame:
    '''
    In storage arrays:
    0 = Happy
    1 = Sad
    2 = Angry
    3 = Neutral
    4 = Other
    '''

    def __init__(self):
        self.correct = [0] * len(ALL_EMOTIONS) # int[]
        self.wrong = [0] * len(ALL_EMOTIONS) # int[]
        self.currentEmotion = "other"
        self.targetEmotion = None
        self.paused = False

        rospy.Subscriber("changeEmotion", String, self.handleEmotionChange)
        rospy.Subscriber("gameControl", String, self.handleGameControl)

        # Options to send: "recalibrate" (should wait 5s before picking up central face), 
        # "newRound" (should wait Xs while tracking current face then start sending emotions)
        self.pubToPipeline = rospy.Publisher("pausePipeline", String)

        # When game ends or robot terminates, submit states to DB
        rospy.on_shutdown(self.submitToDB)

    # Listener for game actions from rest of the robot
    def handleGameControl(self, data):
        match data.data:
            case "pause":
                self.setGamePaused(True)
            case "unpause":
                self.setGamePaused(False)
            case "recalibrate":
                self.performRecalibration()
    
    def handleEmotionChange(self, data):
        if data == None or data.data == None:
            rospy.logerr("Could not register emotion change with data = %s", data.data)
            return
        
        newEmotion = data.data.lower()

        if not newEmotion in self.ALL_EMOTIONS:
            rospy.logerr("Received emotion %s not in list of accepted emotions", newEmotion)
            return
        
        if self.currentEmotion == newEmotion:
            return
        
        self.currentEmotion = newEmotion
        if newEmotion == self.targetEmotion:
            self.roundWinRoutine()

    def roundWinRoutine(self):
        self.correct 
        self.doFeedbackAction("positive")
        self.chooseNewTargetEmotion()
    
    def roundLoseRoutine(self):
        self.doFeedbackAction("negative")
        self.chooseNewTargetEmotion()
    
    def doFeedbackAction(self, feedbackType):
        if feedbackType == "positive":
            return # TODO
        elif feedbackType == "negative":
            return # TODO

    def chooseNewTargetEmotion(self):
        prev = self.ALL_EMOTIONS.index(self.targetEmotion)
        next = prev
        
        while next == prev:
            next = randint(0, len(self.ALL_EMOTIONS) - 1)

        self.targetEmotion = self.ALL_EMOTIONS[next]
        self.pubToPipeline.publish("newRound")
    
    def setGamePaused(self, newVal):
        self.paused = newVal
    
    def performRecalibration(self):
        # Pause game, ask pipeline to recalibrate after X seconds, unpause
        self.setGamePaused(True)
        self.currentEmotion = None
        self.chooseNewTargetEmotion()
        self.pubToPipeline.publish("recalibrate")
        self.setGamePaused(False)

    def submitToDB(self):
        '''
        {
            Correct: int[],
            Wrong: int[],
            GameFin: String,
        }
        '''
        # TODO: Update this format when Rachel gives me the format
        gameFin = datetime.datetime.now().strftime("%I:%M%p on %B %d, %Y")
        data = {
            'Correct': self.correct,
            'Wrong': self.wrong,
            'GameFin': gameFin
        }
        requests.POST("", data) # TODO: Update URL and any headers needed

'''
TODO
Instantiate new class
Start up ROS node
Set up subscribers to:
    - Current emotion changes
    - Remediation pause/unpause
Set up publisher to pipeline node to:
    - Wait X seconds to recalibrate
    - Reset for a new round (delay and reset current emotion)
        - Both should be the same channel but different values
On node teardown, submit class stats to DB
'''

if __name__ == '__main__':
    try:
        rospy.init_node('gameNode')
        gameInstance = EmotionGame()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass