'''
Handles communication with outside system and operates 
all nodes involved in the emotion recognition game
'''

from random import randint

class EmotionGame:
    '''
    In storage arrays:
    0 = Happy
    1 = Sad
    2 = Angry
    3 = Neutral
    4 = Other
    '''
    ALL_EMOTIONS = ["happy", "sad", "angry", "neutral", "other"]
    msToDelay = 1000

    def __init__(self):
        self.correct = [] # int[]
        self.wrong = [] # int[]
        self.numCorrect = 0 # Total
        self.gameFin = None # TimeStamp
        self.currentEmotion = "other"
        self.targetEmotion = None
    
    # TODO: When topic subscription produces a change, send it here
    def registerEmotionChange(self, newEmotion):
        newEmotion = newEmotion.lower()

        if not newEmotion in self.ALL_EMOTIONS:
            return
        
        if self.currentEmotion == newEmotion:
            return
        
        self.currentEmotion = newEmotion
        if newEmotion == self.targetEmotion:
            self.roundWinRoutine()

    def roundWinRoutine(self):
        #TODO
        self.doFeedbackAction("positive")
        self.chooseNewTargetEmotion()
    
    def roundLoseRoutine(self):
        self.doFeedbackAction("negative")
        self.chooseNewTargetEmotion()
    
    def doFeedbackAction(self, feedbackType): #TODO
        if feedbackType == "positive":
            return
        elif feedbackType == "negative":
            return
        
    def chooseNewTargetEmotion(self):
        prev = self.ALL_EMOTIONS.index(self.targetEmotion)
        next = prev
        
        while next == prev:
            next = randint(0, len(self.ALL_EMOTIONS) - 1)

        self.targetEmotion = self.ALL_EMOTIONS[next]
        # TODO: Send pipeline a reset request so it delays before sending new emotions
    
    def pauseGame():
        # Continue to accept emotion changes but ignore any further propagation
        return #TODO

    def unpauseGame():
        # Start up a new round with a new target emotion
        return #TODO
    
    def performRecalibration():
        # Pause game, ask pipeline to recalibrate after X seconds, unpause
        return #TODO
    
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
On node teardown, submit class stats to DB
'''

def submitToDB():
    return #TODO

gameInstance = EmotionGame()