'''
Performs facial detection -> emotion recognition pipeline on camera node input
When confident a new emotion has been recognized, publishes it to the game node
Receives instructions from game node to pause publishing changes for recalibration or a new round delay
'''


ALL_EMOTIONS = ["happy", "sad", "angry", "neutral", "other"]
currentEmotion = ""

