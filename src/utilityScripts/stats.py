from glob import glob

types = ["happy", "angry", "sad", "neutral", "other"]

print("\n\nEmotion\t\tTest\tTrain\tUncropped\tTotal")

for emotion in types:
    testImages = glob(f"testing/{emotion}/*.jpg")
    trainingImages = glob(f"training/{emotion}/*.jpg")
    uncroppedImages = glob(f"uncropped/{emotion}/*.jpg")

    testc = len(testImages)
    trainingc = len(trainingImages)
    uncroppedc = len(uncroppedImages)

    tabs = "\t\t" if len(emotion) <= 7 else "\t"

    print(emotion + tabs + str(testc) + "\t" +
          str(trainingc) + "\t" + str(uncroppedc) + "\t\t" + str(testc + trainingc + uncroppedc))

print("\n\n")