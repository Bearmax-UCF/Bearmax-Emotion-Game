from glob import glob

types = ["happy", "angry", "sad", "surprised", "disgusted", "neutral"]

print("\n\nEmotion\t\tTest\tTrain\tTotal")

for emotion in types:
    testImages = glob(f"testing/{emotion}/*.jpg")
    trainingImages = glob(f"training/{emotion}/*.jpg")

    testc = len(testImages)
    trainingc = len(trainingImages)

    tabs = "\t\t" if len(emotion) <= 7 else "\t"

    print(emotion + tabs + str(testc) + " \t" + str(trainingc) + " \t" + str(testc + trainingc))

print("\n\n")