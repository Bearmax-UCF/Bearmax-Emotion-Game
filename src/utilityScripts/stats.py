from glob import glob

types = ["happy", "angry", "sad", "neutral", "other", "invalid"]

print("\n\nEmotion\t\tVerified\tUncropped")

for emotion in types:
    verified = glob(f"verified/{emotion}/*.jpg")
    uncropped = glob(f"uncropped/{emotion}/*.jpg")

    lver = len(verified)
    luc = len(uncropped)

    tabs = "\t\t" if len(emotion) <= 7 else "\t"

    print(emotion + tabs + str(lver) + "\t\t" + str(luc))

print("\n\n")