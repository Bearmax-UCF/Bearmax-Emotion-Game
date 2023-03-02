from glob import glob
import cv2
import os

imgType = "training"

testImages = glob(f"{imgType}/*.jpg")

count = 0

for img_path in testImages:
    image = cv2.imread(img_path)
    cv2.imshow('Emotion?', image)
    print(img_path)
    print("1 - happy\n2 - angry\n3 - sad\n4 - surprised\n5 - neutral\n6 - disgusted")
    pressed = cv2.waitKey(0)

    emotion = "failed"

    if pressed == 49: # 1
        emotion = "happy"
    elif pressed == 50:
        emotion = "angry"
    elif pressed == 51:
        emotion = "sad"
    elif pressed == 52:
        emotion = "surprised"
    elif pressed == 53:
        emotion = "neutral"
    elif pressed == 54:
        emotion = "disgusted"
    
    parts = img_path.split("/")
    print(parts)

    imgName = parts[len(parts) - 1]

    os.replace(f"{imgType}/{imgName}", f"{imgType}/{emotion}/{imgName}")

    count+=1
    if count == 100:
        break


# imgType = "training"

# testImages = glob(f"{imgType}/*.jpg")

# count = 0

# for img_path in testImages:
#     image = cv2.imread(img_path)
#     cv2.imshow('Emotion?', image)
#     print(img_path)
#     print("1 - happy\n2 - angry\n3 - sad\n4 - surprised\n5 - neutral\n6 - disgusted")
#     pressed = cv2.waitKey(0)

#     emotion = "failed"

#     if pressed == 49: # 1
#         emotion = "happy"
#     elif pressed == 50:
#         emotion = "angry"
#     elif pressed == 51:
#         emotion = "sad"
#     elif pressed == 52:
#         emotion = "surprised"
#     elif pressed == 53:
#         emotion = "neutral"
#     elif pressed == 54:
#         emotion = "disgusted"
    
#     parts = img_path.split("/")
#     print(parts)

#     imgName = parts[len(parts) - 1]

#     os.replace(f"{imgType}/{imgName}", f"{imgType}/{emotion}/{imgName}")

#     count+=1
#     if count == 100:
#         break