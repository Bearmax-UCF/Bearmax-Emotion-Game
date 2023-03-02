import shutil
from glob import glob

emotion = "neutral"
allImages = glob("testing/{emotion}/*.jpg".format(emotion=emotion))

for image in allImages:
    newName = "t" + image.split("/")[2]
    # print(image + " -> " + 'training/{emotion}/{newName}'.format(emotion=emotion, newName=newName))
    shutil.move(image, 'training/{emotion}/{newName}'.format(emotion=emotion, newName=newName))