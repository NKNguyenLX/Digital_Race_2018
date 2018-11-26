import argparse
import cv2
import numpy as np
import cv2
from scipy.misc import imresize
from moviepy.editor import VideoFileClip
from IPython.display import HTML

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",required=True,
                help="path to the (optional) video file")
args = vars(ap.parse_args())

camera = cv2.VideoCapture(args["video"])
count = 100
left_path = "dataset/0/"
right_path = "dataset/1/"
while True:
    # Grab the current frame
    (grabbed, frame) = camera.read()

    # If we are viewing a video and we did not grab a frame, then we have reached the end of the video
    if args.get("video") and not grabbed:
        break
    cv2.imshow("features", frame)
    save = cv2.waitKey(0)

    if save == ord('l'):
        cv2.imwrite(left_path + str(count) + ".jpg",frame)
        count = count + 1
        print "Left image: " + str(count)

    if save == ord('r'):
        cv2.imwrite(right_path + str(count) + ".jpg",frame)
        count = count + 1
        print "Right image: " + str(count)
