from collections import deque
from src.webcamvideostream import WebcamVideoStream
from src.DrumSound import DrumSound
from imutils.video import FileVideoStream
from imutils.video import FPS
from src.Stick import Stick
import os
import numpy as np
import argparse
import cv2
import imutils
import time
import simpleaudio as sa
import src.realsense_depth as rs
import serial
##### taking care of imports
#----------comment-----

snare = DrumSound("../audio/", "Snare", 5, ".wav")
kick = DrumSound("../audio/", "Kick", 5, ".wav")
tom = DrumSound("../audio/", "Tom", 5, ".wav")
floor = DrumSound("../audio/", "Floor", 5, ".wav")
hihat = DrumSound("../audio/", "Hat", 5, ".wav")
ride = DrumSound("../audio/", "Ride", 5, ".wav")


#algoritm for background removal
backSub = cv2.createBackgroundSubtractorKNN(detectShadows=False)


def trackStick(stick):
    stick.setMin(min(stick.getMin(), stick.getY()))
    if (len(stick.getPoints()) == 4):
        yDirection = stick.getPoints()[3][1] - stick.getPoints()[0][1]
        if (stick.getIsGoingDown() and yDirection < -20):
            volume = 600 - stick.getMin()
            volume = int(volume / 100) -1
            #snare.play(volume)
            playDrumByPosition(stick.getX(), stick.getY(), volume)
            stick.setMin(600)
            stick.updateIsGoingDown(False)
        if np.abs(yDirection) > 20 and yDirection >= 0:
            stick.updateIsGoingDown(True)
    return 

def playDrumByPosition(x, y, volume):
    s1.write('s'.encode())
    if (x < 360):
        kick.play(volume)
    elif (x < 360):
        snare.play(volume)
    else:
        hihat.play(volume)

def main():
    debug = False
    record = False  #change to true if working with records
    center = deque(maxlen = 2)
    center.appendleft((0,0))
    center.appendleft((0,0))
    leftStick = Stick("left")
    rightStick = Stick("right")
    frameCount = 0
    file_name ='C:/Users/User/Documents/20210420_143134.bag'  #path to bag file
    vs = rs.DepthCamera(record, file_name )
    vs.startStream()

    cv2.namedWindow('Color Stream', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('Color Stream', vs.mouseRGB)
    time.sleep(1.0)
    while True:
        # Read in 1 frame at a time and flip the image
        is_captured, depth_frame, color_frame,raw_depth_frame = vs.get_frame()
        color_frame = color_frame
        vs.color_frame = color_frame

        #removing background, may cause latency
        #fgMask = backSub.apply(color_frame)
        #color_frame = cv2.bitwise_and(color_frame,color_frame, mask=fgMask)

        # Mask the image so the result is just the drum stick tips
        mask, res = vs.find_color(color_frame)

        # Find contours in the mask
        cnts = vs.find_cnt(mask)

        numSticks = min(len(cnts), 2)
        for i in range(numSticks):
            ((x, y), radius) = cv2.minEnclosingCircle(cnts[i]) #find a circle to enclose cnts[i]
            if (radius > 4):
                center.appendleft((int(x),int(y)))
        for i in range(numSticks):
            if (numSticks > 1):
                if (center[i][0] <= center[(i + 1) % 2][0]): #check which center is left
                    cv2.circle(color_frame, center[i], 10, (156, 76, 76), 3)
                    leftStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(leftStick)
                        distance = vs.get_distance(leftStick.getX(), leftStick.getY(),raw_depth_frame)
                        cv2.putText(color_frame, "{}mm".format(distance), (leftStick.getY(), leftStick.getX() - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

                else:
                    cv2.circle(color_frame, center[i], 10, (76,76,156), 3)
                    rightStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(rightStick)
                        distance=vs.get_distance(rightStick.getX(), rightStick.getY(),raw_depth_frame)
                        cv2.putText(color_frame, "{}mm".format(distance), (rightStick.getY(), rightStick.getX() - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

            # Only one stick - split screen in half

            else:
                if(center[i][0]>= 300):
                    leftStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(leftStick)

                else:
                    rightStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(rightStick)
        if debug:
            cv2.imshow("Depth Strem", depth_frame)
            cv2.imshow("res Strem", res)
        cv2.imshow("Color Stream", color_frame)




        #key = cv2.waitKey(1) & 0xFF
        frameCount += 1

        # if the 'q' key is pressed, stop the loop
        if vs.keyUI():
            break
    vs.release()
    cv2.destroyAllWindows()






if __name__== "__main__":
    s1 = serial.Serial('COM4', 9600)
    time.sleep(3)
    main()
