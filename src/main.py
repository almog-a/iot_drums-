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
    if (x < 150):
        kick.play(volume)
    elif (x < 450):
        snare.play(volume)
    else:
        hihat.play(volume)

def main():
    record = False
    center = deque(maxlen = 2)
    center.appendleft((0,0))
    center.appendleft((0,0))
    leftStick = Stick("left")
    rightStick = Stick("right")
    # Upper and lower bounds (HSV) for the stick color
    objLower = (30, 86, 14)
    objUpper = (97, 244, 255)
    frameCount = 0
    #vs1 = WebcamVideoStream(src=1).start()
    file_name =  'C:/Users/User/Documents/20210420_143134.bag'  #path to bag file

    vs = rs.DepthCamera(record,file_name )
    vs.startStream()

    #vs = FileVideoStream(0).start()
    time.sleep(1.0)
    while True:
        # Read in 1 frame at a time and flip the image
        is_captured, depth_frame, color_frame,raw_depth_frame = vs.get_frame()
        #frame = imutils.resize(frame, width = 600, height = 300)
        depth_frame = cv2.flip(depth_frame, 1)
        color_frame = cv2.flip(color_frame, 1)
        frame = color_frame
        raw_depth_frame=cv2.flip(raw_depth_frame,1)
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', color_frame)
        #cv2.waitKey(1)
        overlay = color_frame.copy()


        #removing background, may cause latency
        fgMask = backSub.apply(frame)
        frame = cv2.bitwise_and(color_frame,color_frame, mask=fgMask)

        overlay = frame.copy()
        alpha = 0.5
        cv2.line(overlay,(150,0),(150,600),(138,138,138),1)
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha,0, frame)
        cv2.line(frame,(450,0),(450,600),(138,138,138),1)

        # Mask the image so the result is just the drum stick tips
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, objLower, objUpper)
        mask = cv2.erode(mask, None, iterations=1)

        # Find contours in the mask
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # sort cnts so we can loop through the two biggest (the sticks hopefully)
        cnts = sorted(cnts,key = lambda x: cv2.contourArea(x), reverse = True)
        
        numSticks = min(len(cnts), 2)
        for i in range(numSticks):
            ((x, y), radius) = cv2.minEnclosingCircle(cnts[i])
            if (radius > 4):
                center.appendleft((int(x),int(y)))
        for i in range(numSticks):
            if (numSticks > 1):
                if (center[i][0] <= center[(i + 1) % 2][0]):
                    cv2.circle(frame, center[i], 10, (156, 76, 76), 3)
                    leftStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(leftStick)
                        distance=vs.get_distance(leftStick.getX(), leftStick.getY(),raw_depth_frame)
                        cv2.putText(color_frame, "{}mm".format(distance), (leftStick.getY(), leftStick.getX() - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

                        #cv2.circle(frame, center[i], 10, (76,76,156), 3

                else:
                    cv2.circle(frame, center[i], 10, (76,76,156), 3)
                    rightStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(rightStick)
                        distance=vs.get_distance(rightStick.getX(), rightStick.getY(),raw_depth_frame)
                        cv2.putText(color_frame, "{}mm".format(distance), (rightStick.getY(), rightStick.getX() - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

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
        cv2.imshow("Color Stream",frame)
        cv2.imshow("Depth Strem", depth_frame)
        #key = cv2.waitKey(1) & 0xFF
        frameCount += 1
    
        # if the 'q' key is pressed, stop the loop
        if vs.keyUI():
            break
    vs.release()
    cv2.destroyAllWindows()



def mouseRGB(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsB = color_image[y,x,0]
        colorsG = color_image[y,x,1]
        colorsR = color_image[y,x,2]
        colors = color_image[y,x]
        print("Red: ",colorsR)
        print("Green: ",colorsG)
        print("Blue: ",colorsB)
        print("BGR Format: ",colors)
        print("Coordinates of pixel: X: ",x,"Y: ",y)

#cv2.setMouseCallback('Color Stream', mouseRGB) put in code to enable function

if __name__== "__main__":
    main()
