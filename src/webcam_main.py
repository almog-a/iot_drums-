from collections import deque
from webcamvideostream import WebcamVideoStream
from DrumSound import DrumSound
from imutils.video import FileVideoStream
from imutils.video import FPS
from webcam_stick import Webcam_Stick
import os
import numpy as np
import argparse
import cv2
import imutils
import time
import simpleaudio as sa

snare = DrumSound("../audio/", "Snare", 5, ".wav")
kick = DrumSound("../audio/", "Kick", 5, ".wav")
tom = DrumSound("../audio/", "Tom", 5, ".wav")
floor = DrumSound("../audio/", "Floor", 5, ".wav")
hihat = DrumSound("../audio/", "Hat", 5, ".wav")
ride = DrumSound("../audio/", "Ride", 5, ".wav")



def trackStick(stick):
    stick.setMin(min(stick.getMin(), stick.getY()))
    if (len(stick.getPoints()) == 4):
        yDirection = stick.getPoints()[3][1] - stick.getPoints()[0][1]
        # y direction: negative = down. positive = up


        if (stick.getIsGoingDown() and yDirection < -20):
            volume = 600 - stick.getMin()
            volume = int(volume / 100) - 1
            # snare.play(volume)
            print("stick sidr is {}".format(stick.getName()))
            is_in_rectangle=playDrumByPosition(stick.getX(), stick.getY(), volume)
           # stick.print_location_history()


            stick.setMin(600)
            stick.updateIsGoingDown(False)
        if np.abs(yDirection) > 20 and yDirection >= 0:
            # positive = up
            stick.updateIsGoingDown(True)
    return


def playDrumByPosition(x,y,volume):
  #  s1.write('s'.encode())

    snare_points,hihat_points,floor_points,kick_points = get_points()
    if(is_drum(x,y,snare_points)):
      snare.play(volume)
      print("hihat is playing")
      '''
      print("x is {} and y is {}".format(x, y))
      return True
    else:
      print("x is {} and y is {} not in the rectangle".format(x, y))
      return False
    # if (is_drum(x, y,kick_points)): kick.play(volume)
    '''
    if (is_drum(x, y, floor_points)):
        floor.play(volume)
        print("floor is playing")
    if (is_drum(x, y, hihat_points)):
        hihat.play(volume)
        print("hihat is playing")
    if (is_drum(x, y, kick_points)):
        kick.play(volume)
        print("kick is playing")

'''
def playDrumByPosition(x, y, volume):
    if (x < 400) :
        kick.play(volume)
    elif (x > 400):
        snare.play(volume)
    else:
        hihat.play(volume)
'''

def is_drum(x,y,points):
    snare_points,hihat_points,floor_points,kick_points = get_points()

    left_x, down_y = points[0][0], points[0][1] #first point
    right_x,up_y=points[1][0], points[1][1] #second point

    #snare_points = [(6, 177), (620, 337), (0, 9999)]
    # if ((y>0)and(y<999)):
    #  return True

   # close_z,far_z=points[2][0],points[2][1]
    delta_y=50;
    if (x<right_x)and(x>left_x)and(y<up_y+delta_y)and(y>down_y-delta_y):
    #if (y < up_y) and (y > down_y):
        return True
    return False



def get_points():
    delta_x=200-6
    delta_y=337-177
    start_x=6
    start_y=50
    snare_points = [(start_x, start_y), (start_x+delta_x,start_y+delta_y), (0, 9999)]
    start_x=start_x+delta_x
    start_y=start_y
    hihat_points = [(start_x, start_y), (start_x+delta_x,start_y+delta_y), (0, 9999)]
    start_x = start_x + delta_x
    start_y = start_y
    floor_points = [(start_x, start_y), (start_x+delta_x,start_y+delta_y), (0, 9999)]
    #snare_points = [(6, 177), (200, 337), (0, 9999)]
    #snare_points = [(6, 0), (620, 999), (0, 9999)]
    start_x=snare_points[0][0]
    start_y=snare_points[1][1]

    kick_points = [(start_x,start_y),(start_x+delta_x*3,start_y+delta_y), (0, 9999)]
    #x between 6 to 620
    #y between 177 to 337
    return snare_points,hihat_points,floor_points,kick_points

def locate_rectangles(frame):
    # locate snare in frame:
    snare_points, hihat_points, floor_points,kick_points = get_points()
    snare = cv2.rectangle(frame, snare_points[0], snare_points[1], (0, 76, 76), 2)
    cv2.putText(frame, "snare", (5, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 76, 76), 2)
    cv2.rectangle(frame, hihat_points[0], hihat_points[1], (0, 76, 76), 2)
    cv2.putText(frame, "hihat", (200, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (139, 0, 0), 2)
    cv2.rectangle(frame, hihat_points[0], hihat_points[1], (139, 0, 0), 2)
    cv2.putText(frame, "floor", (195 + 200, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (2, 34, 100), 2)
    cv2.rectangle(frame, floor_points[0], floor_points[1], (2, 34, 100), 2)
    snare_points, hihat_points, floor_points, kick_points = get_points()
    cv2.rectangle(frame, kick_points[0], kick_points[1], (2, 97, 177), 2)
    cv2.putText(frame, "kick", (195 + 200, 300), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (2, 97, 177), 2)

def main():

    center = deque(maxlen=3)
    center.appendleft((0, 0))
    center.appendleft((0, 0))
    center.appendleft((0, 0))
    leftStick = Webcam_Stick("left")
    rightStick = Webcam_Stick("right")
    legStick = Webcam_Stick("leg")
    #rightStick = Webcam_Stick("right")

    # Upper and lower bounds (HSV) for the stick color
    objLower = (30, 86, 14)
    objUpper = (97, 244, 255)
    frameCount = 0
    vs = WebcamVideoStream(src=0).start()

    # vs = FileVideoStream(0).start()
    time.sleep(1.0)
    while True:

        # Read in 1 frame at a time and flip the image
        frame = vs.read()

        # frame = imutils.resize(frame, width = 600, height = 300)
        frame = cv2.flip(frame, 1)
        locate_rectangles(frame)

        overlay = frame.copy()
        alpha = 0.5
        cv2.line(overlay, (150, 0), (150, 600), (138, 138, 138), 1)
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        cv2.line(frame, (450, 0), (450, 600), (138, 138, 138), 1)

        # Mask the image so the result is just the drum stick tips
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, objLower, objUpper)
        mask = cv2.erode(mask, None, iterations=1)

        # Find contours in the mask
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # sort cnts so we can loop through the two biggest (the sticks hopefully)
        cnts = sorted(cnts, key=lambda x: cv2.contourArea(x), reverse=True)
        #print(cnts[0])

        numSticks = min(len(cnts), 2)
        for i in range(numSticks):
            ((x, y), radius) = cv2.minEnclosingCircle(cnts[i])
            if (radius > 4):
                center.appendleft((int(x), int(y)))
        for i in range(numSticks):
            if (numSticks > 1):
                if (center[i][0] <= center[(i + 1) % 2][0]):
                    cv2.circle(frame, center[i], 10, (156, 76, 76), 3)
                    leftStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(leftStick)

                else:
                    cv2.circle(frame, center[i], 10, (76, 76, 156), 3)

                    rightStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(rightStick)

            # Only one stick - split screen in half
            else:
                if (center[i][0] >= 300):
                    leftStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(leftStick)
                else:
                    rightStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        trackStick(rightStick)
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        frameCount += 1

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break
    vs.stop()
    cv2.destroyAllWindows()



if __name__== "__main__":
    main()