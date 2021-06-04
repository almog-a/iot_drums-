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



def calculateVolume(stick) -> int:
    stick_acceleration = stick.getStickAcceleration()
    volume = 0
    if stick_acceleration > 10000:
        volume = 5
    elif stick_acceleration > 8000:
        volume = 4
    elif stick_acceleration > 6000:
        volume = 3
    elif stick_acceleration > 4000:
        volume = 2
    elif stick_acceleration > 2000:
        volume = 1
    return volume



def trackStick(stick):
    stick.setMin(min(stick.getMin(), stick.getY()))
    if (len(stick.getPoints()) == 4):
        yDirection = stick.getPoints()[3][1] - stick.getPoints()[0][1]  #last-current
        if (stick.getIsGoingDown() and yDirection < -10):
            volume = calculateVolume(stick)

            #volume = 600 - stick.getMin()
            #volume = int(volume / 100) -1
            #snare.play(volume)
            playDrumByPosition(stick.getX(),stick.getY(),stick.getZ(),volume)
            stick.setMin(600)
            stick.updateIsGoingDown(False)
        if np.abs(yDirection) > 10 and yDirection >= 0:
            stick.updateIsGoingDown(True)
    return

def define_locations():
    #define the points of all drums and return them

    #snare_points = [(6, 0), (263, 99999), (0, 99999)]
    #kick_points = [(359, 0), (625, 99999), (0, 9999)]


    hihate_points = [(6, 150), (263, 350), (0, 99999)]
    snare_points =[(359, 150), (625, 350), (0, 9999)]

    kick_points = [(359, 390), (625, 100000), (0, 9999)]


    return snare_points,kick_points,hihate_points;

def locate_drums_in_frame(color_frame):
    #locate the drums rectangles
    snare_points,kick_points,hihat_points=define_locations()
    snare=cv2.rectangle(color_frame, snare_points[0], snare_points[1], (0, 76, 76), 2)
    cv2.putText(color_frame, "snare", (5, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 76, 76), 2)
    hihat = cv2.rectangle(color_frame, hihat_points[0], hihat_points[1], (255, 0, 0), 2)
    cv2.putText(color_frame, "kick", (380, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 0), 2)
    kick = cv2.rectangle(color_frame, kick_points[0], kick_points[1], (255, 0, 0), 2)

   # cv2.putText(color_frame, 'snare', (snare_points[0], snare_points[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
    #kick=cv2.rectangle(img, kick_points[0], kick_points[1], (255, 0, 0), 2)
    return
'''
def playDrumByPosition(x, y, volume):
  #  s1.write('s'.encode())
    if (x < 360):
        kick.play(volume)
    elif (x < 360):
        snare.play(volume)
    else:
        hihat.play(volume)
'''

def is_drum(x,y,z,points):
    #return True
    left_x, down_y = points[0][0], points[0][1] #first point
    right_x,up_y=points[1][0],points[1][1] #second point
    close_z,far_z=points[2][0],points[2][1]

    # down_y:177, up_y: 337, right_x:263, up_y=337

    #up_y is ok and down has problems!!
    if (x < right_x) and (x > left_x) and (y < up_y+30) and (y > down_y-55):  # and (z < far_z) and (z > close_z):

    #if (x < right_x) and (x > left_x) and (y < up_y) and (y > down_y): #and (z < far_z) and (z > close_z):


 #   if (x<right_x)and(x>left_x)and(y<up_y)and(y>down_y)and(z<far_z)and(z>close_z):
        return True
    return False


def playDrumByPosition(x,y,z,volume):
  #  s1.write('s'.encode())
    snare_points,kick_points,hihat_points=define_locations()
    snare_points,kick_points,hihat_points=define_locations()
    if(is_drum(x,y,z,snare_points)): snare.play(volume)
    if (is_drum(x, y,z, kick_points)): kick.play(volume)
    if (is_drum(x, y, z, hihat_points)): hihat.play(volume)

def main():
    debug = True
    record = False  #change to true if working with records
    center = deque(maxlen = 2)
    center.appendleft((0,0))
    center.appendleft((0,0))

    frameCount = 0
    file_name ='C:/Users/User/Documents/20210420_143134.bag'  #path to bag file
    vs = rs.DepthCamera(record, file_name )
    vs.startStream()
    leftStick = Stick("left",vs)
    rightStick = Stick("right",vs)

    cv2.namedWindow('Color Stream', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('Color Stream', vs.mouseRGB)
    time.sleep(1.0)
    cap,s=vs.createTrackbar()


    #l, u = (26, 10, 30), (26, 10, 30)
    #l,u=(26, 10, 30),(97, 100, 255)
    while True:
        ''''''
        vs.controlBar()
        # Read in 1 frame at a time and flip the image
        is_captured, depth_frame, color_frame,raw_depth_frame = vs.get_frame()
        leftStick.setRawDepthFrame(raw_depth_frame)
        rightStick.setRawDepthFrame(raw_depth_frame)
        vs.color_frame = color_frame
        l=vs.objLower
        u=vs.objUpper

        '''
        #change all green to pure green
        hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)

        #check if we are before calibration
        flag1=np.array_equal((30, 86, 14),l)
        flag2=np.array_equal((97, 244, 255),u)
        if((flag1 is False) or (flag2 is False)):
        #if(True):
            greenMask = cv2.inRange(hsv, l, u) #after calibration
            color_frame[greenMask == 255] = (0, 255, 0)
        #vs.color_frame = color_frame
        l,u=vs.objLower,vs.objUpper
        
        
        #green mask ends here - probably unnecessary
        '''

        #removing background, may cause latency
        #fgMask = backSub.apply(color_frame)
        #color_frame = cv2.bitwise_and(color_frame,color_frame, mask=fgMask)

         #important !!! to return it to code
        # Mask the image so the result is just the drum stick tips

        mask, res = vs.find_color(color_frame)

        # Find contours in the mask
        cnts = vs.find_cnt(mask)
        ##### was commented until year in debug



        '''
        ####temporarely added!! to debug color recognition problem 
        # Mask the image so the result is just the drum stick tips
        hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (30, 86, 14), (97, 244, 255))
        mask = cv2.erode(mask, None, iterations=1)
        # Find contours in the mask
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # sort cnts so we can loop through the two biggest (the sticks hopefully)
        cnts = sorted(cnts, key=lambda x: cv2.contourArea(x), reverse=True)
        res = cv2.bitwise_and(color_frame, color_frame, mask=mask)
        ## added until here in debug 
        '''



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

                        #distance = vs.get_distance(leftStick.getX(), leftStick.getY(),raw_depth_frame)
                        trackStick(leftStick)

                        cv2.putText(color_frame, "{}mm".format(leftStick.getZ()), (leftStick.getX() ,leftStick.getY()- 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

                else:
                    cv2.circle(color_frame, center[i], 10, (76,76,156), 3)
                    rightStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        #distance=vs.get_distance(rightStick.getX(), rightStick.getY(),raw_depth_frame)
                        trackStick(rightStick)
                        cv2.putText(color_frame, "{}mm".format(rightStick.getZ()), (rightStick.getX() ,rightStick.getY() - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

            # Only one stick - split screen in half

            else:

                if(center[i][0]>= 300):
                    leftStick.addPoint(center[i][0], center[i][1])
                    #distance = vs.get_distance(leftStick.getX(), leftStick.getY(), raw_depth_frame)
                    if (frameCount > 4):
                        trackStick(leftStick)

                else:
                    rightStick.addPoint(center[i][0], center[i][1])
                    #distance = vs.get_distance(rightStick.getX(), rightStick.getY(), raw_depth_frame)
                    if (frameCount > 4):
                        trackStick(rightStick)
        if debug:
            cv2.imshow("Depth Strem", depth_frame)
            cv2.imshow("res Strem", res)
            cv2.imshow("mask", mask)

        locate_drums_in_frame(color_frame)
        cv2.imshow("Color Stream", color_frame)




        #key = cv2.waitKey(1) & 0xFF
        frameCount += 1

        # if the 'Esc' key is pressed, quit
        if vs.keyUI():
            break
    vs.release()
    cv2.destroyAllWindows()






if __name__== "__main__":
   # s1 = serial.Serial('COM3', 9600)
    time.sleep(3)
    main()
