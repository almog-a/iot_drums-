from collections import deque
from src.DrumSound import DrumSound
from src.Stick import Stick
from src.drums import Drums
import numpy as np
import cv2
import time
import src.realsense_depth as rs
import serial

from graphics import graphic_drums
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
    delta=2000
    if stick_acceleration > 10000-delta:
        volume = 5
    elif stick_acceleration > 8000-delta:
        volume = 4
    elif stick_acceleration > 6000-delta:
        volume = 3
    elif stick_acceleration > 4000-delta:
        volume = 2
    elif stick_acceleration > 2000-delta:
        volume = 1
    return volume


def define_locations():
    hihat_points = [(6, 150), (263, 350), (0, 99999)]
    snare_points =[(359, 150), (625, 350), (0, 9999)]
    kick_points = [(359, 390), (625, 100000), (0, 9999)]
    return dict(hihat_points=hihat_points, snare_points=snare_points, kick_points=kick_points)


def trackStick(stick, drum_locations):
    stick.setMin(min(stick.getMin(), stick.getY()))
    if (len(stick.getPoints()) == 4):
        yDirection = stick.getPoints()[3][1] - stick.getPoints()[0][1]  #last-current
        if (stick.getIsGoingDown() and yDirection < -10):
            volume = calculateVolume(stick)
            playDrumByPosition(stick.getX(),stick.getY(),stick.getZ(),volume,drum_locations)
            stick.setMin(600)
            stick.updateIsGoingDown(False)
        if np.abs(yDirection) > 10 and yDirection >= 0:
            stick.updateIsGoingDown(True)
    return


def is_drum(x,y,z,points):
    #return True
    left_x, down_y = points[0][0], points[0][1] #first point
    right_x,up_y=points[1][0],points[1][1] #second point
    close_z,far_z=points[2][0],points[2][1]

    # down_y:177, up_y: 337, right_x:263, up_y=337

    #up_y is ok and down has problems!!
    if (x < right_x) and (x > left_x) and (y < up_y+30) and (y > down_y-55):  # and (z < far_z) and (z > close_z):
        return True
    return False

def playDrumByPosition(x,y,z,volume,drum_locations):
    if(is_drum(x,y,z,drum_locations['snare_points'])):
        #s1.write('snare@'.encode())
        snare.play(volume)
    elif (is_drum(x, y,z, drum_locations['kick_points'])):
        #s1.write('kick@'.encode())
        kick.play(volume)
    elif (is_drum(x, y, z, drum_locations['hihat_points'])):
        #s1.write('hihat@'.encode())
        hihat.play(volume)

    elif (is_drum(x, y, z, drum_locations['tom_points'])):
        #s1.write('tom@'.encode())
        tom.play(volume)

    elif (is_drum(x, y, z, drum_locations['floor_points'])):
        #s1.write('floor@'.encode())
        floor.play(volume)

    elif (is_drum(x, y, z, drum_locations['ride_points'])):
        #s1.write('ride@'.encode())
        ride.play(volume)


def main():
    debug = True
    record = False  #change to true if working with records
    center = deque(maxlen = 2)
    center.appendleft((0,0))
    center.appendleft((0,0))
    frameCount = 0
    file_name ='C:/Users/User/Documents/20210420_143134.bag'  #path to bag file
    vs = rs.DepthCamera(record, file_name)
    vs.startStream()
    leftStick = Stick("left",vs)
    rightStick = Stick("right",vs)
    drums = Drums()
    #dictionary with drum boundaries
    #drum_locations = define_locations()
    drum_locations = drums.get_locations()
    graphicDrums = graphic_drums(vs=vs, drum_locations=drum_locations, is_debug=debug)
    vs.setUpdateBarFunc(graphicDrums.updateBar)
    time.sleep(1.0)
    cap,s=graphicDrums.createTrackbar()

    while True:
        graphicDrums.controlBar()
        # Read in 1 frame at a time and flip the image
        is_captured, depth_frame, color_frame,raw_depth_frame = vs.get_frame()
        leftStick.setRawDepthFrame(raw_depth_frame)
        rightStick.setRawDepthFrame(raw_depth_frame)
        vs.color_frame = color_frame
        l=vs.objLower
        u=vs.objUpper

        #removing background, may cause latency
        #fgMask = backSub.apply(color_frame)
        #color_frame = cv2.bitwise_and(color_frame,color_frame, mask=fgMask)

         #important !!! to return it to code
        # Mask the image so the result is just the drum stick tips
        mask, res = vs.find_color(color_frame)
        # Find contours in the mask
        cnts = vs.find_cnt(mask)
        ##### was commented until year in debug
        numSticks = min(len(cnts), 2)
        for i in range(numSticks):
            ((x, y), radius) = cv2.minEnclosingCircle(cnts[i]) #find a circle to enclose cnts[i]
            if (radius > 4):
                center.appendleft((int(x),int(y)))
        for i in range(numSticks):
            if (numSticks > 1):
                if (center[i][0] <= center[(i + 1) % 2][0]): #check which center is left
                    #cv2.circle(color_frame, center[i], 10, (156, 76, 76), 3)
                    graphicDrums.add_circle(center[i],(156, 76, 76))
                    leftStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):

                        #distance = vs.get_distance(leftStick.getX(), leftStick.getY(),raw_depth_frame)
                        trackStick(leftStick, drum_locations)
                        cv2.putText(color_frame, "{}mm".format(leftStick.getZ()), (leftStick.getX() ,leftStick.getY()- 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

                else:
                    #cv2.circle(color_frame, center[i], 10, (76,76,156), 3)
                    graphicDrums.add_circle(center[i],(76,76,156))
                    rightStick.addPoint(center[i][0], center[i][1])
                    if (frameCount > 4):
                        #distance=vs.get_distance(rightStick.getX(), rightStick.getY(),raw_depth_frame)
                        trackStick(rightStick, drum_locations)
                        cv2.putText(color_frame, "{}mm".format(rightStick.getZ()), (rightStick.getX() ,rightStick.getY() - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

            # Only one stick - split screen in half

            else:

                if(center[i][0]>= 300):
                    leftStick.addPoint(center[i][0], center[i][1])
                    #distance = vs.get_distance(leftStick.getX(), leftStick.getY(), raw_depth_frame)
                    if (frameCount > 4):
                        trackStick(leftStick, drum_locations)

                else:
                    rightStick.addPoint(center[i][0], center[i][1])
                    #distance = vs.get_distance(rightStick.getX(), rightStick.getY(), raw_depth_frame)
                    if (frameCount > 4):
                        trackStick(rightStick, drum_locations)
        graphicDrums.locate_drums_in_frame(color_frame)
        graphicDrums.show_graphics(color_frame,depth_frame=depth_frame, res=res, mask=mask)
        #key = cv2.waitKey(1) & 0xFF
        frameCount += 1

        # if the 'Esc' key is pressed, quit
        if vs.keyUI():
            break
    vs.release()



if __name__== "__main__":
   # s1 = serial.Serial('COM3', 9600)
    time.sleep(3)
    main()
