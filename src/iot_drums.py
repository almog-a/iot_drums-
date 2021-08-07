from collections import deque
from DrumSound import DrumSound
from Stick import Stick
from drums import Drums
import numpy as np
import cv2
import time
import realsense_depth as rs
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


def trackStick(stick, drum_locations,isArduinoConnected):

    stick.setMin(min(stick.getMin(), stick.getY()))
    if (len(stick.getPoints()) == 4):
        yDirection = stick.getPoints()[3][1] - stick.getPoints()[0][1]  #last-current
        if (stick.getIsGoingDown() and yDirection < -stick.sensitivity):
            volume = calculateVolume(stick)
            playDrumByPosition(stick.getX(),stick.getY(),stick.getZ(),volume,drum_locations,isArduinoConnected)
            stick.setMin(600)
            stick.updateIsGoingDown(False)
        if np.abs(yDirection) > stick.sensitivity and yDirection >= 0:
            stick.updateIsGoingDown(True)
    return


def is_drum(x,y,z,points):
    #return True
    left_x, down_y = points[0][0], points[0][1] #first point
    right_x,up_y=points[1][0],points[1][1] #second point
    close_z,far_z=points[2][0],points[2][1]

    # down_y:177, up_y: 337, right_x:263, up_y=337

    #up_y is ok and down has problems!!
    #if (x < right_x) and (x > left_x) and (y < up_y+30) and (y > down_y-55) and (z < far_z) and (z > close_z):
        #return True
    if (x < right_x) and (x > left_x) and (y < up_y + 30) and (y > down_y - 55):
        #print(x,y)
        if (z < far_z) and (z > close_z):
            return True
        else:
            print('miss')
            print(z)
        return False



def playDrumByPosition(x,y,z,volume,drum_locations,isArduinoConnected=0):
    drumStr = ''

    if(is_drum(x,y,z,drum_locations['snare_points'])):
        drumStr='snare@'
        snare.play(volume)
    elif (is_drum(x, y,z, drum_locations['kick_points'])):
        drumStr = 'kick@'
        kick.play(volume)
    elif (is_drum(x, y, z, drum_locations['hihat_points'])):
        drumStr = 'hihat@'
        hihat.play(volume)

    elif (is_drum(x, y, z, drum_locations['tom_points'])):
        drumStr = 'tom@'
        tom.play(volume)

    elif (is_drum(x, y, z, drum_locations['floor_points'])):
        drumStr = 'floor@'
        floor.play(volume)

    elif (is_drum(x, y, z, drum_locations['ride_points'])):
        drumStr = 'ride@'
        ride.play(volume)
    if (drumStr!='') and (isArduinoConnected): s1.write(drumStr.encode)


class iot_drums:

    def __init__(self):
        self.debug = True
        self.record = True  #change to true if working with records
        self.center = deque(maxlen = 2)
        self.center2 = deque(maxlen=1)

        self.center.appendleft((0,0))
        self.center.appendleft((0,0))
        self.center2.appendleft((0,0))

        self.frameCount = 0
        file_name ='C:/Users/User/Documents/20210420_143134.bag'  #path to bag file
        self.vs = rs.DepthCamera(self.record, file_name)
        self.vs.startStream()
        self.leftStick = Stick("left",self.vs)
        self.rightStick = Stick("right",self.vs)
        self.legStick = Stick("leg",self.vs, sensitivity=5)

        self.drums = Drums()
        #dictionary with drum boundaries
        #drum_locations = define_locations()
        self.drum_locations = self.drums.get_locations()
        self.graphicDrums = graphic_drums(vs=self.vs, drum_locations=self.drum_locations, is_debug=self.debug)
        self.vs.setUpdateBarFunc(self.graphicDrums.updateBar)
        time.sleep(1.0)
        cap,s,s2=self.graphicDrums.createTrackbar()
        self.isArduinoConnected=0 #choose if arduino is in use

        if(self.isArduinoConnected):
            s1 = serial.Serial('COM3', 9600)
            time.sleep(3)


    def iteration(self):
        self.graphicDrums.controlBar()
        # Read in 1 frame at a time and flip the image
        is_captured, depth_frame, color_frame,raw_depth_frame = self.vs.get_frame()
        self.leftStick.setRawDepthFrame(raw_depth_frame)
        self.rightStick.setRawDepthFrame(raw_depth_frame)
        self.legStick.setRawDepthFrame(raw_depth_frame)
        self.vs.color_frame = color_frame

         #important !!! to return it to code
        # Mask the image so the result is just the drum stick tips
        mask, res, mask2, res2 = self.vs.find_color(color_frame)
        # Find contours in the mask
        cnts = self.vs.find_cnt(mask)
        cnts2 = self.vs.find_cnt(mask2)

        ##### was commented until year in debug
        numSticks = min(len(cnts), 2)
        numSticks2 = min(len(cnts2), 1)

        for i in range(numSticks):
            ((x, y), radius) = cv2.minEnclosingCircle(cnts[i]) #find a circle to enclose cnts[i]
            if (radius > 4):
                self.center.appendleft((int(x),int(y)))

        for i in range(numSticks2):
            ((x2, y2), radius2) = cv2.minEnclosingCircle(cnts2[i])
            if (radius2 > 4):
                self.center2.appendleft((int(x2),int(y2)))

        #center and center2 are updated

        for i in range(numSticks2):
            self.graphicDrums.add_circle(self.center2[i], (76, 76, 156))
            self.legStick.addPoint(self.center2[i][0], self.center2[i][1])
            if (self.frameCount > 4):
                # distance=vs.get_distance(rightStick.getX(), rightStick.getY(),raw_depth_frame)
                trackStick(self.legStick, self.drum_locations,self.isArduinoConnected)
                cv2.putText(color_frame, "{}mm".format(self.legStick.getZ()), (self.legStick.getX(), self.legStick.getY() ),
                            cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)




        for i in range(numSticks):
            if (numSticks > 1):
                if (self.center[i][0] <= self.center[(i + 1) % 2][0]): #check which center is left
                    #cv2.circle(color_frame, center[i], 10, (156, 76, 76), 3)
                    self.graphicDrums.add_circle(self.center[i],(156, 76, 76))

                    self.leftStick.addPoint(self.center[i][0], self.center[i][1])
                    if (self.frameCount > 4):

                        #distance = vs.get_distance(leftStick.getX(), leftStick.getY(),raw_depth_frame)
                        trackStick(self.leftStick, self.drum_locations,self.isArduinoConnected)
                        cv2.putText(color_frame, "{}mm".format(self.leftStick.getZ()), (self.leftStick.getX() ,self.leftStick.getY()- 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

                else:
                    #cv2.circle(color_frame, center[i], 10, (76,76,156), 3)
                    self.graphicDrums.add_circle(self.center[i],(76,76,156))
                    self.rightStick.addPoint(self.center[i][0], self.center[i][1])
                    if (self.frameCount > 4):
                        #distance=vs.get_distance(rightStick.getX(), rightStick.getY(),raw_depth_frame)
                        trackStick(self.rightStick, self.drum_locations,self.isArduinoConnected)
                        cv2.putText(color_frame, "{}mm".format(self.rightStick.getZ()), (self.rightStick.getX() ,self.rightStick.getY() - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

            # Only one stick - split screen in half

            else:

                if(self.center[i][0]>= 300):
                    self.leftStick.addPoint(self.center[i][0], self.center[i][1])
                    #distance = vs.get_distance(leftStick.getX(), leftStick.getY(), raw_depth_frame)
                    if (self.frameCount > 4):
                        trackStick(self.leftStick, self.drum_locations,self.isArduinoConnected)

                else:
                    self.rightStick.addPoint(self.center[i][0], self.center[i][1])
                    #distance = vs.get_distance(rightStick.getX(), rightStick.getY(), raw_depth_frame)
                    if (self.frameCount > 4):
                        trackStick(self.rightStick, self.drum_locations,self.isArduinoConnected)



        self.graphicDrums.locate_drums_in_frame(color_frame)
        self.graphicDrums.show_graphics(color_frame,depth_frame=depth_frame, res=res, mask=mask,res2=res2,mask2=mask2)
        #graphicDrums.show_graphics(color_frame, depth_frame=depth_frame, res=res2, mask=mask2)

        #key = cv2.waitKey(1) & 0xFF
        self.frameCount += 1

        return color_frame, [res, res2, depth_frame]

        # if the 'Esc' key is pressed, quit
        #if vs.keyUI():
        #   break
    #vs.release()


if __name__== "__main__":

    main()
