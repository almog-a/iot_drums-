from collections import deque
import os
import numpy as np
import realsense_depth as rs

class Stick:
    def __init__(self, name,vs ,sensitivity = 7):
        self.points = deque(maxlen = 4)
        self.minPoint = 480
        self.isGoingDown = False
        self.min = 480
        self.name = name
        self.raw_depth_frame=[]
        self.vs=vs
        self.sensitivity = sensitivity
        self.points.appendleft((0, 0,0))
        self.points.appendleft((0, 0,0))
        self.points.appendleft((0, 0,0))
        self.points.appendleft((0, 0,0))


    def setRawDepthFrame(self,raw_depth_frame):
        self.raw_depth_frame=raw_depth_frame

    # def isCloseEnough(self):
    #     return self.isSound

    def getName(self):
        return self.name
   
    def setMin(self, min):
        self.min = min

    def getMin(self):
        return self.min

    def getIsGoingDown(self):
        return self.isGoingDown

    def updateIsGoingDown(self, isGoingDown):
        self.isGoingDown = isGoingDown

    def findDepthByXY(self,x,y):
        z=self.vs.get_distance(x,y,self.raw_depth_frame)
        return z

    def getZ(self):
        #z=self.vs.get_distance(self.getX(), self.getY(),self.raw_depth_frame)
        #return z
        z = 9000
        for point in self.points:
            if (point[2] != 0):
                z = point[2]
                break
        return z

    def getPoints(self):
        return self.points

    def addPoint(self, x, y):
        prev_point = (self.getX(), self.getY())
        z = self.findDepthByXY(x,y)
        #self.points.appendleft((x,y,z))
        if z == 0:
            z = self.points[0][2]
        #if len(self.points) != 0 and (x, y) != (self.points[0][0], self.points[0][1]):
        self.points.appendleft((x,y,z))

        cur_point=(x,y)

        dist=np.sqrt((cur_point[0] - prev_point[0]) ** 2 + (cur_point[1] - prev_point[1]) ** 2)
        #if (dist>45):self.isSound=False
        #else: self.isSound=True

    def getX(self):
        return self.points[0][0]

    def getY(self):
        return self.points[0][1]

    def getZ(self):
        return self.points[0][2]
    #velocity returned in pixel/s

    def getStickVelocity(self,start_point,end_point):
        vector_x = (end_point[0]-start_point[0])/(2/30)
        vector_y = (end_point[1] - start_point[1])/(2/30)
        #vector_z = (end_point[2] - start_point[2])/(2/30)
        vector_z = 0
        return (vector_x**2 + vector_y**2 + vector_z**2)**(0.5)

    #returns acceleration pixel/s^2

    def getStickAcceleration(self):
        fps = 30 #frames per second
        curr_velocity = self.getStickVelocity(end_point=self.points[0],start_point=self.points[1])
        prev_velocity = self.getStickVelocity(end_point=self.points[2],start_point=self.points[3])
        #it takes 1/30 to take one frame, the 2 velocities are based on 4 frames so 4*(1/30)=4/30
        a = (curr_velocity - prev_velocity)/(4/30)
        return a





    def getStickVelocity(self,start_point,end_point):
        vector_x = (end_point[0]-start_point[0])/(2/30)
        vector_y = (end_point[1] - start_point[1])/(2/30)
        #vector_z = (end_point[2] - start_point[2])/(2/30)
        vector_z = 0
        return (vector_x**2 + vector_y**2 + vector_z**2)**(0.5)

    #returns acceleration pixel/s^2
    def getStickAcceleration(self):
        fps = 30 #frames per second
        curr_velocity = self.getStickVelocity(end_point=self.points[0],start_point=self.points[1])
        prev_velocity = self.getStickVelocity(end_point=self.points[2],start_point=self.points[3])
        #it takes 1/30 to take one frame, the 2 velocities are based on 4 frames so 4*(1/30)=4/30
        a = (curr_velocity - prev_velocity)/(4/30)
        return a
