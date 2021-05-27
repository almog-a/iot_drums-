from collections import deque
import os
import numpy as np
import src.realsense_depth as rs

class Stick:
    def __init__(self, name,vs):
        self.points = deque(maxlen = 4)
        self.minPoint = 480
        self.isGoingDown = False
        self.min = 480
        self.name = name
        self.raw_depth_frame=[]
        self.vs=vs

        self.points.appendleft((0, 0))
        self.points.appendleft((0, 0))
        self.points.appendleft((0, 0))
        self.points.appendleft((0, 0))

    def setRawDepthFrame(self,raw_depth_frame):
        self.raw_depth_frame=raw_depth_frame

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
        return self.points[0][2]

    def getPoints(self):
        return self.points

    def addPoint(self, x, y):
        z = self.findDepthByXY(x,y)
        #self.points.appendleft((x,y,z))

        #if len(self.points) != 0 and (x, y) != (self.points[0][0], self.points[0][1]):
        self.points.appendleft((x,y,z))

    def getX(self):
        return self.points[0][0]

    def getY(self):
        return self.points[0][1]

    def getZ(self):
        return self.points[0][2]
