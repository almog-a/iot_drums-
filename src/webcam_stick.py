from collections import deque
import os
import numpy as np


class Webcam_Stick:
    def __init__(self, name):
        self.points = deque(maxlen=4)

        self.points.appendleft((0,0))
        self.points.appendleft((0, 0))
        self.points.appendleft((0, 0))
        self.points.appendleft((0, 0))

        self.minPoint = 500
        self.isGoingDown = False
        self.min = 500
        self.name = name

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

    def getPoints(self):
        return self.points

    def addPoint(self, x, y):
        if not (len(self.points) != 0 and (x, y) != (self.points[0][0], self.points[0][1])):
            x=5
        if len(self.points)!=0  and (x,y)!=(self.points[0][0],self.points[0][1]):
          self.points.appendleft((x, y))

    def getX(self):
        return self.points[0][0]

    def getY(self):
        #return self.points[0][1]
        return self.points[0][1]

    def print_location_history_xy(self):
        x_1=self.points[0][0]
        y_1=self.points[0][1]

        x_2 = self.points[1][0]
        y_2 = self.points[1][1]

        x_3 = self.points[2][0]
        y_3 = self.points[2][1]

        x_4 = self.points[3][0]
        y_4 = self.points[3][1]

        print("{} stick: x,y coordinates: ({},{}),({},{}),({},{}),({},{})".format(self.name,x_1,y_1,x_2,y_2,x_3,y_3,x_4,y_4))

