import cv2
from realsense_depth import DepthCamera
import numpy as np


class Drums:

    def __init__(self, pixel_ratio=3.7):#3.7 from a distance of 1.5 m
        self.pixel_ratio = pixel_ratio
        self.drums_width = 25 #size of drums in cm
        self.drums_height = 25
        self.drums_depth = 99999

        self.default_depth=99999
        self.default_depth2 = 99999
        #self.hihat_points = self.calculate_drum_size(290, 80,self.default_depth) before change
        self.hihat_points = self.calculate_drum_size(235, 80, self.default_depth) #after change
        self.snare_points = self.calculate_drum_size(410, 205,self.default_depth)
        #self.tom_points = self.calculate_drum_size(510, 205,self.default_depth)
        self.tom_points = self.calculate_drum_size(310, 205, self.default_depth)
        self.floor_points = self.calculate_drum_size(210, 205,self.default_depth)
        #self.ride_points = self.calculate_drum_size(310, 205,self.default_depth)
        self.ride_points = self.calculate_drum_size( 470, 80, self.default_depth)
        self.kick_points = self.calculate_drum_size(424, 400,1450)

    def calculate_drum_size(self,x ,y, z=0):
        return [(x, y), (x+int(self.drums_width*self.pixel_ratio), y+int(self.drums_height*self.pixel_ratio)), (z-self.drums_depth*10,z )]

    def get_locations(self):
        return dict(hihat_points=self.hihat_points, snare_points=self.snare_points, tom_points=self.tom_points,
                    floor_points=self.floor_points, ride_points=self.ride_points,kick_points=self.kick_points)
