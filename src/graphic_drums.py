import cv2
from realsense_depth import DepthCamera
import numpy as np


class graphic_drums:

    def __init__(self, vs:DepthCamera, drum_locations, is_debug = False):
        self.points_dict = drum_locations
        self.vs = vs
        self.is_debug = is_debug
        cv2.namedWindow('Color Stream', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('Color Stream', self.vs.mouseRGB)

    def __del__(self):
        cv2.destroyAllWindows()

    def createTrackbar(self):
        cap = cv2.VideoCapture(0)
        s="Trackbars"
        cv2.namedWindow(s)
        low_h=self.vs.objLower[0]
        low_s = self.vs.objLower[1]
        low_v = self.vs.objLower[2]
        high_h = self.vs.objUpper[0]
        high_s = self.vs.objUpper[1]
        high_v = self.vs.objUpper[2]
        delta_h=50
        delta_s=15
        delta_v=15
        nothing=lambda *args : None
        cv2.createTrackbar("L - H", "Trackbars", low_h, 179, nothing)
        cv2.createTrackbar("L - S", "Trackbars", low_s, 256, nothing)
        cv2.createTrackbar("L - V", "Trackbars", low_v, 256, nothing)
        cv2.createTrackbar("U - H", "Trackbars", high_h, 179, nothing)
        cv2.createTrackbar("U - S", "Trackbars", high_s, 256, nothing)
        cv2.createTrackbar("U - V", "Trackbars", high_v, 256, nothing)
        # cv2.createTrackbar("L - H", "Trackbars", low_h, low_h+delta_h, nothing)
        # cv2.createTrackbar("L - S", "Trackbars", low_s, low_s-delta_s, nothing)
        # cv2.createTrackbar("L - V", "Trackbars", low_v, low_v+delta_v, nothing)
        # cv2.createTrackbar("U - H", "Trackbars", high_h, high_h+delta_h, nothing)
        # cv2.createTrackbar("U - S", "Trackbars", high_s, high_s+delta_s, nothing)
        # cv2.createTrackbar("U - V", "Trackbars", high_v, high_v+delta_v, nothing)
        return cap,s

    def controlBar(self):
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        self.vs.objLower = np.array([l_h, l_s, l_v])
        self.vs.objUpper = np.array([u_h, u_s, u_v])

    def updateBar(self):
        cv2.setTrackbarPos("L - H","Trackbars",self.vs.objLower[0])
        cv2.setTrackbarPos("L - s", "Trackbars", self.vs.objLower[1])
        cv2.setTrackbarPos("L - v", "Trackbars", self.vs.objLower[2])

        cv2.setTrackbarPos("U - H","Trackbars",self.vs.objUpper[0])
        cv2.setTrackbarPos("U - s", "Trackbars", self.vs.objUpper[1])
        cv2.setTrackbarPos("U - v", "Trackbars", self.vs.objUpper[2])

    def locate_drums_in_frame(self, color_frame):
        # locate the drums rectangles
        snare_points = self.points_dict["snare_points"]
        kick_points = self.points_dict["kick_points"]
        hihat_points = self.points_dict["hihat_points"]
        snare = cv2.rectangle(color_frame, snare_points[0], snare_points[1], (0, 76, 76), 2)
        cv2.putText(color_frame, "snare", (5, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 76, 76), 2)
        hihat = cv2.rectangle(color_frame, hihat_points[0], hihat_points[1], (255, 0, 0), 2)
        cv2.putText(color_frame, "kick", (380, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 0), 2)
        kick = cv2.rectangle(color_frame, kick_points[0], kick_points[1], (255, 0, 0), 2)

    def show_graphics(self,color_frame,depth_frame = None,res=None,mask=None):
        if(self.is_debug):
            cv2.imshow("Depth Strem", depth_frame)
            cv2.imshow("res Strem", res)
            cv2.imshow("mask", mask)
        cv2.imshow("Color Stream", color_frame)
