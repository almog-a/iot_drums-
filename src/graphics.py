import cv2
from realsense_depth import DepthCamera
import numpy as np


class graphic_drums:

    def __init__(self, vs:DepthCamera, drum_locations, is_debug = False):
        self.points_dict = drum_locations
        self.vs = vs
        self.is_debug = is_debug
        self.circles = []
        cv2.namedWindow('Color Stream', cv2.WND_PROP_FULLSCREEN)
        cv2.setMouseCallback('Color Stream', self.vs.mouseRGB)

    def __del__(self):
        cv2.destroyAllWindows()

    def createTrackbar(self):
        cap = cv2.VideoCapture(0)
        s="Trackbars"
        s2="Trackbars second color"

        cv2.namedWindow(s)
        cv2.namedWindow(s2)

        low_h=self.vs.objLower[0]
        low_s = self.vs.objLower[1]
        low_v = self.vs.objLower[2]
        high_h = self.vs.objUpper[0]
        high_s = self.vs.objUpper[1]
        high_v = self.vs.objUpper[2]

        low_h2 = self.vs.objLower_second[0]
        low_s2 = self.vs.objLower_second[1]
        low_v2 = self.vs.objLower_second[2]
        high_h2 = self.vs.objUpper_second[0]
        high_s2 = self.vs.objUpper_second[1]
        high_v2 = self.vs.objUpper_second[2]

        nothing=lambda *args : None
        cv2.createTrackbar("L - H", s, low_h, 179, nothing)
        cv2.createTrackbar("L - S", s, low_s, 256, nothing)
        cv2.createTrackbar("L - V", s, low_v, 256, nothing)
        cv2.createTrackbar("U - H", s, high_h, 179, nothing)
        cv2.createTrackbar("U - S", s, high_s, 256, nothing)
        cv2.createTrackbar("U - V", s, high_v, 256, nothing)

        cv2.createTrackbar("L - H   2", s2, low_h2, 179, nothing)
        cv2.createTrackbar("L - S   2", s2, low_s2, 256, nothing)
        cv2.createTrackbar("L - V   2", s2, low_v2, 256, nothing)
        cv2.createTrackbar("U - H   2", s2, high_h2, 179, nothing)
        cv2.createTrackbar("U - S   2", s2, high_s2, 256, nothing)
        cv2.createTrackbar("U - V   2", s2, high_v2, 256, nothing)

        #return cap, s,
        return cap,s,s2

    def controlBar(self):
        s="Trackbars"
        s2 = "Trackbars second color"

        l_h = cv2.getTrackbarPos("L - H", s)
        l_s = cv2.getTrackbarPos("L - S", s)
        l_v = cv2.getTrackbarPos("L - V", s)
        u_h = cv2.getTrackbarPos("U - H", s)
        u_s = cv2.getTrackbarPos("U - S", s)
        u_v = cv2.getTrackbarPos("U - V", s)

        self.vs.objLower = np.array([l_h, l_s, l_v])
        self.vs.objUpper = np.array([u_h, u_s, u_v])


        l_h2 = cv2.getTrackbarPos("L - H   2", s2)
        l_s2 = cv2.getTrackbarPos("L - S   2", s2)
        l_v2 = cv2.getTrackbarPos("L - V   2", s2)
        u_h2 = cv2.getTrackbarPos("U - H   2", s2)
        u_s2 = cv2.getTrackbarPos("U - S   2", s2)
        u_v2 = cv2.getTrackbarPos("U - V   2", s2)

        self.vs.objLower_second = np.array([l_h2, l_s2, l_v2])
        self.vs.objUpper_second = np.array([u_h2, u_s2, u_v2])


    def updateBar(self):
        s = "Trackbars"
        s2 = "Trackbars second color"
        cv2.setTrackbarPos("L - H",s,self.vs.objLower[0])
        cv2.setTrackbarPos("L - S", s, self.vs.objLower[1])
        cv2.setTrackbarPos("L - V", s, self.vs.objLower[2])

        cv2.setTrackbarPos("U - H",s,self.vs.objUpper[0])
        cv2.setTrackbarPos("U - S", s, self.vs.objUpper[1])
        cv2.setTrackbarPos("U - V", s, self.vs.objUpper[2])

        s2="Trackbars second color"
        cv2.setTrackbarPos("L - H   2", s2, self.vs.objLower_second[0])
        cv2.setTrackbarPos("L - S   2", s2, self.vs.objLower_second[1])
        cv2.setTrackbarPos("L - V   2", s2, self.vs.objLower_second[2])

        cv2.setTrackbarPos("U - H   2", s2, self.vs.objUpper_second[0])
        cv2.setTrackbarPos("U - S   2", s2, self.vs.objUpper_second[1])
        cv2.setTrackbarPos("U - V   2", s2, self.vs.objUpper_second[2])


    def locate_drums_in_frame(self, color_frame):
        # locate the drums rectangles
        snare_points = self.points_dict["snare_points"]
        kick_points = self.points_dict["kick_points"]
        hihat_points = self.points_dict["hihat_points"]
        tom_points = self.points_dict["tom_points"]
        floor_points = self.points_dict["floor_points"]
        ride_points = self.points_dict["ride_points"]

        snare = cv2.rectangle(color_frame, snare_points[0], snare_points[1], (0, 0, 255), 2)
        cv2.putText(color_frame, "snare", snare_points[0], cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2)

        hihat = cv2.rectangle(color_frame, hihat_points[0], hihat_points[1], (255, 0, 0), 2)
        cv2.putText(color_frame, "hihat", hihat_points[0], cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 0), 2)

        kick = cv2.rectangle(color_frame, kick_points[0], kick_points[1], (0, 255, 255), 2)
        cv2.putText(color_frame, "kick", kick_points[0], cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 255), 2)

        tom = cv2.rectangle(color_frame, tom_points[0], tom_points[1], (0, 120, 255), 2)
        cv2.putText(color_frame, "tom", tom_points[0], cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 120, 255), 2)

        #floor = cv2.rectangle(color_frame, floor_points[0], floor_points[1], (0, 255, 0), 2)
        #cv2.putText(color_frame, "floor", floor_points[0], cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 0), 2)

        ride = cv2.rectangle(color_frame, ride_points[0], ride_points[1], (255, 0, 255), 2)
        cv2.putText(color_frame, "ride", ride_points[0], cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 255), 2)

    def show_graphics(self,color_frame,depth_frame = None,res=None,mask=None,res2=None,mask2=None):
        if(self.is_debug):
            cv2.imshow("Depth Strem", depth_frame)
            cv2.imshow("res Strem", res)
            #cv2.imshow("mask", mask)
            cv2.imshow("res Strem 2", res2)
            #cv2.imshow("mask", mask2)


        #color_frame2=cv2.convertScaleAbs(color_frame, -2, 2)
        img=color_frame
        '''
        brightness = 85
        contrast = -20
        img = np.int16(color_frame)
        img = img * (contrast / 127 + 1) - contrast + brightness
        img = np.clip(img, 0, 255)
        img = np.uint8(img)
        '''
        cv2.imshow("Color Stream", color_frame)
        #cv2.imshow("Color Stream2", img)

    def draw_sticks_circles(self, color_frame):#draw circles to sticks on color stream
        for center,color in self.circles:
            cv2.circle(color_frame, center, 10, color, 3)
        self.circles.clear()

    def add_circle(self,center,color):
        self.circles.append((center, color))

