import pyrealsense2 as rs
import numpy as np
import cv2
import imutils

#--
class DepthCamera:
    def __init__(self, run_record =False ,file_name='NO_FILE'):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.run_record = run_record
        self.objLower = (30, 86, 14)
        self.objUpper = (97, 244, 255)

        self.calibrate_color = False  # if turn to True, will calibrate color according to next mouse click
        self.calibrate_points = []  # will contain calibration points.
        self.color_frame = []
        if not run_record:
        # Get device product line for setting a supporting resolution
            pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            pipeline_profile = self.config.resolve(pipeline_wrapper)
            device = pipeline_profile.get_device()
            device_product_line = str(device.get_info(rs.camera_info.product_line))
            self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
            self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)

        else:
            # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
            rs.config.enable_device_from_file(self.config, file_name)
            # Configure the pipeline to stream the depth stream
            # Change this parameters according to the recorded bag file resolution
            self.config.enable_stream(rs.stream.depth, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, rs.format.rgb8, 30)

    def startStream(self):
        # Start streaming
        self.pipeline.start(self.config)
        #wait for a coherent color and depth picture
        while True:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue
            else:
                break

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        raw_depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        colorizer = rs.colorizer()
        depth_color_frame = colorizer.colorize(raw_depth_frame)
        depth_image = np.asanyarray(depth_color_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if self.run_record:
            #changing coloer format for record
            color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        raw_depth_image = np.asanyarray(raw_depth_frame.get_data())
        if not raw_depth_frame or not color_frame:
            return False, None, None, None
        return True, cv2.flip(depth_image, 1), cv2.flip(color_image, 1), cv2.flip(raw_depth_image, 1)

    def release(self):
        self.pipeline.stop()

    def keyUI(self):
            #get a key from main loop, if key is space, stop video if escape quit
        flag = False #if getting Esc key change to true
        key = cv2.waitKey(1)
        if key == 27:  # Esc key
            flag = True
        elif key == ord(" "):
            while True:
                key = cv2.waitKey(1)
                if key == ord(" "):
                    break
        elif key == ord("c"): #calibrate color
            self.calibrate_color = True #starting the calibrating process
        elif key == ord("f"):  # finish calibrate color
            self.calibrate_color = False
            self.calibrate_points = []  # initializing points for next calibration

        return flag


    def get_distance(self,x,y,depth_frame):
        distance = depth_frame[y,x]
        return distance

    def find_color(self, frame):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, self.objLower, self.objUpper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, None, iterations=1)

        #mask = cv2.dilate(mask, kernel)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        #mask = cv2.erode(mask, None, iterations=1)


        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        return mask, res

    def find_cnt(self, mask):
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # sort cnts so we can loop through the two biggest (the sticks hopefully)
        cnts = sorted(cnts,key = lambda x: cv2.contourArea(x), reverse = True)
        return cnts

    def findSticks(self, cnts):

        numSticks = min(len(cnts), 2)

        return numSticks

    def calibrateColor(self, hsv_color):
        # this function is called after the user pick a point for calibration, and setting lower and upper coordinates
        # for color

        delta = 0
        delta1 = 0

        self.calibrate_points.append(hsv_color)
        self.objLower = (np.array(self.calibrate_points)).min(0) - np.array([delta, delta1, delta1])
        self.objUpper = (np.array(self.calibrate_points)).max(0) + np.array([delta, delta1, delta1])

    #def nothing():
     #   pass

    def mouseRGB(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # checks mouse left button down condition
            #colorsB = self.color_frame[y, x, 0]
            #colorsG = self.color_frame[y, x, 1]
            #colorsR = self.color_frame[y, x, 2]
            colors = self.color_frame[y, x]
            hsv_color = np.squeeze(cv2.cvtColor(np.uint8([[colors]]), cv2.COLOR_BGR2HSV))
            if self.calibrate_color:
                self.calibrateColor(hsv_color)
                self.updateBar()



            print("HSV Format: ", hsv_color)
            print("BGR Format: ", colors)
            print("Coordinates of pixel: X: ", x, "Y: ", y)
