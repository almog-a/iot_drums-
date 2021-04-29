import pyrealsense2 as rs
import numpy as np
import cv2
#--
class DepthCamera:
    def __init__(self, run_record =False ,file_name='NO_FILE'):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.run_record = run_record
        if not run_record:
        # Get device product line for setting a supporting resolution
            pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            pipeline_profile = self.config.resolve(pipeline_wrapper)
            device = pipeline_profile.get_device()
            device_product_line = str(device.get_info(rs.camera_info.product_line))
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
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
        return True, depth_image, color_image, raw_depth_image

    def release(self):
        self.pipeline.stop()

    def keyUI(self, key):
            #get a key from main loop, if key is space, stop video
        if key == ord(" "):
            while True:
                key = cv2.waitKey(1)
                if key == ord(" "):
                    break

    def get_distance(self,x,y,depth_frame):
        distance = depth_frame[y,x]
        return distance
