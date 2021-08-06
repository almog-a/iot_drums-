"""
In this example, we demonstrate how to create simple camera viewer using Opencv3 and PyQt5

Author: Berrouba.A
Last edited: 21 Feb 2018
"""

# import system module
import sys

# import some PyQt5 modules
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer
from PyQt5 import QtMultimedia

# import Opencv module
import cv2

from ui_main_window1 import *
import iot_drums as iot


class MainWindow(QMainWindow):
    # class constructor
    def __init__(self):
        # call QWidget constructor
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # create a timer
        self.timer = QTimer()
        # set timer timeout callback function
        self.timer.timeout.connect(self.viewCam)
        # set control_bt callback clicked  function
        self.ui.start_bt.clicked.connect(self.controlTimer)
        self.iot = self.init_iot_drums()
        self.ui.actionPlay_Song.triggered.connect(self.playSong)
        self.ui.actionCalibrate.triggered.connect(self.iot.vs.calibrate_stics)


    def init_iot_drums(self):
        return iot.iot_drums()


            # view camera
    # def viewCam(self):
    #     # read image in BGR format
    #     ret, image = self.cap.read()
    #     # convert image to RGB format
    #     image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #     # get image infos
    #     height, width, channel = image.shape
    #     step = channel * width
    #     # create QImage from image
    #     qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
    #     # show image in img_label
    #     self.ui.image_label.setPixmap(QPixmap.fromImage(qImg))

    def viewCam(self):
        self.iot.iteration()
        color_frame = self.iot.vs.color_frame

        height, width, channel = color_frame.shape
        step = channel * width
        # create QImage from image
        qImg = QImage(color_frame.data, width, height, step, QImage.Format_RGB888)
        # show image in img_label
        self.ui.stream1.setPixmap(QPixmap.fromImage(qImg))
        self.ui.stream1.mousePressEvent = self.getPixel

    def getPixel(self, event):
        x = event.pos().x()
        y = event.pos().y()
        # c = self.img.pixel(x, y)  # color code (integer): 3235912
        # # depending on what kind of value you like (arbitary examples)
        # c_qobj = QColor(c)  # color object
        # c_rgb = QColor(c).getRgb()  # 8bit RGBA: (255, 23, 0, 255)
        # c_rgbf = QColor(c).getRgbf()  # RGBA float: (1.0, 0.3123, 0.0, 1.0)
        print (x, y)#, c_rgb)


    # start/stop timer
    def controlTimer(self):
        # if timer is stopped
        if not self.timer.isActive():
            # create video capture
            self.cap = cv2.VideoCapture(0)
            # start timer
            self.timer.start()
            # update control_bt text
            self.ui.start_bt.setText("Stop")
        # if timer is started
        else:
            # stop timer
            self.timer.stop()
            # release video capture
            self.cap.release()
            # update control_bt text
            self.ui.start_bt.setText("Start")

    def playSong(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        files, _ = QFileDialog.getOpenFileNames(self, "QFileDialog.getOpenFileNames()", "",
                                                "All Files (*);;Python Files (*.py)", options=options)
        if files:
            print(files)
            url = QtCore.QUrl.fromLocalFile(*files)
            content = QtMultimedia.QMediaContent(url)
            player = QtMultimedia.QMediaPlayer()
            player.setMedia(content)
            player.play()



if __name__ == '__main__':
    app = QApplication(sys.argv)

    # create and show mainWindow
    mainWindow = MainWindow()
    mainWindow.show()

    sys.exit(app.exec_())