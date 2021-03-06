"""
In this example, we demonstrate how to create simple camera viewer using Opencv3 and PyQt5

Author: Berrouba.A
Last edited: 21 Feb 2018
"""

# import system module
import sys

# import some PyQt5 modules
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog ,QSlider, QCommonStyle, QStyle, QDialog
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer
from PyQt5 import QtMultimedia

# import Opencv module
import cv2

from ui_main_window1 import *
from ui_opening_window import *
import iot_drums as iot


class OpeningWindow(QDialog):
    # class constructor
    def __init__(self):
        # call QWidget constructor
        super().__init__()
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.setWindowTitle("Air Drums")

        self.ui.pushButton.clicked.connect(self.MoveToMainWindow)

    def MoveToMainWindow(self):
        mainWindow = MainWindow()
        size = mainWindow.size()
        widget.addWidget(mainWindow)
        widget.setCurrentIndex(widget.currentIndex() + 1)
        widget.resize(size)


class MainWindow(QMainWindow):
    # class constructor
    def __init__(self):
        # call QWidget constructor
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("Air Drums")

        # create a timer
        self.timer = QTimer()
        # set timer timeout callback function
        self.timer.timeout.connect(self.putStream)
        # set control_bt callback clicked  function
        self.iot = self.init_iot_drums()
        self.player = QtMultimedia.QMediaPlayer()
        self.initGui()

    def initGui(self):
        self.ui.start_bt.clicked.connect(self.controlTimer)
        self.ui.actionPlay_Song.triggered.connect(self.playSong)
        self.ui.actionCalibrate.triggered.connect(self.iot.vs.calibrate_sticks)
        self.ui.songVolumeSlider.valueChanged[int].connect(self.changeVolume)
        self.setWindowIcon(QtGui.QIcon('icon.png'))
        self.ui.bt_vol_icon.setIcon(self.style().standardIcon(QStyle.SP_MediaVolume))
        self.ui.actionCalibrate_Leg.triggered.connect(self.iot.vs.calibrate_leg)
        self.ui.actionSave_Calibration.triggered.connect(self.iot.vs.save_calibration)
        self.ui.actionFinish_Sticks_Calibration.triggered.connect(self.iot.vs.finish_calibrate)
        self.ui.left_bt.setIcon(self.style().standardIcon(QStyle.SP_ArrowLeft))
        self.ui.right_bt.setIcon(self.style().standardIcon(QStyle.SP_ArrowRight))
        self.ui.left_bt.clicked.connect(lambda: self.changeStream(-1))
        self.ui.right_bt.clicked.connect(lambda:  self.changeStream(1))
        self.ui.checkBox_Arduino.stateChanged.connect(lambda:  self.set_Arduino(self.ui.checkBox_Arduino.isChecked()))
        self.ui.checkBox_MIDI.stateChanged.connect(lambda: self.set_MIDI(self.ui.checkBox_MIDI.isChecked()))
        self.ui.checkBox_Depth.stateChanged.connect(lambda: self.iot.pm.set_isDepthOn(self.ui.checkBox_Depth.isChecked()))
        self.ui.checkBox_circle_sticks.stateChanged.connect(lambda: self.iot.setDrawSticks(self.ui.checkBox_circle_sticks.isChecked()))


    def set_MIDI(self, MIDI_state):
        self.iot.pm.is_midi = MIDI_state
        if MIDI_state:
            self.iot.pm.open_port()
        else:
            self.iot.pm.close_port()

    def set_Arduino(self, Arduino_state):
        if Arduino_state:
            self.iot.pm.arduino_config(Arduino_state)
        else:
            self.iot.pm.close_arduino()


    def init_iot_drums(self):
        self.stream_num=0
        self.streams_name = ["Sticks Stream", "Leg Stream", "Depth Stream"]
        return iot.iot_drums()

    def changeVolume(self, value):
        self.player.setVolume(value)

    def changeStream(self,x):
        self.stream_num = (self.stream_num+x) % 3
        self.ui.stream_label.setText(self.streams_name[self.stream_num])


    def putStream(self):
        color_frame, secondary_frames = self.iot.iteration()
        color_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
        sec_frame = cv2.cvtColor(secondary_frames[self.stream_num], cv2.COLOR_BGR2RGB)

        height, width, channel = color_frame.shape
        step = channel * width
        # create QImage from image
        qImg_stream1 = QImage(color_frame.data, width, height, step, QImage.Format_RGB888)
        qImg_stream2 = QImage(sec_frame.data, width, height, step, QImage.Format_RGB888)
        # show image in img_label
        self.ui.stream1.setPixmap(QPixmap.fromImage(qImg_stream1))
        self.ui.stream1.mousePressEvent = self.getPixel

        self.ui.stream2.setPixmap(QPixmap.fromImage(qImg_stream2))
        #for expanding
        #self.ui.stream1.setScaledContents(True)
        #self.ui.stream1.setSizePolicy(QtWidgets.QSizePolicy.Ignored,QtWidgets.QSizePolicy.Ignored)



    def getPixel(self, event):
        x = event.pos().x()
        y = event.pos().y()
        self.iot.vs.calibrate_callback(x,y)


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
            self.player.setMedia(content)
            self.player.play()
            self.player.setVolume(int(self.ui.songVolumeSlider.value()))


if __name__ == '__main__':
    app = QApplication(sys.argv)

    # create and show mainWindow
    widget = QtWidgets.QStackedWidget()
    #mainWindow = MainWindow()
    openingWindow = OpeningWindow()
    widget.addWidget(openingWindow)
    #widget.addWidget(mainWindow)
    widget.resize(openingWindow.size())
    widget.show()

    #mainWindow.show()

    sys.exit(app.exec_())