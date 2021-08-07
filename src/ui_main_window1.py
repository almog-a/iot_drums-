# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_main_window1.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1332, 611)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.start_bt = QtWidgets.QPushButton(self.centralwidget)
        self.start_bt.setGeometry(QtCore.QRect(240, 530, 331, 23))
        self.start_bt.setObjectName("start_bt")
        self.songVolumeSlider = QtWidgets.QSlider(self.centralwidget)
        self.songVolumeSlider.setGeometry(QtCore.QRect(50, 530, 160, 22))
        self.songVolumeSlider.setProperty("value", 50)
        self.songVolumeSlider.setOrientation(QtCore.Qt.Horizontal)
        self.songVolumeSlider.setObjectName("songVolumeSlider")
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(0, 500, 1331, 20))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.left_bt = QtWidgets.QPushButton(self.centralwidget)
        self.left_bt.setGeometry(QtCore.QRect(890, 530, 75, 23))
        self.left_bt.setText("")
        self.left_bt.setObjectName("left_bt")
        self.right_bt = QtWidgets.QPushButton(self.centralwidget)
        self.right_bt.setGeometry(QtCore.QRect(1000, 530, 75, 23))
        self.right_bt.setText("")
        self.right_bt.setObjectName("right_bt")
        self.bt_vol_icon = QtWidgets.QPushButton(self.centralwidget)
        self.bt_vol_icon.setGeometry(QtCore.QRect(20, 530, 21, 23))
        self.bt_vol_icon.setAutoFillBackground(False)
        self.bt_vol_icon.setText("")
        self.bt_vol_icon.setObjectName("bt_vol_icon")
        self.checkBox_MIDI = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox_MIDI.setGeometry(QtCore.QRect(770, 530, 70, 17))
        self.checkBox_MIDI.setObjectName("checkBox_MIDI")
        self.checkBox_Arduino = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox_Arduino.setGeometry(QtCore.QRect(680, 530, 70, 17))
        self.checkBox_Arduino.setObjectName("checkBox_Arduino")
        self.stream2 = QtWidgets.QLabel(self.centralwidget)
        self.stream2.setGeometry(QtCore.QRect(662, 20, 641, 480))
        self.stream2.setObjectName("stream2")
        self.stream1 = QtWidgets.QLabel(self.centralwidget)
        self.stream1.setGeometry(QtCore.QRect(20, 20, 641, 480))
        self.stream1.setObjectName("stream1")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1332, 21))
        self.menubar.setObjectName("menubar")
        self.menuSet_Up = QtWidgets.QMenu(self.menubar)
        self.menuSet_Up.setObjectName("menuSet_Up")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionCalibrate = QtWidgets.QAction(MainWindow)
        self.actionCalibrate.setObjectName("actionCalibrate")
        self.actionFinish_Sticks_Calibration = QtWidgets.QAction(MainWindow)
        self.actionFinish_Sticks_Calibration.setObjectName("actionFinish_Sticks_Calibration")
        self.actionPlay_Song = QtWidgets.QAction(MainWindow)
        self.actionPlay_Song.setObjectName("actionPlay_Song")
        self.actionCalibrate_Leg = QtWidgets.QAction(MainWindow)
        self.actionCalibrate_Leg.setObjectName("actionCalibrate_Leg")
        self.actionSave_Calibration = QtWidgets.QAction(MainWindow)
        self.actionSave_Calibration.setObjectName("actionSave_Calibration")
        self.menuSet_Up.addAction(self.actionCalibrate)
        self.menuSet_Up.addAction(self.actionCalibrate_Leg)
        self.menuSet_Up.addAction(self.actionFinish_Sticks_Calibration)
        self.menuSet_Up.addAction(self.actionSave_Calibration)
        self.menuSet_Up.addAction(self.actionPlay_Song)
        self.menubar.addAction(self.menuSet_Up.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.start_bt.setText(_translate("MainWindow", "start"))
        self.checkBox_MIDI.setText(_translate("MainWindow", "MIDI"))
        self.checkBox_Arduino.setText(_translate("MainWindow", "Arduino"))
        self.stream2.setText(_translate("MainWindow", "TextLabel"))
        self.stream1.setText(_translate("MainWindow", "TextLabel"))
        self.menuSet_Up.setTitle(_translate("MainWindow", "Set Up"))
        self.actionCalibrate.setText(_translate("MainWindow", "Calibrate Sticks"))
        self.actionCalibrate.setShortcut(_translate("MainWindow", "C"))
        self.actionFinish_Sticks_Calibration.setText(_translate("MainWindow", "Finish Calibration"))
        self.actionFinish_Sticks_Calibration.setShortcut(_translate("MainWindow", "F"))
        self.actionPlay_Song.setText(_translate("MainWindow", "Play Song"))
        self.actionCalibrate_Leg.setText(_translate("MainWindow", "Calibrate Leg"))
        self.actionCalibrate_Leg.setShortcut(_translate("MainWindow", "X"))
        self.actionSave_Calibration.setText(_translate("MainWindow", "Save Calibration"))
        self.actionSave_Calibration.setShortcut(_translate("MainWindow", "S"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
