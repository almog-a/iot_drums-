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
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.start_bt = QtWidgets.QPushButton(self.centralwidget)
        self.start_bt.setGeometry(QtCore.QRect(200, 520, 331, 23))
        self.start_bt.setObjectName("start_bt")
        self.stream1 = QtWidgets.QLabel(self.centralwidget)
        self.stream1.setGeometry(QtCore.QRect(100, 10, 571, 491))
        self.stream1.setObjectName("stream1")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 21))
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
        self.menuSet_Up.addAction(self.actionCalibrate)
        self.menuSet_Up.addAction(self.actionFinish_Sticks_Calibration)
        self.menuSet_Up.addAction(self.actionPlay_Song)
        self.menubar.addAction(self.menuSet_Up.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.start_bt.setText(_translate("MainWindow", "start"))
        self.stream1.setText(_translate("MainWindow", "TextLabel"))
        self.menuSet_Up.setTitle(_translate("MainWindow", "Set Up"))
        self.actionCalibrate.setText(_translate("MainWindow", "Calibrate Sticks"))
        self.actionCalibrate.setShortcut(_translate("MainWindow", "C"))
        self.actionFinish_Sticks_Calibration.setText(_translate("MainWindow", "Finish Sticks Calibration"))
        self.actionFinish_Sticks_Calibration.setShortcut(_translate("MainWindow", "F"))
        self.actionPlay_Song.setText(_translate("MainWindow", "Play Song"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
