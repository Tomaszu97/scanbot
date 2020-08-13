# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'layout.ui'
##
## Created by: Qt User Interface Compiler version 5.15.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import (
    QCoreApplication,
    QDate,
    QDateTime,
    QMetaObject,
    QObject,
    QPoint,
    QRect,
    QSize,
    QTime,
    QUrl,
    Qt,
)
from PySide2.QtGui import (
    QBrush,
    QColor,
    QConicalGradient,
    QCursor,
    QFont,
    QFontDatabase,
    QIcon,
    QKeySequence,
    QLinearGradient,
    QPalette,
    QPainter,
    QPixmap,
    QRadialGradient,
)
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1500, 800)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.serial_refresh_btn = QPushButton(self.centralwidget)
        self.serial_refresh_btn.setObjectName(u"serial_refresh_btn")
        self.serial_refresh_btn.setGeometry(QRect(160, 40, 131, 41))
        self.serial_combo = QComboBox(self.centralwidget)
        self.serial_combo.setObjectName(u"serial_combo")
        self.serial_combo.setGeometry(QRect(10, 41, 131, 41))
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(10, 20, 101, 20))
        self.serial_connect_btn = QPushButton(self.centralwidget)
        self.serial_connect_btn.setObjectName(u"serial_connect_btn")
        self.serial_connect_btn.setGeometry(QRect(310, 40, 131, 41))
        self.command_output_short = QTextBrowser(self.centralwidget)
        self.command_output_short.setObjectName(u"command_output_short")
        self.command_output_short.setGeometry(QRect(10, 520, 481, 91))
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(10, 430, 101, 20))
        self.label_3 = QLabel(self.centralwidget)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(10, 500, 101, 20))
        self.forward_btn = QPushButton(self.centralwidget)
        self.forward_btn.setObjectName(u"forward_btn")
        self.forward_btn.setGeometry(QRect(80, 160, 41, 41))
        self.backward_btn = QPushButton(self.centralwidget)
        self.backward_btn.setObjectName(u"backward_btn")
        self.backward_btn.setGeometry(QRect(80, 210, 41, 41))
        self.turn_left_btn = QPushButton(self.centralwidget)
        self.turn_left_btn.setObjectName(u"turn_left_btn")
        self.turn_left_btn.setGeometry(QRect(30, 210, 41, 41))
        self.turn_right_btn = QPushButton(self.centralwidget)
        self.turn_right_btn.setObjectName(u"turn_right_btn")
        self.turn_right_btn.setGeometry(QRect(130, 210, 41, 41))
        self.get_azimuth_btn = QPushButton(self.centralwidget)
        self.get_azimuth_btn.setObjectName(u"get_azimuth_btn")
        self.get_azimuth_btn.setGeometry(QRect(210, 170, 111, 41))
        self.beep_btn = QPushButton(self.centralwidget)
        self.beep_btn.setObjectName(u"beep_btn")
        self.beep_btn.setGeometry(QRect(210, 220, 111, 41))
        self.get_distance_btn = QPushButton(self.centralwidget)
        self.get_distance_btn.setObjectName(u"get_distance_btn")
        self.get_distance_btn.setGeometry(QRect(210, 120, 111, 41))
        self.kill_btn = QPushButton(self.centralwidget)
        self.kill_btn.setObjectName(u"kill_btn")
        self.kill_btn.setGeometry(QRect(330, 170, 111, 41))
        self.print_siema_btn = QPushButton(self.centralwidget)
        self.print_siema_btn.setObjectName(u"print_siema_btn")
        self.print_siema_btn.setGeometry(QRect(330, 120, 111, 41))
        self.send_button = QPushButton(self.centralwidget)
        self.send_button.setObjectName(u"send_button")
        self.send_button.setGeometry(QRect(510, 450, 111, 41))
        self.command_input = QLineEdit(self.centralwidget)
        self.command_input.setObjectName(u"command_input")
        self.command_input.setGeometry(QRect(10, 450, 481, 41))
        self.line = QFrame(self.centralwidget)
        self.line.setObjectName(u"line")
        self.line.setGeometry(QRect(10, 90, 621, 21))
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)
        self.label_4 = QLabel(self.centralwidget)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(10, 100, 101, 20))
        self.line_2 = QFrame(self.centralwidget)
        self.line_2.setObjectName(u"line_2")
        self.line_2.setGeometry(QRect(10, 290, 621, 21))
        self.line_2.setFrameShape(QFrame.HLine)
        self.line_2.setFrameShadow(QFrame.Sunken)
        self.label_5 = QLabel(self.centralwidget)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(10, 0, 101, 20))
        self.tab_container = QTabWidget(self.centralwidget)
        self.tab_container.setObjectName(u"tab_container")
        self.tab_container.setGeometry(QRect(639, 10, 851, 781))
        self.tab_2d = QWidget()
        self.tab_2d.setObjectName(u"tab_2d")
        self.gridLayoutWidget = QWidget(self.tab_2d)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(0, 0, 841, 751))
        self.grid_2d_layout = QGridLayout(self.gridLayoutWidget)
        self.grid_2d_layout.setObjectName(u"grid_2d_layout")
        self.grid_2d_layout.setContentsMargins(0, 0, 0, 0)
        self.tab_container.addTab(self.tab_2d, "")
        self.tab_3d = QWidget()
        self.tab_3d.setObjectName(u"tab_3d")
        self.gridLayoutWidget_2 = QWidget(self.tab_3d)
        self.gridLayoutWidget_2.setObjectName(u"gridLayoutWidget_2")
        self.gridLayoutWidget_2.setGeometry(QRect(0, 0, 841, 751))
        self.grid_3d_layout = QGridLayout(self.gridLayoutWidget_2)
        self.grid_3d_layout.setObjectName(u"grid_3d_layout")
        self.grid_3d_layout.setContentsMargins(0, 0, 0, 0)
        self.tab_container.addTab(self.tab_3d, "")
        self.tab_raw = QWidget()
        self.tab_raw.setObjectName(u"tab_raw")
        self.command_output_long = QTextBrowser(self.tab_raw)
        self.command_output_long.setObjectName(u"command_output_long")
        self.command_output_long.setGeometry(QRect(0, 0, 841, 751))
        self.tab_container.addTab(self.tab_raw, "")
        self.label_6 = QLabel(self.centralwidget)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(10, 410, 101, 20))
        self.tower_slider = QSlider(self.centralwidget)
        self.tower_slider.setObjectName(u"tower_slider")
        self.tower_slider.setGeometry(QRect(30, 260, 141, 22))
        self.tower_slider.setOrientation(Qt.Horizontal)
        self.line_3 = QFrame(self.centralwidget)
        self.line_3.setObjectName(u"line_3")
        self.line_3.setGeometry(QRect(10, 390, 621, 21))
        self.line_3.setFrameShape(QFrame.HLine)
        self.line_3.setFrameShadow(QFrame.Sunken)
        self.label_7 = QLabel(self.centralwidget)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(10, 300, 151, 20))
        self.go_btn = QPushButton(self.centralwidget)
        self.go_btn.setObjectName(u"go_btn")
        self.go_btn.setGeometry(QRect(210, 330, 151, 51))
        self.algorithm_combo = QComboBox(self.centralwidget)
        self.algorithm_combo.addItem("")
        self.algorithm_combo.addItem("")
        self.algorithm_combo.setObjectName(u"algorithm_combo")
        self.algorithm_combo.setGeometry(QRect(10, 340, 161, 41))
        self.label_8 = QLabel(self.centralwidget)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setGeometry(QRect(10, 320, 47, 13))
        self.connection_status_label = QLabel(self.centralwidget)
        self.connection_status_label.setObjectName(u"connection_status_label")
        self.connection_status_label.setGeometry(QRect(460, 30, 81, 61))
        font = QFont()
        font.setPointSize(28)
        font.setBold(False)
        font.setWeight(50)
        font.setKerning(False)
        font.setStyleStrategy(QFont.PreferDefault)
        self.connection_status_label.setFont(font)
        self.connection_status_label.setAlignment(Qt.AlignCenter)
        self.connection_status_label.setMargin(0)
        MainWindow.setCentralWidget(self.centralwidget)
        QWidget.setTabOrder(self.serial_combo, self.serial_refresh_btn)
        QWidget.setTabOrder(self.serial_refresh_btn, self.serial_connect_btn)
        QWidget.setTabOrder(self.serial_connect_btn, self.forward_btn)
        QWidget.setTabOrder(self.forward_btn, self.turn_left_btn)
        QWidget.setTabOrder(self.turn_left_btn, self.backward_btn)
        QWidget.setTabOrder(self.backward_btn, self.turn_right_btn)
        QWidget.setTabOrder(self.turn_right_btn, self.get_distance_btn)
        QWidget.setTabOrder(self.get_distance_btn, self.get_azimuth_btn)
        QWidget.setTabOrder(self.get_azimuth_btn, self.beep_btn)
        QWidget.setTabOrder(self.beep_btn, self.print_siema_btn)
        QWidget.setTabOrder(self.print_siema_btn, self.kill_btn)
        QWidget.setTabOrder(self.kill_btn, self.algorithm_combo)
        QWidget.setTabOrder(self.algorithm_combo, self.go_btn)
        QWidget.setTabOrder(self.go_btn, self.command_input)
        QWidget.setTabOrder(self.command_input, self.send_button)
        QWidget.setTabOrder(self.send_button, self.command_output_short)
        QWidget.setTabOrder(self.command_output_short, self.tab_container)
        QWidget.setTabOrder(self.tab_container, self.command_output_long)
        QWidget.setTabOrder(self.command_output_long, self.tower_slider)

        self.retranslateUi(MainWindow)

        self.tab_container.setCurrentIndex(2)

        QMetaObject.connectSlotsByName(MainWindow)

    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(
            QCoreApplication.translate(
                "MainWindow", u"ScanBot Control Application", None
            )
        )
        self.serial_refresh_btn.setText(
            QCoreApplication.translate("MainWindow", u"refresh", None)
        )
        self.label.setText(
            QCoreApplication.translate("MainWindow", u"choose serial port", None)
        )
        self.serial_connect_btn.setText(
            QCoreApplication.translate("MainWindow", u"connect", None)
        )
        self.label_2.setText(
            QCoreApplication.translate("MainWindow", u"command line", None)
        )
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"output", None))
        self.forward_btn.setText(QCoreApplication.translate("MainWindow", u"^", None))
        self.backward_btn.setText(
            QCoreApplication.translate("MainWindow", u"\\/", None)
        )
        self.turn_left_btn.setText(QCoreApplication.translate("MainWindow", u"<", None))
        self.turn_right_btn.setText(
            QCoreApplication.translate("MainWindow", u">", None)
        )
        self.get_azimuth_btn.setText(
            QCoreApplication.translate("MainWindow", u"GET_AZIMUTH", None)
        )
        self.beep_btn.setText(QCoreApplication.translate("MainWindow", u"BEEP", None))
        self.get_distance_btn.setText(
            QCoreApplication.translate("MainWindow", u"GET_DISTANCE", None)
        )
        self.kill_btn.setText(QCoreApplication.translate("MainWindow", u"KILL", None))
        self.print_siema_btn.setText(
            QCoreApplication.translate("MainWindow", u"PRINT SIEMA", None)
        )
        self.send_button.setText(
            QCoreApplication.translate("MainWindow", u"SEND", None)
        )
        self.label_4.setText(
            QCoreApplication.translate("MainWindow", u"MANUAL CONTROL", None)
        )
        self.label_5.setText(
            QCoreApplication.translate("MainWindow", u"CONNECTION", None)
        )
        self.tab_container.setTabText(
            self.tab_container.indexOf(self.tab_2d),
            QCoreApplication.translate("MainWindow", u"2D view", None),
        )
        self.tab_container.setTabText(
            self.tab_container.indexOf(self.tab_3d),
            QCoreApplication.translate("MainWindow", u"3D view", None),
        )
        self.tab_container.setTabText(
            self.tab_container.indexOf(self.tab_raw),
            QCoreApplication.translate("MainWindow", u"raw output", None),
        )
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"CONSOLE", None))
        self.label_7.setText(
            QCoreApplication.translate("MainWindow", u"AUTONOMOUS DRIVE", None)
        )
        self.go_btn.setText(QCoreApplication.translate("MainWindow", u"GO", None))
        self.algorithm_combo.setItemText(
            0, QCoreApplication.translate("MainWindow", u"follow_the_wall", None)
        )
        self.algorithm_combo.setItemText(
            1, QCoreApplication.translate("MainWindow", u"ping_pong", None)
        )

        self.label_8.setText(
            QCoreApplication.translate("MainWindow", u"algorithm", None)
        )
        self.connection_status_label.setText(
            QCoreApplication.translate(
                "MainWindow",
                u'<html><head/><body><p align="center">\u274c</p></body></html>',
                None,
            )
        )

    # retranslateUi

