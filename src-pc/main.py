#!/bin/python3
import sys
from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox
from layout.layout import Ui_MainWindow
from pyside_material import apply_stylesheet
import serial.tools.list_ports
import serial
import datetime
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
import math
import time
from robot import *
from timer_emiter import *
from statistics import median
from strings import *
##
## manually turn off gui bluetooth manager
## sudo rfkill unblock all
## 

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        self.robot = Robot(self.send_command)

        QMainWindow.__init__(self)
        #apply_stylesheet(app, theme="dark_blue.xml")
        self.setFixedWidth(1150)
        self.setFixedHeight(650)

        # 2D graph
        self.setupUi(self)
        self.plot_2d = pg.PlotWidget()
        self.plot_2d.setXRange(-300,300)
        self.plot_2d.setYRange(-300,300)
        self.plot_2d.showGrid(x = True, y = True, alpha = 0.3) ####################################
        # self.plot_2d.addItem(pg.ScatterPlotItem([50], [50], pen='#FF00FF', symbol='+'))
        self.grid_2d_layout.addWidget(self.plot_2d)

        # 3D graph
        self.plot_3d = gl.GLViewWidget()
        self.plot_3d.setCameraPosition(distance=10)
        gr = gl.GLGridItem()
        self.plot_3d.addItem(gr)
        # md = gl.MeshData.sphere(rows=4, cols=9)
        # m = gl.GLMeshItem(
        #     meshdata=md, smooth=False, drawFaces=False, drawEdges=True, edgeColor=(1, 0, 1, 1)
        # )
        # self.plot_3d.addItem(m)
        self.grid_3d_layout.addWidget(self.plot_3d)

        # Magnetometer calibration graph
        self.plot_3d_mag = gl.GLViewWidget()
        self.reset_mag_plot()
        # sp1 = gl.GLScatterPlotItem(pos=np.array( (0,0,0) ), color=np.array( (1,1,0,0.7) ))
        # self.plot_3d_mag.addItem(sp1)
        self.grid_mag_cal_layout.addWidget(self.plot_3d_mag)

        # Workers ##WARNING## use one worker for one task ##WARNING##
        self.calibration_timer = TimerEmiter(interval=0.2, repeat_count=50)

        # Controls' handlers
        self.connect_signals()
        
    def closeEvent(self, event):
        try:
            self.ser.close()
        except AttributeError:
            pass

    def make_message(self, title, text, informativetext=None):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setWindowTitle(title)
        msg.setText(text)
        if informativetext is not None:
            msg.setInformativeText(informativetext)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec()

    def connect_signals(self):
        self.serial_refresh_btn.clicked.connect(self.serial_refresh)
        self.serial_connect_btn.clicked.connect(self.serial_connect)
        self.forward_btn.pressed.connect(lambda : self.robot.drive(90,90))
        self.forward_btn.released.connect(lambda : self.robot.drive(0,0))
        self.backward_btn.pressed.connect(lambda : self.robot.drive(-90,-90))
        self.backward_btn.released.connect(lambda : self.robot.drive(0,0))
        self.turn_left_btn.pressed.connect(lambda : self.robot.drive(-90,90))
        self.turn_left_btn.released.connect(lambda : self.robot.drive(0,0))
        self.turn_right_btn.pressed.connect(lambda : self.robot.drive(90,-90))
        self.turn_right_btn.released.connect(lambda : self.robot.drive(0,0))
        self.tower_slider.valueChanged.connect(lambda val: self.robot.rotate_tower(val))
        self.get_distance_btn.clicked.connect(lambda: self.robot.get_distance())
        self.get_azimuth_btn.clicked.connect(lambda: self.robot.get_azimuth())
        self.beep_btn.clicked.connect(lambda: self.robot.beep(30,3))
        self.print_duck_btn.clicked.connect(lambda: self.robot.print(duckstr))
        self.kill_btn.clicked.connect(self.robot.kill)
        self.get_mag_btn.clicked.connect(self.robot.get_mag)
        self.get_mag_cal_btn.clicked.connect(self.robot.get_mag_cal)
        self.send_btn.clicked.connect(lambda: self.send_command(self.command_input.text()) )
        self.scan_btn.clicked.connect(self.scan)
        self.forward_step_btn.clicked.connect(lambda : self.robot.move(self.drive_step_slider.value()))
        self.backward_step_btn.clicked.connect(lambda : self.robot.move(-self.drive_step_slider.value()))
        self.turn_left_step_btn.clicked.connect(lambda : self.robot.rotate(-self.rotate_step_slider.value()))
        self.turn_right_step_btn.clicked.connect(lambda : self.robot.rotate(self.rotate_step_slider.value()))
        self.rotate_to_btn.clicked.connect(lambda : self.robot.rotate_to(self.rotate_step_slider.value()))
        self.clear_buffer_btn.clicked.connect(self.flush_serial_buffer)
        self.measure_btn.clicked.connect(self.calibration_timer.start)
        self.mag_cal_reset_btn.clicked.connect(self.reset_mag_cal)
        self.calibration_timer.tick_signal.connect(self.measure_handler)
        self.calibration_timer.tick_signal.connect(lambda val: self.calibration_progbar.setValue(val))

    def serial_refresh(self):
        self.serial_combo.clear()
        for comport in serial.tools.list_ports.comports():
            device = comport.device
            self.serial_combo.addItem(str(device))

    def serial_connect(self):
        if self.connection_status_label.text == "✔":
            self.ser.close()

        self.connection_status_label.setText("⌚")
        try:
            self.ser = serial.Serial(
                port=f"{self.serial_combo.currentText()}",
                baudrate=38400,
                timeout=2,
                parity=serial.PARITY_EVEN,
                rtscts=1,
            )
            
            self.connection_status_label.setText("✔")
        except serial.SerialException as e:
            self.connection_status_label.setText("❌")
            self.make_message("Error", "Serial connection failed.", str(e))

        x = self.ser.read_until().decode()
        toprint = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] RECEIVED> {x}'
        self.command_output_long.setText(self.command_output_long.toPlainText() + toprint)
        self.command_output_short.setText(self.command_output_short.toPlainText() + toprint)

    def flush_serial_buffer(self):
        self.command_output_long.setText("")
        self.command_output_short.setText("")
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def scroll_outputs(self):
        sb=self.command_output_long.verticalScrollBar()
        sb.setValue(sb.maximum())
        sb=self.command_output_short.verticalScrollBar()
        sb.setValue(sb.maximum())

    def send_command(self, command):
        self.ser.write(command.encode('utf-8'))
        toprint = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] SENT> {command}\r\n'
        self.command_output_long.setText(self.command_output_long.toPlainText() + toprint)
        self.command_output_short.setText(self.command_output_short.toPlainText() + toprint)
        self.scroll_outputs()
        
        x = self.ser.read_until().decode()
        toprint = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] RECEIVED> {x}'
        self.command_output_long.setText(self.command_output_long.toPlainText() + toprint)
        self.command_output_short.setText(self.command_output_short.toPlainText() + toprint)

        self.scroll_outputs()
        x = x.replace("\r\n", "")
        return x

    def place_pole(self, distance, angle ,x_pos=0, y_pos=0, color='#FF00FF'):
        angle = math.radians(angle)
        x = distance * math.cos(angle) + x_pos
        y = distance * math.sin(angle) + y_pos
        self.plot_2d.addItem(pg.ScatterPlotItem([x], [y], pen=color, symbol='+'))

    def scan(self):
        data = self.send_command(f"SCAN#").split(",")[:-1]
        for i in range(len(data)):
            data[i] = int(data[i])

        self.plot_2d.clear()
        for idx, dist in enumerate(data):
            if dist >= 15:
                ang = 1*idx
                self.place_pole(dist, ang, 0, 0)
                
    def measure_handler(self):
        x,y,z = [],[],[]
        for i in range(3):
            _x, _y, _z = self.robot.get_mag()
            x.append(_x)
            y.append(_y)
            z.append(_z)
        x = median(x)
        y = median(y)
        z = median(z)
        
        if x < int(self.min_x_label.text()):
            self.min_x_label.setText(str(x))
        if x > int(self.max_x_label.text()):
            self.max_x_label.setText(str(x))
        if y < int(self.min_y_label.text()):
            self.min_y_label.setText(str(y))
        if y > int(self.max_y_label.text()):
            self.max_y_label.setText(str(y))
        if z < int(self.min_z_label.text()):
            self.min_z_label.setText(str(z))
        if z > int(self.max_z_label.text()):
            self.max_z_label.setText(str(z))

        self.offset_x_label.setText( str( int((int(self.min_x_label.text()) + int(self.max_x_label.text()))/2)) )
        self.offset_y_label.setText( str( int((int(self.min_y_label.text()) + int(self.max_y_label.text()))/2)) )
        self.offset_z_label.setText( str( int((int(self.min_z_label.text()) + int(self.max_z_label.text()))/2)) )

        item = gl.GLScatterPlotItem( pos=np.array((x,y,z)), color=np.array((1,1,0,0.7)), size=5 )
        self.plot_3d_mag.addItem(item)

    def reset_mag_plot(self):
        toremove = []
        for i in self.plot_3d_mag.items:
            toremove.append(i)
        for i in toremove:
            self.plot_3d_mag.removeItem(i)
        
        self.plot_3d_mag.setCameraPosition(pos=pg.Vector(0,0,0), distance=40000, azimuth=45, elevation=30)
        gr1 = gl.GLGridItem(color=(255,0,0,76.5))
        gr1.setSize(10000, 10000, 10000)
        gr1.setSpacing(1000,1000,1000)
        gr2 = gl.GLGridItem(color=(0,255,0,76.5))
        gr2.setSize(10000, 10000, 10000)
        gr2.setSpacing(1000,1000,1000)
        gr2.rotate(90,1,0,0)
        gr3 = gl.GLGridItem(color=(0,0,255,76.5))
        gr3.setSize(10000, 10000, 10000)
        gr3.setSpacing(1000,1000,1000)
        gr3.rotate(90,0,1,0)
        self.plot_3d_mag.addItem(gr1)
        self.plot_3d_mag.addItem(gr2)
        self.plot_3d_mag.addItem(gr3)

    def reset_mag_cal(self):
        self.min_x_label.setText(str(0))
        self.max_x_label.setText(str(0))
        self.offset_x_label.setText(str(0))
        self.min_y_label.setText(str(0))
        self.max_y_label.setText(str(0))
        self.offset_y_label.setText(str(0))
        self.min_z_label.setText(str(0))
        self.max_z_label.setText(str(0))
        self.offset_z_label.setText(str(0))
        self.reset_mag_plot()

if __name__ == "__main__":

    app = QApplication(sys.argv)
    MainWindow = MainWindow()
    MainWindow.show()
    sys.exit(app.exec_())
