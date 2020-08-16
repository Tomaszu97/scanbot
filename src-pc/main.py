#!/bin/python3
import sys
from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox
from layout.layout import Ui_MainWindow
from pyside_material import apply_stylesheet
import serial.tools.list_ports
import serial
import datetime
import pyqtgraph as pg
import math
import time

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        #apply_stylesheet(app, theme="dark_blue.xml")
        self.setFixedWidth(1150)
        self.setFixedHeight(650)
        #self.setFixedSize(self.size())

        self.setupUi(self)
        self.plot_2d = pg.PlotWidget()
        self.plot_2d.setXRange(-100,100)
        self.plot_2d.setYRange(-100,100)
        self.plot_2d.showGrid(x = True, y = True, alpha = 0.3)
        # self.plot_2d.addItem(pg.ScatterPlotItem([50], [50], pen='#FF00FF', symbol='+'))
        #plot_3d = opengl stuff 
        self.grid_2d_layout.addWidget(self.plot_2d)
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
        self.forward_btn.pressed.connect(lambda : self.send_command("DRIVE:90,90#"))
        self.forward_btn.released.connect(lambda : self.send_command("DRIVE:0,0#"))
        self.backward_btn.pressed.connect(lambda : self.send_command("DRIVE:-90,-90#"))
        self.backward_btn.released.connect(lambda : self.send_command("DRIVE:0,0#"))
        self.turn_left_btn.pressed.connect(lambda : self.send_command("DRIVE:-90,90#"))
        self.turn_left_btn.released.connect(lambda : self.send_command("DRIVE:0,0#"))
        self.turn_right_btn.pressed.connect(lambda : self.send_command("DRIVE:90,-90#"))
        self.turn_right_btn.released.connect(lambda : self.send_command("DRIVE:0,0#"))
        self.tower_slider.valueChanged.connect(lambda val: self.send_command(f"ROTATE_TOWER:{val}#"))
        self.get_distance_btn.clicked.connect(lambda: self.send_command("GET_DISTANCE#"))
        self.get_azimuth_btn.clicked.connect(lambda: self.send_command("GET_AZIMUTH#"))
        self.beep_btn.clicked.connect(lambda: self.send_command("BEEP:10,3#"))
        self.print_siema_btn.clicked.connect(lambda: self.send_command("PRINT:SIEMA#"))
        self.kill_btn.clicked.connect(lambda: self.send_command("KILL#"))
        self.send_btn.clicked.connect(lambda: self.send_command(self.command_input.text()) )
        self.scan_btn.clicked.connect(self.scan)
        self.forward_step_btn.clicked.connect(lambda : self.send_command(f"MOVE:{self.drive_step_slider.value()}#"))
        self.backward_step_btn.clicked.connect(lambda : self.send_command(f"MOVE:{-self.drive_step_slider.value()}#"))
        self.turn_left_step_btn.clicked.connect(lambda : self.send_command(f"ROTATE:{-self.rotate_step_slider.value()}#"))
        self.turn_right_step_btn.clicked.connect(lambda : self.send_command(f"ROTATE:{self.rotate_step_slider.value()}#"))
        self.rotate_to_btn.clicked.connect(lambda : self.send_command(f"ROTATE_TO:{self.rotate_to_slider.value()}#"))
        
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
            if dist < 40 and dist > 10:
                ang = 10*idx
                self.place_pole(dist, ang, 0, 0)
                

if __name__ == "__main__":

    app = QApplication(sys.argv)
    MainWindow = MainWindow()
    MainWindow.show()
    sys.exit(app.exec_())


#TODO: response OK# for everything - also onyl use send with waiting for receive