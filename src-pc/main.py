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
from math import radians, degrees, sin, cos, sqrt, asin
import time
from robot import *
from timer_emiter import *
from statistics import median
from strings import *
import randomcolor


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        # apply_stylesheet(app, theme="dark_blue.xml")
        self.setFixedWidth(1150)
        self.setFixedHeight(650)

        # Data / objects
        self.robot = Robot(self.send_command)
        self.randcol = lambda: randomcolor.RandomColor().generate(
            luminosity="bright", count=1
        )
        self.pole_plot_points = np.zeros((0, 2))
        self.pole_plot_points_colors = []
        self.mag_cal_points = np.zeros((0, 2))
        self.mag_cal_points_colors = []

        # 2D graph
        self.setupUi(self)
        self.pole_plot = pg.PlotWidget()
        self.pole_plot_layout.addWidget(self.pole_plot)
        self.reset_pole_plot()

        # 3D graph
        self.pole_plot_3d = gl.GLViewWidget()
        self.pole_plot_3d.setCameraPosition(distance=10)
        gr = gl.GLGridItem()
        self.pole_plot_3d.addItem(gr)
        self.pole_plot_3d_layout.addWidget(self.pole_plot_3d)

        # Magnetometer calibration graph
        self.mag_cal_plot = pg.PlotWidget()
        self.mag_cal_plot.setAspectLocked()
        self.mag_cal_plot_layout.addWidget(self.mag_cal_plot)
        self.reset_mag_cal_plot()

        # Workers ##WARNING## use one worker for one task ##WARNING##
        self.calibration_timer = TimerEmiter(interval=0.2, repeat_count=50)

        # Controls' handlers
        self.connect_signals()

        # Search for serial ports
        self.serial_refresh()

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
        self.forward_btn.pressed.connect(lambda: self.robot.drive(90, 90))
        self.forward_btn.released.connect(lambda: self.robot.drive(0, 0))
        self.backward_btn.pressed.connect(lambda: self.robot.drive(-90, -90))
        self.backward_btn.released.connect(lambda: self.robot.drive(0, 0))
        self.turn_left_btn.pressed.connect(lambda: self.robot.drive(-90, 90))
        self.turn_left_btn.released.connect(lambda: self.robot.drive(0, 0))
        self.turn_right_btn.pressed.connect(lambda: self.robot.drive(90, -90))
        self.turn_right_btn.released.connect(lambda: self.robot.drive(0, 0))
        self.tower_slider.valueChanged.connect(lambda val: self.robot.rotate_tower(val))
        self.get_distance_btn.clicked.connect(lambda: self.robot.get_distance())
        self.get_azimuth_btn.clicked.connect(lambda: self.robot.get_azimuth())
        self.beep_btn.clicked.connect(lambda: self.robot.beep(30, 3))
        self.print_duck_btn.clicked.connect(lambda: self.robot.print(duckstr))
        self.kill_btn.clicked.connect(self.robot.kill)
        self.get_mag_btn.clicked.connect(self.robot.get_mag)
        self.get_mag_cal_btn.clicked.connect(self.robot.get_mag_cal)
        self.send_btn.clicked.connect(
            lambda: self.send_command(self.command_input.text())
        )
        self.scan_btn.clicked.connect(self.scan)
        self.forward_step_btn.clicked.connect(
            lambda: self.robot.move(self.drive_step_slider.value())
        )
        self.backward_step_btn.clicked.connect(
            lambda: self.robot.move(-self.drive_step_slider.value())
        )
        self.turn_left_step_btn.clicked.connect(
            lambda: self.robot.rotate(-self.rotate_step_slider.value())
        )
        self.turn_right_step_btn.clicked.connect(
            lambda: self.robot.rotate(self.rotate_step_slider.value())
        )
        self.rotate_to_btn.clicked.connect(
            lambda: self.robot.rotate_to(self.rotate_to_slider.value())
        )
        self.clear_buffer_btn.clicked.connect(self.flush_serial_buffer)
        self.measure_btn.clicked.connect(self.calibration_timer.start)
        self.clear_mag_cal_data_btn.clicked.connect(self.clear_mag_cal_data)
        self.autocalibration_btn.clicked.connect(self.autocal_mag_cal)
        self.clear_pole_data_btn.clicked.connect(self.clear_pole_data)
        self.calibration_timer.tick_signal.connect(self.measure_mag_cal)
        self.calibration_timer.tick_signal.connect(
            lambda val: self.calibration_progbar.setValue(val)
        )

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
                timeout=5,
                parity=serial.PARITY_NONE,
                rtscts=0,
            )

            self.connection_status_label.setText("✔")
        except serial.SerialException as e:
            self.connection_status_label.setText("❌")
            self.make_message("Error", "Serial connection failed.", str(e))

        x = self.ser.read_until().decode()
        toprint = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] RECEIVED> {x}'
        self.command_output_long.setText(
            self.command_output_long.toPlainText() + toprint
        )
        self.command_output_short.setText(
            self.command_output_short.toPlainText() + toprint
        )

    def flush_serial_buffer(self):
        self.command_output_long.setText("")
        self.command_output_short.setText("")
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def scroll_outputs(self):
        sb = self.command_output_long.verticalScrollBar()
        sb.setValue(sb.maximum())
        sb = self.command_output_short.verticalScrollBar()
        sb.setValue(sb.maximum())

    def read_until(self, terminator="\n"):
        data = ""
        while True:
            char = self.ser.recv(1).decode("ASCII")
            data += char
            if char == terminator:
                break
        return data

    def send(self, data):
        self.ser.write(data)

    def send_command(self, command):
        self.send(command.encode("ASCII"))
        toprint = (
            f'[{datetime.datetime.now().strftime("%H:%M:%S")}] SENT> {command}\r\n'
        )
        self.command_output_long.setText(
            self.command_output_long.toPlainText() + toprint
        )
        self.command_output_short.setText(
            self.command_output_short.toPlainText() + toprint
        )
        self.scroll_outputs()

        x = self.ser.read_until().decode()
        toprint = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] RECEIVED> {x}'
        self.command_output_long.setText(
            self.command_output_long.toPlainText() + toprint
        )
        self.command_output_short.setText(
            self.command_output_short.toPlainText() + toprint
        )

        self.scroll_outputs()
        x = x.replace("\r\n", "")
        return x

    def reset_pole_plot(self):
        self.pole_plot.clear()
        self.pole_plot.setXRange(-300, 300)
        self.pole_plot.setYRange(-300, 300)
        self.pole_plot.showGrid(x=True, y=True, alpha=0.3)

    def reset_mag_cal_plot(self):
        self.mag_cal_plot.clear()
        self.mag_cal_plot.setXRange(-3000, 3000)
        self.mag_cal_plot.setYRange(-3000, 3000)
        self.mag_cal_plot.showGrid(x=True, y=True, alpha=0.3)

    def place_point_pole_plot(self, distance, _angle, color="#FF00FF"):
        angle = _angle + self.robot.azimuth
        angle = radians(angle)
        x = distance * cos(angle) + self.robot.position[0]
        y = distance * sin(angle) + self.robot.position[1]
        self.pole_plot_points = np.append(
            self.pole_plot_points, np.array([[x, y]]), axis=0
        )
        self.pole_plot_points_colors.append(color)

    def place_point_mag_cal_plot(self, x, y, color="#FF00FF"):
        self.mag_cal_points = np.append(self.mag_cal_points, np.array([[x, y]]), axis=0)
        self.mag_cal_points_colors.append(color)

    def redraw_pole_plot(self):
        self.pole_plot.clear()
        poles = pg.ScatterPlotItem(
            self.pole_plot_points[:, 0],
            self.pole_plot_points[:, 1],
            pen="#FF00FF",
            symbol="+",
        )  # TODO handle separate colors for each point
        self.pole_plot.addItem(poles)

        # TODO rethink this
        linedata = np.zeros((0, 2))
        for pole in self.pole_plot_points:
            linedata = np.append(linedata, np.array([[0, 0]]), axis=0)
            linedata = np.append(linedata, np.array([[pole[0], pole[1]]]), axis=0)
            # self.plot_2d.plot((0,pole[0]), (0, pole[1]), symbol=None)
        lines = pg.PlotCurveItem(x=linedata[:, 0], y=linedata[:, 1], connect="pairs")
        self.pole_plot.addItem(lines)
        ###

    def redraw_mag_cal_plot(self):
        self.reset_mag_cal_plot()
        points = pg.ScatterPlotItem(
            self.mag_cal_points[:, 0],
            self.mag_cal_points[:, 1],
            pen="#FF00FF",
            symbol="o",
        )  # TODO handle separate colors for each point
        self.mag_cal_plot.addItem(points)

    def clear_pole_data(self):
        self.pole_plot_points_colors = []
        self.pole_plot_points = np.zeros((0, 2))
        self.reset_pole_plot()

    def clear_mag_cal_data(self):
        self.min_x_label.setText("-")
        self.max_x_label.setText("-")
        self.offset_x_label.setText("-")
        self.min_y_label.setText("-")
        self.max_y_label.setText("-")
        self.offset_y_label.setText("-")
        self.rmin_label.setText("-")
        self.rmax_label.setText("-")
        self.theta_label.setText("-")
        self.costheta_label.setText("-")
        self.costheta2_label.setText("-")
        self.sintheta_label.setText("-")
        self.minussintheta_label.setText("-")
        self.sigma_label.setText("-")
        self.mag_cal_points_colors = []
        self.mag_cal_points = np.zeros((0, 2))
        self.reset_mag_cal_plot()

    def scan(self):
        data = self.robot.scan()
        for idx, dist in enumerate(data):
            if dist >= 15:
                ang = 1 * idx
                self.place_point_pole_plot(dist, ang)
        self.redraw_pole_plot()

    def measure_mag_cal(self):
        # Get rid of spikes
        x, y = [], []
        for i in range(3):
            _x, _y = self.robot.get_mag()
            x.append(_x)
            y.append(_y)
        x = median(x)
        y = median(y)

        # Update min/max values
        r = sqrt(x ** 2 + y ** 2)
        if self.rmin_label.text() == "-" or r < float(self.rmin_label.text()):
            self.rmin_label.setText(str(round(r, 1)))
        if self.rmax_label.text() == "-" or r > float(self.rmax_label.text()):
            self.rmax_label.setText(str(round(r, 1)))
            # Calculate and update Soft Iron R matrix
            theta = asin(y / r)
            costheta = cos(theta)
            sintheta = sin(theta)
            self.theta_label.setText(str(round(theta, 3)))
            self.costheta_label.setText(str(round(costheta, 3)))
            self.costheta2_label.setText(str(round(costheta, 3)))
            self.sintheta_label.setText(str(round(sintheta, 3)))
            self.minussintheta_label.setText(str(round(-sintheta, 3)))

        rmin = float(self.rmin_label.text())
        rmax = float(self.rmax_label.text())
        if rmax != 0:
            self.sigma_label.setText(str(round((rmin / rmax), 3)))

        if self.min_x_label.text() == "-" or x < int(self.min_x_label.text()):
            self.min_x_label.setText(str(x))

        if self.max_x_label.text() == "-" or x > int(self.max_x_label.text()):
            self.max_x_label.setText(str(x))

        if self.min_y_label.text() == "-" or y < int(self.min_y_label.text()):
            self.min_y_label.setText(str(y))

        if self.max_y_label.text() == "-" or y > int(self.max_y_label.text()):
            self.max_y_label.setText(str(y))

        # Calculate and update Hard Iron offsets
        if self.min_x_label.text() != "-" and self.max_x_label.text() != "-":
            self.offset_x_label.setText(
                str(
                    int(
                        (int(self.min_x_label.text()) + int(self.max_x_label.text()))
                        / 2
                    )
                )
            )

        if self.min_y_label.text() != "-" and self.max_y_label.text() != "-":
            self.offset_y_label.setText(
                str(
                    int(
                        (int(self.min_y_label.text()) + int(self.max_y_label.text()))
                        / 2
                    )
                )
            )

        self.place_point_mag_cal_plot(x, y)
        self.redraw_mag_cal_plot()

    def autocal_mag_cal(self):
        steps = 20
        self.measure_mag_cal()
        for i in range(steps):
            self.robot.drive(90, -90)
            sleep(0.25)
            self.robot.stop()
            sleep(0.1)
            self.measure_mag_cal()
            self.calibration_progbar.setValue(i * 100 / (steps - 1))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    MainWindow = MainWindow()
    MainWindow.show()
    sys.exit(app.exec_())
