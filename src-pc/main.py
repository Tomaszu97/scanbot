import sys
from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox
from layout.layout import Ui_MainWindow
from pyside_material import apply_stylesheet
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
from time import sleep
from robot import Robot
from timer_emiter import TimerEmiter
from pole_plots import PolePlots
from serial_conn import SerialConnection
from statistics import median
from strings import *
import randomcolor
import PySide2.QtCore as QtCore
from math import radians, degrees, sin, cos, sqrt, atan2, pi
from scipy.spatial import distance


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        # apply_stylesheet(app, theme="dark_blue.xml")
        self.setFixedWidth(1150)
        self.setFixedHeight(650)
        self.setupUi(self)

        # Data / objects
        self.serial = SerialConnection(self)
        self.robot = Robot(self.serial.send_command)
        self.randcol = lambda: randomcolor.RandomColor().generate(
            luminosity="bright", count=1)[0]

        # Magnetometer calibration plot
        self.mag_cal_points = np.zeros((0, 2))
        self.mag_cal_points_colors = []
        self.mag_cal_plot = pg.PlotWidget()
        self.mag_cal_plot.setAspectLocked()
        self.mag_cal_plot_layout.addWidget(self.mag_cal_plot)
        self.reset_mag_cal_plot()

        # Pole plots
        self.pole_plots = PolePlots(self)

        # Workers
        self.calibration_timer = TimerEmiter(interval=0.2, repeat_count=50)

        # Controls' handlers
        self.connect_signals()

        # Search for serial ports
        self.serial.refresh_list()

    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            key = event.key()
            if key == QtCore.Qt.Key_Up:
                self.robot.drive(90, 90)
            elif key == QtCore.Qt.Key_Down:
                self.robot.drive(-90, -90)
            elif key == QtCore.Qt.Key_Left:
                self.robot.drive(-90, 90)
            elif key == QtCore.Qt.Key_Right:
                self.robot.drive(90, -90)
            elif key == QtCore.Qt.Key_S:
                self.scan()
        return super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            key = event.key()
            if key == QtCore.Qt.Key_Up:
                self.robot.stop()
            elif key == QtCore.Qt.Key_Down:
                self.robot.stop()
            elif key == QtCore.Qt.Key_Left:
                self.robot.stop()
            elif key == QtCore.Qt.Key_Right:
                self.robot.stop()
        return super().keyReleaseEvent(event)

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
        self.serial_refresh_btn.clicked.connect(self.serial.refresh_list)
        self.serial_connect_btn.clicked.connect(self.serial.connect)
        self.forward_btn.pressed.connect(lambda: self.robot.drive(90, 90))
        self.forward_btn.released.connect(lambda: self.robot.drive(0, 0))
        self.backward_btn.pressed.connect(lambda: self.robot.drive(-90, -90))
        self.backward_btn.released.connect(lambda: self.robot.drive(0, 0))
        self.turn_left_btn.pressed.connect(lambda: self.robot.drive(-90, 90))
        self.turn_left_btn.released.connect(lambda: self.robot.drive(0, 0))
        self.turn_right_btn.pressed.connect(lambda: self.robot.drive(90, -90))
        self.turn_right_btn.released.connect(lambda: self.robot.drive(0, 0))
        self.tower_slider.valueChanged.connect(
            lambda val: self.robot.rotate_tower(val))
        self.get_distance_btn.clicked.connect(
            lambda: self.robot.get_distance())
        self.get_azimuth_btn.clicked.connect(lambda: self.robot.get_azimuth())
        self.beep_btn.clicked.connect(lambda: self.robot.beep(30, 3))
        self.print_duck_btn.clicked.connect(lambda: self.robot.print(duckstr))
        self.kill_btn.clicked.connect(self.robot.kill)
        self.get_mag_btn.clicked.connect(self.robot.get_mag)
        self.get_mag_cal_btn.clicked.connect(self.robot.get_mag_cal)
        self.send_btn.clicked.connect(
            lambda: self.serial.send_command(self.command_input.text())
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
        self.clear_buffer_btn.clicked.connect(self.serial.flush_buffer)
        self.clear_pole_data_btn.clicked.connect(self.pole_plots.reset_all)

        self.clear_mag_cal_data_btn.clicked.connect(self.clear_mag_cal_data)
        self.send_clear_cal_data_btn.clicked.connect(
            lambda: self.robot.set_mag_cal(0, 0, 0, 1))
        self.measure_btn.clicked.connect(self.calibration_timer.start)
        self.calibration_timer.tick_signal.connect(self.measure_mag_cal)
        self.calibration_timer.tick_signal.connect(
            lambda val: self.calibration_progbar.setValue(val)
        )
        self.auto_measure_btn.clicked.connect(self.autocal_mag_cal)
        self.correct_hard_btn.clicked.connect(self.correct_hard_mag_cal)
        self.correct_soft_btn.clicked.connect(self.correct_soft_mag_cal)
        self.send_cal_data_btn.clicked.connect(self.sendcal_mag_cal)

    def place_point_mag_cal_plot(self, x, y, color="#FF00FF"):
        self.mag_cal_points = np.append(
            self.mag_cal_points, np.array([[x, y]]), axis=0)
        self.mag_cal_points_colors.append(color)

    def redraw_mag_cal_plot(self):
        self.reset_mag_cal_plot()
        points = pg.ScatterPlotItem(
            self.mag_cal_points[:, 0],
            self.mag_cal_points[:, 1],
            pen="#FF00FF",
            symbol="o",
        )  # TODO handle separate colors for each point
        self.mag_cal_plot.addItem(points)

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

    def find_xy_closest_point_distance(self, this_point, other_points):
        min_distance = min(distance.cdist([this_point], other_points)[0])
        return min_distance
        # smallest_distance = None
        # for other_point in other_points:
        #     current_distance = sqrt(
        #         (other_point[0]-this_point[0])**2 + (other_point[1]-this_point[1])**2)
        #     if smallest_distance is None or current_distance < smallest_distance:
        #         smallest_distance = current_distance
        # return smallest_distance

    def rotate_points(self, points, angle, center_x=0, center_y=0):
        rotated_points = np.zeros((0, 2))
        for point in points:
            rotated_points = np.append(rotated_points, np.array(
                [[point[0]+angle, point[1]]]), axis=0)
        return rotated_points

    def calc_xy_overlap_score(self, points1, points2):
        score = 0
        for point in points1:
            distance = self.find_xy_closest_point_distance(
                [point[0], point[1]], points2)
            # important! this way we're avoiding very high 1/distance value
            distance = int(distance)
            if distance == 0:
                score += 100
            elif distance >= 100:
                pass
            else:
                score += int((1/distance)*100)
        return score

    def scan(self):
        self.robot.get_azimuth()
        data = self.robot.scan()
        self.robot.rotate_tower(90)

        # filter out too short measurements, convert idx to angle
        filtered_data = np.zeros((0, 2))
        for idx, dist in enumerate(data):
            if dist >= 15:
                ang = 1 * idx  # 1 degree step
                filtered_data = np.append(
                    filtered_data, np.array([[ang, dist]]), axis=0)

        # rotate to fit
        if self.pole_plots.points.size != 0:
            max_correction_angle = 15
            rotation_scores = {}
            for current_rotation in range(-max_correction_angle, max_correction_angle+1):
                # rotate
                rotated_data = self.rotate_points(
                    filtered_data, current_rotation)

                # calc xy positions
                rotated_xy_data = np.zeros((0, 2))
                for point in rotated_data:
                    x, y = self.calc_robot_angle_distance_to_xy(
                        point[0], point[1])
                    rotated_xy_data = np.append(
                        rotated_xy_data, np.array([[x, y]]), axis=0)

                # calculate score for this particular adjustment angle
                score = self.calc_xy_overlap_score(
                    rotated_xy_data, self.pole_plots.points)
                rotation_scores[current_rotation] = score

            max_score = rotation_scores[max(
                rotation_scores, key=rotation_scores.get)]
            best_rotation = max(rotation_scores, key=rotation_scores.get)
            print(
                f"max score {max_score} for adjustment rotation {best_rotation}. adjusted only if score is above threshold")

            if score > 2500:
                rotated_data = self.rotate_points(filtered_data, best_rotation)
            else:
                rotated_data = filtered_data

        else:
            rotated_data = filtered_data

        rcol = self.randcol()
        for ang, dist in rotated_data:
            self.pole_plots.place_pole(
                angle=ang, distance=dist, color=rcol)

        self.pole_plots.redraw()

    def reset_mag_cal_plot(self):
        self.mag_cal_plot.clear()
        self.mag_cal_plot.setXRange(-3000, 3000)
        self.mag_cal_plot.setYRange(-3000, 3000)
        self.mag_cal_plot.showGrid(x=True, y=True, alpha=0.3)

    def measure_mag_cal(self):
        # Get rid of spikes
        x, y = [], []
        for i in range(3):
            _x, _y = self.robot.get_mag()
            x.append(_x)
            y.append(_y)
        x = median(x)
        y = median(y)

        if self.min_x_label.text() == "-" or x < int(self.min_x_label.text()):
            self.min_x_label.setText(str(x))

        if self.max_x_label.text() == "-" or x > int(self.max_x_label.text()):
            self.max_x_label.setText(str(x))

        if self.min_y_label.text() == "-" or y < int(self.min_y_label.text()):
            self.min_y_label.setText(str(y))

        if self.max_y_label.text() == "-" or y > int(self.max_y_label.text()):
            self.max_y_label.setText(str(y))

        self.place_point_mag_cal_plot(x, y)
        self.redraw_mag_cal_plot()

    def autocal_mag_cal(self):
        self.clear_mag_cal_data()
        self.reset_mag_cal_plot()
        steps = 20
        self.measure_mag_cal()
        for i in range(steps):
            self.robot.drive(90, -90)
            sleep(0.25)
            self.robot.stop()
            sleep(0.1)
            self.measure_mag_cal()
            self.calibration_progbar.setValue(i * 100 / (steps - 1))

    def correct_hard_mag_cal(self):
        # Calculate Hard Iron error
        for i in range(len(self.mag_cal_points)):
            x = self.mag_cal_points[i, 0]
            y = self.mag_cal_points[i, 1]
            if self.min_x_label.text() != "-" and self.max_x_label.text() != "-":
                self.offset_x_label.setText(
                    str(int((int(self.min_x_label.text()) + int(self.max_x_label.text())) / 2)))

            if self.min_y_label.text() != "-" and self.max_y_label.text() != "-":
                self.offset_y_label.setText(
                    str(int((int(self.min_y_label.text()) + int(self.max_y_label.text())) / 2)))

        # Apply Hard Iron correction
        for i in range(len(self.mag_cal_points)):
            self.mag_cal_points[i, 0] = self.mag_cal_points[i,
                                                            0] - int(self.offset_x_label.text())
            self.mag_cal_points[i, 1] = self.mag_cal_points[i,
                                                            1] - int(self.offset_y_label.text())

        # Redraw plot
        self.redraw_mag_cal_plot()

    def correct_soft_mag_cal(self):
        # Calculate Soft Iron error
        for i in range(len(self.mag_cal_points)):
            x = self.mag_cal_points[i, 0]
            y = self.mag_cal_points[i, 1]
            r = sqrt(x ** 2 + y ** 2)
            if self.rmin_label.text() == "-" or r < float(self.rmin_label.text()):
                self.rmin_label.setText(str(round(r, 1)))
            if self.rmax_label.text() == "-" or r > float(self.rmax_label.text()):
                self.rmax_label.setText(str(round(r, 1)))
                # Calculate and update Soft Iron R matrix
                if y >= 0:
                    theta = atan2(y, x)
                else:
                    theta = 2*pi+atan2(y, x)
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

        # Apply Soft Iron correction
        R = np.matrix(
            [[float(self.costheta_label.text()), float(self.sintheta_label.text())],
             [float(self.minussintheta_label.text()), float(self.costheta2_label.text())]])
        for i in range(len(self.mag_cal_points)):
            # rotate
            v = np.matrix([[self.mag_cal_points[i, 0]],
                           [self.mag_cal_points[i, 1]]])
            v2 = R*v
            v2[0] = v2[0]*float(self.sigma_label.text())  # scale
            self.mag_cal_points[i, 0] = int(v2[0])
            self.mag_cal_points[i, 1] = int(v2[1])

        # Redraw plot
        self.redraw_mag_cal_plot()

    def sendcal_mag_cal(self):
        self.robot.set_mag_cal(x=-int(self.offset_x_label.text()), y=-int(self.offset_y_label.text(
        )), theta=float(self.theta_label.text()), sigma=float(self.sigma_label.text()))

    def calc_robot_angle_distance_to_xy(self, _angle, distance):
        # TODO: think about moving it back to main
        angle = _angle + self.robot.azimuth
        angle = radians(angle)
        x = distance * cos(angle) + self.robot.position[0]
        y = distance * sin(angle) + self.robot.position[1]
        return x, y


if __name__ == "__main__":
    app = QApplication(sys.argv)
    MainWindow = MainWindow()
    MainWindow.show()
    sys.exit(app.exec_())
