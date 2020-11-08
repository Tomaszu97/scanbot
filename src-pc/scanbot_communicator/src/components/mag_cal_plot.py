import pyqtgraph as pg
import numpy as np
from time import sleep
from statistics import median
from math import radians, degrees, sin, cos, sqrt, atan2, pi

class MagCalPlot:
    def __init__(self, send_recv_function, main_window):
        self.main_window = main_window
        self.send = send_recv_function
        
        self.points = np.zeros((0, 2))
        self.points_colors = []
        self.plot = pg.PlotWidget()
        
        self.plot.setAspectLocked()
        self.main_window.mag_cal_plot_layout.addWidget(self.plot)
        self.reset_all()

    def reset_data(self):
        self.main_window.min_x_label.setText("-")
        self.main_window.max_x_label.setText("-")
        self.main_window.offset_x_label.setText("-")
        self.main_window.min_y_label.setText("-")
        self.main_window.max_y_label.setText("-")
        self.main_window.offset_y_label.setText("-")
        self.main_window.rmin_label.setText("-")
        self.main_window.rmax_label.setText("-")
        self.main_window.theta_label.setText("-")
        self.main_window.costheta_label.setText("-")
        self.main_window.costheta2_label.setText("-")
        self.main_window.sintheta_label.setText("-")
        self.main_window.minussintheta_label.setText("-")
        self.main_window.sigma_label.setText("-")
        self.points_colors = []
        self.points = np.zeros((0, 2))

    def reset_plot(self):
        self.plot.clear()
        self.plot.showGrid(x=True, y=True, alpha=0.3)

    def reset_camera(self):
        self.plot.setXRange(-3000, 3000)
        self.plot.setYRange(-3000, 3000)
    
    def reset_all(self):
        self.reset_data()
        self.reset_plot()
        self.reset_camera()

    def place_point(self, x, y, color="#FF00FF"):
        self.points = np.append(self.points, np.array([[x, y]]), axis=0)
        self.points_colors.append(color)

    def redraw(self):
        self.reset_plot()
        points = pg.ScatterPlotItem(
            self.points[:, 0],
            self.points[:, 1],
            pen="#FF00FF",
            symbol="o",
        )  # TODO handle separate colors for each point
        self.plot.addItem(points)

    def measure(self):
        # Get rid of spikes
        x, y = [], []
        for i in range(3):
            _x, _y = self.main_window.robot.get_mag()
            x.append(_x)
            y.append(_y)
        x = median(x)
        y = median(y)

        if self.main_window.min_x_label.text() == "-" or x < int(self.main_window.min_x_label.text()):
            self.main_window.min_x_label.setText(str(x))

        if self.main_window.max_x_label.text() == "-" or x > int(self.main_window.max_x_label.text()):
            self.main_window.max_x_label.setText(str(x))

        if self.main_window.min_y_label.text() == "-" or y < int(self.main_window.min_y_label.text()):
            self.main_window.min_y_label.setText(str(y))

        if self.main_window.max_y_label.text() == "-" or y > int(self.main_window.max_y_label.text()):
            self.main_window.max_y_label.setText(str(y))

        self.place_point(x, y)
        self.redraw()

    def autocal(self):
        self.reset_all()
        steps = 20
        self.measure()
        for i in range(steps):
            self.main_window.robot.drive(90, -90)
            sleep(0.25)
            self.main_window.robot.stop()
            sleep(0.1)
            self.measure()
            self.main_window.calibration_progbar.setValue(i * 100 / (steps - 1))

    def correct_hard(self):
        # Calculate Hard Iron error
        for i in range(len(self.points)):
            x = self.points[i, 0]
            y = self.points[i, 1]
            if self.main_window.min_x_label.text() != "-" and self.main_window.max_x_label.text() != "-":
                self.main_window.offset_x_label.setText(
                    str(int((int(self.main_window.min_x_label.text()) + int(self.main_window.max_x_label.text())) / 2)))

            if self.main_window.min_y_label.text() != "-" and self.main_window.max_y_label.text() != "-":
                self.main_window.offset_y_label.setText(
                    str(int((int(self.main_window.min_y_label.text()) + int(self.main_window.max_y_label.text())) / 2)))

        # Apply Hard Iron correction
        for i in range(len(self.points)):
            self.points[i, 0] = self.points[i, 0] - int(self.main_window.offset_x_label.text())
            self.points[i, 1] = self.points[i, 1] - int(self.main_window.offset_y_label.text())

        # Redraw plot
        self.redraw()

    def correct_soft(self):
        # Calculate Soft Iron error
        for i in range(len(self.points)):
            x = self.points[i, 0]
            y = self.points[i, 1]
            r = sqrt(x ** 2 + y ** 2)
            if self.main_window.rmin_label.text() == "-" or r < float(self.main_window.rmin_label.text()):
                self.main_window.rmin_label.setText(str(round(r, 1)))
            if self.main_window.rmax_label.text() == "-" or r > float(self.main_window.rmax_label.text()):
                self.main_window.rmax_label.setText(str(round(r, 1)))
                # Calculate and update Soft Iron R matrix
                if y >= 0:
                    theta = atan2(y, x)
                else:
                    theta = 2*pi+atan2(y, x)
                costheta = cos(theta)
                sintheta = sin(theta)
                self.main_window.theta_label.setText(str(round(theta, 3)))
                self.main_window.costheta_label.setText(str(round(costheta, 3)))
                self.main_window.costheta2_label.setText(str(round(costheta, 3)))
                self.main_window.sintheta_label.setText(str(round(sintheta, 3)))
                self.main_window.minussintheta_label.setText(str(round(-sintheta, 3)))
            rmin = float(self.main_window.rmin_label.text())
            rmax = float(self.main_window.rmax_label.text())
            if rmax != 0:
                self.main_window.sigma_label.setText(str(round((rmin / rmax), 3)))

        # Apply Soft Iron correction
        R = np.matrix(
            [[float(self.main_window.costheta_label.text()), float(self.main_window.sintheta_label.text())],
             [float(self.main_window.minussintheta_label.text()), float(self.main_window.costheta2_label.text())]])
        for i in range(len(self.points)):
            # Rotate and scale
            v = np.matrix([[self.points[i, 0]],
                           [self.points[i, 1]]])
            v2 = R*v
            v2[0] = v2[0]*float(self.main_window.sigma_label.text())
            self.points[i, 0] = int(v2[0])
            self.points[i, 1] = int(v2[1])

        # Redraw plot
        self.redraw()

    def sendcal(self):
        self.main_window.robot.set_mag_cal(x=-int(self.main_window.offset_x_label.text()), y=-int(self.main_window.offset_y_label.text(
        )), theta=float(self.main_window.theta_label.text()), sigma=float(self.main_window.sigma_label.text()))
