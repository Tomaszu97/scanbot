import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from math import radians, degrees, sin, cos, sqrt, atan2, pi
import PySide2.QtGui as QtGui


class PolePlots():
    def __init__(self, main_window):
        self.main_window = main_window
        self.points = np.zeros((0, 2))
        self.colors = []

        self.pole_plot = pg.PlotWidget()
        self.main_window.pole_plot_layout.addWidget(self.pole_plot)

        self.reset_all()

    def reset_data(self):
        self.points = np.zeros((0, 2))
        self.colors = []

    def reset_plot(self):
        self.pole_plot.clear()
        self.pole_plot.showGrid(x=True, y=True, alpha=0.3)

    def reset_camera(self):
        self.pole_plot.setXRange(-400, 400)
        self.pole_plot.setYRange(-400, 400)

    def reset_all(self):
        self.reset_data()
        self.reset_plot()
        self.reset_camera()

    def place_pole(self, x=None, y=None, angle=None, distance=None, color="#FF00FF"):
        if angle is not None and distance is not None:
            x, y = self.calc_robot_angle_distance_to_xy(
                angle, distance)

        self.points = np.append(
            self.points, np.array([[x, y]]), axis=0
        )
        self.colors.append(color)

    def redraw(self):
        self.reset_plot()

        # draw lines
        # linedata = np.zeros((0, 2))
        # for pole in self.points:
        #     linedata = np.append(linedata, np.array(
        #         [[0, 0]]), axis=0)
        #     linedata = np.append(linedata, np.array(
        #         [[pole[0], pole[1]]]), axis=0)
        # lines = pg.PlotCurveItem(
        #     x=linedata[:, 0], y=linedata[:, 1], connect="pairs", pen=pg.mkPen("#555555"))
        # self.pole_plot.addItem(lines)

        pens = []
        for color in self.colors:
            pens.append(pg.mkPen(color))

        poles = pg.ScatterPlotItem(
            self.points[:, 0],
            self.points[:, 1],
            pen=pens,
            symbol="+",
        )
        self.pole_plot.addItem(poles)

    def get_poles(self):
        pass

    def place_robot(self, color="#777700"):
        pass

    def calc_robot_angle_distance_to_xy(self, _angle, distance):
        angle = _angle + self.main_window.robot.azimuth - 90
        angle = radians(angle)
        x = distance * cos(angle) + self.main_window.robot.position[0]
        y = distance * sin(angle) + self.main_window.robot.position[1]
        return x, y