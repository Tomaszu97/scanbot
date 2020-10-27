import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from math import radians, degrees, sin, cos, sqrt, atan2, pi
import PySide2.QtGui as QtGui


class PolePlots():
    def __init__(self, main_window):
        self.main_window = main_window

        # Data
        self.points = np.zeros((0, 2))
        self.colors = []

        # 2D graph
        self.pole_plot = pg.PlotWidget()
        self.main_window.pole_plot_layout.addWidget(self.pole_plot)

        # 3D graph
        self.pole_plot_3d = gl.GLViewWidget()
        self.pole_plot_3d.setCameraPosition(distance=400)
        gr = gl.GLGridItem(size=QtGui.QVector3D(500, 500, 1))
        gr.setSpacing(x=10, y=10, z=10)
        self.pole_plot_3d.addItem(gr)
        self.main_window.pole_plot_3d_layout.addWidget(self.pole_plot_3d)

        self.reset_all()

    def reset_data(self):
        self.points = np.zeros((0, 2))
        self.colors = []

    def reset_plots(self):
        # TODO reset both 2d and 3d pole plots
        self.pole_plot.clear()
        self.pole_plot.showGrid(x=True, y=True, alpha=0.3)

    def reset_camera(self):
        # TODO reset both 2d and 3d pole plots
        self.pole_plot.setXRange(-400, 400)
        self.pole_plot.setYRange(-400, 400)

    def reset_all(self):
        self.reset_data()
        self.reset_plots()
        self.reset_camera()

    def place_pole(self, x=None, y=None, angle=None, distance=None, color="#FF00FF"):
        if angle is not None and distance is not None:
            x, y = self.main_window.calc_robot_angle_distance_to_xy(
                angle, distance)

        # 2D plot
        self.points = np.append(
            self.points, np.array([[x, y]]), axis=0
        )
        self.colors.append(color)

        # 3D plot
        md = gl.MeshData.cylinder(1, 5, radius=[1, 1], length=25)
        m = gl.GLMeshItem(
            meshdata=md, smooth=False, drawFaces=False, drawEdges=True, edgeColor=(1, 0, 1, 1)
        )
        m.translate(x, y, 0)
        # TODO: redraw in self.redraw somehow
        self.pole_plot_3d.addItem(m)

    def redraw(self):
        self.reset_plots()

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
