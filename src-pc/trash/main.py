from pyqtgraph.Qt import QtCore, QtGui  # PySide2
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np

app = QtGui.QApplication([])
w = QtGui.QWidget()

g = gl.GLViewWidget()
g.resize(1280, 720)
g.setCameraPosition(distance=10)
gr = gl.GLGridItem()
# gr.scale(2, 2, 1)
g.addItem(gr)
md = gl.MeshData.sphere(rows=4, cols=9)
m = gl.GLMeshItem(
    meshdata=md, smooth=False, drawFaces=False, drawEdges=True, edgeColor=(1, 0, 1, 1)
)


def handler():
    m.translate(0, 0.1, 0)
    g.addItem(m)
    print("witam")


refresh_serial_btn = QtGui.QPushButton("refresh\nserial\nports")
serial_combo = QtGui.QComboBox()
connect_serial_btn = QtGui.QPushButton("connect")
sample_label = QtGui.QLabel("sample text")
plot = pg.PlotWidget()

refresh_serial_btn.clicked.connect(handler)

hboxlayout = QtGui.QHBoxLayout()
vboxlayout1 = QtGui.QVBoxLayout()
vboxlayout2 = QtGui.QVBoxLayout()
vboxlayout1.addWidget(refresh_serial_btn)
vboxlayout1.addWidget(serial_combo)
vboxlayout1.addWidget(connect_serial_btn)
vboxlayout1.addWidget(g)
vboxlayout2.addWidget(sample_label)
vboxlayout2.addWidget(plot)
hboxlayout.addLayout(vboxlayout1, 1)
hboxlayout.addLayout(vboxlayout2, 1)
w.setLayout(hboxlayout)

plot.plot(y=[1, 3, 4, 2, 1, 3, 4, 2])
serial_combo.addItem("COM X")
serial_combo.addItem("COM Y")
serial_combo.addItem("COM Z")


w.resize(1280, 720)
w.show()
app.exec_()


# md = gl.MeshData.sphere(rows=4, cols=9)
# m = gl.GLMeshItem(
#     meshdata=md, smooth=False, drawFaces=False, drawEdges=True, edgeColor=(1, 0, 1, 1)
# )
# self.plot_3d.addItem(m)

