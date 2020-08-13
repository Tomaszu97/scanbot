# -*- coding: utf-8 -*-
"""
Simple examples demonstrating the use of GLMeshItem.

"""

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np

app = QtGui.QApplication([])
w = gl.GLViewWidget()
w.resize(1280, 720)
w.show()
w.setWindowTitle("🍆")
w.setCameraPosition(distance=10)

# g = gl.GLGridItem()
# g.scale(2, 2, 1)
# w.addItem(g)


md = gl.MeshData.sphere(rows=7, cols=7)
m4 = gl.GLMeshItem(
    meshdata=md, smooth=False, drawFaces=False, drawEdges=True, edgeColor=(1, 0, 1, 1)
)
# m4.translate(0, 10, 0)
w.addItem(m4)


## Start Qt event loop unless running in interactive mode.
if __name__ == "__main__":
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
        QtGui.QApplication.instance().exec_()

    print("witam")

