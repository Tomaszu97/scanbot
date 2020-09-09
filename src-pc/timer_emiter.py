from PySide2 import QtCore
from time import sleep
import numpy as np


class TimerEmiter(QtCore.QThread):

    tick_signal = QtCore.Signal(int)
    
    def __init__(self, interval=1, repeat_count=1):
        QtCore.QThread.__init__(self)
        self.interval = interval
        self.repeat_count = repeat_count

    def run(self):
        for i in range(self.repeat_count):
            self.tick_signal.emit(int(100*i/self.repeat_count))
            sleep(self.interval)