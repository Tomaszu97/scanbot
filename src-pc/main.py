from src.robot import Robot
from src.timer_emiter import TimerEmiter
from src.pole_plots import PolePlots
from src.serial_conn import SerialConnection
from src.mag_cal_plot import MagCalPlot
from src.strings import *
from layout.layout import Ui_MainWindow
import sys
from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox
import PySide2.QtCore as QtCore
from pyside_material import apply_stylesheet
import randomcolor


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        # Window initialization
        QMainWindow.__init__(self)
        #apply_stylesheet(app, theme="dark_blue.xml")
        self.setFixedWidth(1150)
        self.setFixedHeight(650)
        self.setupUi(self)

        # Data / objects
        self.serial = SerialConnection(self)
        self.robot = Robot(self.serial.send_command, self)
        self.pole_plots = PolePlots(self)
        self.mag_cal_plot = MagCalPlot(self.serial.send_command,self)
        self.calibration_timer = TimerEmiter(interval=0.2, repeat_count=50)       
        self.randcol = lambda: randomcolor.RandomColor().generate(
            luminosity="bright", count=1)[0]

        self.connect_signals()
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
        
        self.get_distance_btn.clicked.connect(self.robot.get_distance)
        self.get_azimuth_btn.clicked.connect(self.robot.get_azimuth)
        self.beep_btn.clicked.connect(lambda: self.robot.beep(30, 3))
        self.print_duck_btn.clicked.connect(lambda: self.robot.print(duckstr))
        self.kill_btn.clicked.connect(self.robot.kill)
        self.get_mag_btn.clicked.connect(self.robot.get_mag)
        self.get_mag_cal_btn.clicked.connect(self.robot.get_mag_cal)
        self.scan_btn.clicked.connect(self.robot.scan)

        self.clear_buffer_btn.clicked.connect(self.serial.flush_buffer)
        self.send_btn.clicked.connect(
            lambda: self.serial.send_command(self.command_input.text())
        )
        
        self.clear_pole_data_btn.clicked.connect(self.pole_plots.reset_all)

        self.clear_mag_cal_data_btn.clicked.connect(self.mag_cal_plot.reset_all)
        self.send_clear_cal_data_btn.clicked.connect(
            lambda: self.robot.set_mag_cal(0, 0, 0, 1))
        self.measure_btn.clicked.connect(self.calibration_timer.start)
        self.calibration_timer.tick_signal.connect(self.mag_cal_plot.measure)
        self.calibration_timer.tick_signal.connect(
            lambda val: self.calibration_progbar.setValue(val)
        )
        self.auto_measure_btn.clicked.connect(self.mag_cal_plot.autocal)
        self.correct_hard_btn.clicked.connect(self.mag_cal_plot.correct_hard)
        self.correct_soft_btn.clicked.connect(self.mag_cal_plot.correct_soft)
        self.send_cal_data_btn.clicked.connect(self.mag_cal_plot.sendcal)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    MainWindow = MainWindow()
    MainWindow.show()
    sys.exit(app.exec_())
