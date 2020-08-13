import sys
from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox
from layout.layout import Ui_MainWindow
from pyside_material import apply_stylesheet
import serial.tools.list_ports
import serial


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        # apply_stylesheet(app, theme="dark_blue.xml")
        self.setupUi(self)
        self.connect_signals()
        # self.setWindowTitle("ScanBot Control Application")
        # self.setFixedWidth(1500)
        # self.setFixedHeight(800)
        self.setFixedSize(self.size())

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
        self.forward_btn.clicked.connect(self.go_forward)

    def serial_refresh(self):
        self.serial_combo.clear()
        for comport in serial.tools.list_ports.comports():
            device = comport.device
            self.serial_combo.addItem(str(device))

    def serial_connect(self):
        if self.connection_status_label.text == "✔":
            self.ser.close()

        print(self.serial_combo.currentText())

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

    def go_forward(self):
        self.ser.write(b"GET_DISTANCE#")
        x = self.ser.read_until().decode()
        x = x.replace("\r\n", "")
        print(x)


if __name__ == "__main__":

    app = QApplication(sys.argv)
    MainWindow = MainWindow()
    MainWindow.show()
    sys.exit(app.exec_())
