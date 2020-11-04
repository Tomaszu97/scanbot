import serial.tools.list_ports
import serial
import datetime


class SerialConnection():
    def __init__(self, main_window):
        self.main_window = main_window

    def refresh_list(self):
        self.main_window.serial_combo.clear()
        for comport in serial.tools.list_ports.comports():
            device = comport.device
            self.main_window.serial_combo.addItem(str(device))

    def connect(self):
        # TODO display error message if no serial exists
        if self.main_window.connection_status_label.text == "✔":
            self.ser.close()

        self.main_window.connection_status_label.setText("⌚")
        try:
            self.ser = serial.Serial(
                port=f"{self.main_window.serial_combo.currentText()}",
                baudrate=38400,
                timeout=5,
                parity=serial.PARITY_NONE,
                rtscts=0,
            )

            self.main_window.connection_status_label.setText("✔")
        except serial.SerialException as e:
            self.main_window.connection_status_label.setText("❌")
            self.main_window.make_message(
                "Error", "Serial connection failed.", str(e))

        x = self.ser.read_until().decode()
        toprint = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] RECEIVED> {x}\n'
        toprint += f'[{datetime.datetime.now().strftime("%H:%M:%S")}] BUFFER FLUSHED\n'
        self.main_window.command_output_long.setText(
            self.main_window.command_output_long.toPlainText() + toprint
        )
        self.main_window.command_output_short.setText(
            self.main_window.command_output_short.toPlainText() + toprint
        )

    def flush_buffer(self):
        self.main_window.command_output_long.setText("")
        self.main_window.command_output_short.setText("")
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

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
        self.main_window.command_output_long.setText(
            self.main_window.command_output_long.toPlainText() + toprint
        )
        self.main_window.command_output_short.setText(
            self.main_window.command_output_short.toPlainText() + toprint
        )
        self.scroll_outputs()

        x = self.ser.read_until().decode()
        toprint = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] RECEIVED> {x}'
        self.main_window.command_output_long.setText(
            self.main_window.command_output_long.toPlainText() + toprint
        )
        self.main_window.command_output_short.setText(
            self.main_window.command_output_short.toPlainText() + toprint
        )

        self.scroll_outputs()
        x = x.replace("\r\n", "")
        return x

    def scroll_outputs(self):
        sb = self.main_window.command_output_long.verticalScrollBar()
        sb.setValue(sb.maximum())
        sb = self.main_window.command_output_short.verticalScrollBar()
        sb.setValue(sb.maximum())
