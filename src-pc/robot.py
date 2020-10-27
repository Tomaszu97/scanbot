import datetime
from math import radians, degrees, sin, cos


class Robot:
    def __init__(self, send_recv_function, main_window):
        self.send = send_recv_function
        self.position = [0, 0]
        self.azimuth = 0
        self.main_window = main_window

    def rotate_tower(self, angle):
        return self.send(f"ROTATE_TOWER:{angle}#")

    def drive(self, left_speed, right_speed):
        return self.send(f"DRIVE:{left_speed},{right_speed}#")

    def stop(self):
        return self.drive(0, 0)

    def reset_position(self):
        self.position = [0, 0]
        self.main_window.robot_pos_x_label.setText("0")
        self.main_window.robot_pos_y_label.setText("0")

    def rotate(self, angle):
        self.azimuth = int(self.send(f"ROTATE:{angle}#"))
        return self.azimuth

    def rotate_to(self, azimuth):
        self.azimuth = int(self.send(f"ROTATE_TO:{azimuth}#"))
        return self.azimuth

    def move(self, distance):
        self.get_azimuth()
        self.position[0] += cos(radians(self.azimuth)) * distance
        self.position[1] += sin(radians(self.azimuth)) * distance
        self.main_window.robot_pos_x_label.setText(
            str(round(self.position[0], 2)))
        self.main_window.robot_pos_y_label.setText(
            str(round(self.position[1], 2)))
        return self.send(f"MOVE:{distance}#")

    def get_distance(self):
        return int(self.send("GET_DISTANCE#"))

    def get_azimuth(self):
        self.azimuth = int(self.send("GET_AZIMUTH#"))
        return self.azimuth

    def beep(self, time_ms, repeat_count=1):
        return self.send(f"BEEP:{time_ms},{repeat_count}#")

    def print(self, text):
        return self.send(f"PRINT:{text}#")

    def kill(self):
        return self.send("KILL#")

    def get_mag(self):
        data = self.send("GET_MAG#")
        data = data.strip()
        x, y = data.split(",")
        x = int(x)
        y = int(y)
        return (x, y)

    def get_mag_cal(self):
        data = self.send("GET_MAG_CAL#")
        data = data.strip()
        x, y, theta, sigma = data.split(",")
        x = int(x)
        y = int(y)
        theta = float(theta)
        sigma = float(sigma)
        return (x, y, theta, sigma)

    def set_mag_cal(self, x=0, y=0, theta=0, sigma=1):
        return self.send(f"SET_MAG_CAL:{x},{y},{theta},{sigma}#")

    def scan(self):
        data = self.send("SCAN#").split(",")[:-1]
        for i in range(len(data)):
            data[i] = int(data[i])
        return data
