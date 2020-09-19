import datetime
from math import radians, degrees, sin, cos

class Robot():
    def __init__(self, send_recv_function):
        self.send = send_recv_function
        self.position = [0,0]
        self.azimuth = 0       


    def rotate_tower(self, angle):
        return self.send(f"ROTATE_TOWER:{angle}#")

    def drive(self, left_speed, right_speed):
        return self.send(f"DRIVE:{left_speed},{right_speed}#")

    def stop(self):
        return self.drive(0,0)

    def rotate(self, angle):
        self.azimuth = int(self.send(f"ROTATE:{angle}#"))
        return self.azimuth

    def rotate_to(self, azimuth):
        self.azimuth = int(self.send(f"ROTATE_TO:{azimuth}#"))
        return self.azimuth

    def move(self, distance):
        self.position[0] += cos(radians(self.azimuth)) * distance
        self.position[1] += sin(radians(self.azimuth)) * distance
        return self.send(f"MOVE:{distance}#")

    def get_distance(self):
        return int(self.send("GET_DISTANCE#"))
    
    def get_azimuth(self):
        self.azimuth = int(self.send("GET_AZIMUTH#"))
        return self.azimuth

    def beep(self, time_ms, repeat_count = 1):
        return self.send(f"BEEP:{time_ms},{repeat_count}#")

    def print(self, text):
        return self.send(f"PRINT:{text}#")

    def kill(self):
        return self.send("KILL#")

    def get_mag(self):
        data = self.send("GET_MAG#")
        data = data.strip()
        x,y,z = data.split(",")
        x = int(x)
        y = int(y)
        z = int(z)
        return (x, y, z)

    def get_mag_cal(self):
        data =  self.send("GET_MAG_CAL#")
        data = data.strip()
        x, y, z, a1, a2, a3, a4, mul = data.split(",")
        x = int(x)
        y = int(y)
        z = int(z)
        a1 = int(a1)
        a2 = int(a2)
        a3 = int(a3)
        a4 = int(a4)
        mul = int(mul)
        return (x, y, z, a1, a2, a3, a4, mul)

    def set_mag_cal(self, x=0, y=0, z=0, a1=0, a2=90, a3=180, a4=270, mul=-1):
        return self.send(f"SET_MAG_CAL:{x},{y},{z},{a1},{a2},{a3},{a4},{mul}#")
    