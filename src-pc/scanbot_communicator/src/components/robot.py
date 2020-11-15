import datetime
from math import radians, degrees, sin, cos, pi
import rospy
import tf
from sensor_msgs.msg import LaserScan
import numpy as np

class Robot:
    def __init__(self, send_recv_function, main_window):
        self.main_window = main_window
        self.send = send_recv_function
        
        self.position = [0, 0]
        self.azimuth = 0
        
        self.pos_bcaster = tf.TransformBroadcaster()
        self.scan_publisher = rospy.Publisher('scan', LaserScan, queue_size=10)
        rospy.init_node('scanbot_communicator')
        self.odom_corrected_listener = tf.TransformListener()
        

    def rotate_tower(self, angle):
        return self.send(f"ROTATE_TOWER:{angle}#")

    def drive(self, left_speed, right_speed):
        return self.send(f"DRIVE:{left_speed},{right_speed}#")

    def stop(self):
        return self.drive(0, 0)

    def rotate(self, angle):
        self.send(f"ROTATE:{angle}#")
        self.azimuth += angle
        self.publish_ros_odometry()
        return self.azimuth

    def rotate_to(self, azimuth):
        self.azimuth = int(self.send(f"ROTATE_TO:{azimuth}#"))
        self.publish_ros_odometry()
        return self.azimuth

    def move(self, distance):
        self.position[0] += cos(radians(self.azimuth)) * distance
        self.position[1] += sin(radians(self.azimuth)) * distance
        self.publish_ros_odometry()
        return self.send(f"MOVE:{distance*1.125}#")#TODO

    def get_distance(self):
        return int(self.send("GET_DISTANCE#"))

    def get_azimuth(self):
        self.azimuth = int(self.send("GET_AZIMUTH#"))
        self.publish_ros_odometry()
        return self.azimuth

    def get_azimuth_kalman(self):
        q = 0.1
        estimate_err = 3
        measure_err = 3
        last_est = int(self.send("GET_AZIMUTH#"))

        for _ in range(20):
            measurement = int(self.send("GET_AZIMUTH#"))
            kalman_gain = estimate_err/(estimate_err + measure_err)
            current_est = last_est + kalman_gain*(measurement - last_est)
            estimate_err = (1 - kalman_gain)*estimate_err + abs(last_est-current_est)*q
            last_est = current_est

            # Print values in console output
            toprint = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] KALMAN OUTPUT>  RAW: {measurement} FILTERED: {round(current_est,2)}\n'
            self.main_window.command_output_long.setText(
                self.main_window.command_output_long.toPlainText() + toprint
            )
            self.main_window.command_output_short.setText(
                self.main_window.command_output_short.toPlainText() + toprint
            )
            sb = self.main_window.command_output_long.verticalScrollBar()
            sb.setValue(sb.maximum())
            sb = self.main_window.command_output_short.verticalScrollBar()
            sb.setValue(sb.maximum())

        current_est = int(current_est)
        self.azimuth = current_est
        self.publish_ros_odometry()
        return self.azimuth

    def beep(self, time_ms=100, repeat_count=1):
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
        
        self.publish_ros_odometry()
        self.publish_ros_scan(data)
        return data

    def publish_ros_odometry(self):
        self.pos_bcaster.sendTransform(
            translation=(self.position[0]/100, self.position[1]/100, 0),
            rotation=tf.transformations.quaternion_from_euler(0, 0, radians(self.azimuth)),
            time=rospy.Time.now(),
            child='base_link',
            parent='odom'
        )

    def publish_ros_scan(self, data):
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_laser'
        msg.angle_min = -(pi/2)
        msg.angle_max = pi/2
        msg.angle_increment = pi/180
        msg.range_min = 0.2
        msg.range_max = 8
        msg.ranges = [ distance/100 for distance in data ]
        self.scan_publisher.publish(msg)


    def _auto_drive(self):
        LR1 = 50
        LR2 = 70
        LR3 = 90
        FR =  30
        LARGE_TURN = 30
        SMALL_TURN = 10
        MOVE_STEP = 10

        while True:
            data = self.scan()
            F_ZONE = data[70:111]
            L_ZONE = data[140:]

            fscore = 0
            for dist in F_ZONE:
                if dist < FR:
                    fscore += 1

            l1score = 0
            for dist in L_ZONE:
                if dist < LR1:
                    l1score += 1

            l2score = 0
            for dist in L_ZONE:
                if dist > LR2 and dist < LR3:
                    l2score += 1

            if fscore > 2:
                self.beep(300,1)
                self.rotate(-LARGE_TURN)
            elif l1score > 2:
                self.beep(30,2)
                self.rotate(-SMALL_TURN)
                self.move(MOVE_STEP)
                self.rotate(SMALL_TURN)
            elif l2score > 2:
                self.beep(30,3)
                self.rotate(SMALL_TURN)
                self.move(MOVE_STEP)
                self.rotate(-SMALL_TURN)
            else:
                self.move(MOVE_STEP)
            
        