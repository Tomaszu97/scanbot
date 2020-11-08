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
        return self.send(f"MOVE:{distance}#")

    def get_distance(self):
        return int(self.send("GET_DISTANCE#"))

    def get_azimuth(self):
        self.azimuth = int(self.send("GET_AZIMUTH#"))
        self.publish_ros_odometry()
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
        
        # Draw on pole plots
        # transformed_data = np.zeros((0, 2))
        # for idx, dist in enumerate(data):
        #     ang = idx
        #     transformed_data = np.append(
        #             transformed_data, np.array([[ang, dist]]), axis=0)
        # rcol = self.main_window.randcol()
        # for ang, dist in transformed_data:
        #     self.main_window.pole_plots.place_pole(
        #         angle=ang, distance=dist, color=rcol)
        # self.main_window.pole_plots.redraw()

        # print("Correcting position according to scan (no rotation correction)")
        # try:
        #     trans, rot = self.odom_corrected_listener.lookupTransform("/odom", "/map", rospy.Time(0))
        #     self.position[0] = trans[0]
        #     self.position[1] = trans[1]
        #     self.publish_ros_odometry()
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass

        # self.rotate_tower(90)
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
        # create and set more appropriate tower frame
        msg.header.frame_id = 'base_laser'
        msg.angle_min = -(pi/2)
        msg.angle_max = pi/2
        msg.angle_increment = pi/180
        msg.range_min = 0.2
        msg.range_max = 8
        msg.ranges = [ distance/100 for distance in data ]
        self.scan_publisher.publish(msg)