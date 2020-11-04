#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
import math

def update_scan():
    pos_bcaster = tf.TransformBroadcaster()
    scan_publisher = rospy.Publisher('scan', LaserScan, queue_size=10)
    rate = rospy.Rate(1)

    position = 0
    while not rospy.is_shutdown():
        # update odometry
        pos_bcaster.sendTransform(
            translation=(position, 0, 0),
            rotation=tf.transformations.quaternion_from_euler(0, 0, 0),
            time=rospy.Time.now(),
            child='scanbot',
            parent='map'
        )
        position += 0.1

        # update laser scan
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        # create and set more appropriate tower frame
        msg.header.frame_id = 'scanbot'
        msg.angle_min = 0
        msg.angle_max = math.pi
        msg.angle_increment = math.pi/180
        msg.range_min = 0.2
        msg.range_max = 8
        for i in range(180):
            msg.ranges.append(0.5+0.02*i)
        scan_publisher.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('scanbot_communicator')
    try:
        update_scan()
    except rospy.ROSInterruptException:
        pass

