#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math

def laser_publisher():
    rospy.init_node('laser_publisher', anonymous=True)
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Create a LaserScan message
        laser_scan = LaserScan()
        laser_scan.header.stamp = current_time
        laser_scan.header.frame_id = 'laser_frame'
        laser_scan.angle_min = -math.pi / 2
        laser_scan.angle_max = math.pi / 2
        laser_scan.angle_increment = math.pi / 180.0  # 1 degree resolution
        laser_scan.time_increment = (1.0 / 10.0) / 360.0  # 10Hz / 360 degrees
        laser_scan.range_min = 0.0
        laser_scan.range_max = 10.0
        laser_scan.ranges = [3.0] * 180  # 3.0 meters for all 180 beams
        laser_scan.intensities = [1.0] * 180  # Fake intensity values

        pub.publish(laser_scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        laser_publisher()
    except rospy.ROSInterruptException:
        pass
