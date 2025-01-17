#!/usr/bin/env python3
from std_msgs.msg import Header
import serial
import rospy
from geometry_msgs.msg import Vector3
from sensors.msg import imu_data
ser = serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/ttyUSB2'
ser.open()
rospy.init_node('IMU', anonymous=True)
pub = rospy.Publisher('/imu_data', imu_data, queue_size=1)
msg = imu_data()
rospy.sleep(2)
while not rospy.is_shutdown():
    try:
        data = ser.readline().decode('utf-8').rstrip().split(",")
        print(data)
    except:
        print('skipped data line')
        continue
    #print(data)
    if(len(data)!=6):
        continue
    #print(data)
    try:
        #msg.header = Header(stamp = rospy.Time.now(),frame_id="imu")
        
        msg.acceleration.x =  float(data[0])
        msg.acceleration.y =  float(data[1])
        msg.acceleration.z =  float(data[2])
        msg.orientation.x = float(data[5])
        msg.orientation.y = float(data[4])
        msg.orientation.z = float(data[3])
    except Exception as e:
        print(e)
        continue
    pub.publish(msg)

