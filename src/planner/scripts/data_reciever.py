#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from sensors.msg import imu_data
import csv
from sensor_msgs.msg import Image
from planner.srv import Data,DataResponse
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix 

class DataSaver:
    count=1
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.imu_data = None
        self.gps_data = None
        self.csv_filename = '/home/mrm/MRM-URC2024-NavStack/Data/data.csv'
        # self.imu_sub = rospy.Subscriber("/imu_data", Imu, self.imuCallback)
        self.imu_sub = rospy.Subscriber("/imu_data", imu_data, self.imuCallback)
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.gpsCallback)
        self.img_sub = rospy.Subscriber("/img",Image, self.imgCallback)

    def imgCallback(self,img):
        self.image = img
    
    def imuCallback(self,imu_msg):
        self.imu_data=imu_msg    

    def gpsCallback(self,fix):
        self.gps_data=fix

    def saver(self, req):
        data = req.status
        image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")

        if data:
            image_filename = "/home/mrm/MRM-IRC2024-NavStack/Data/object"+str(DataSaver.count)+".png"
            cv2.imwrite(image_filename, image)
            DataSaver.count+=1
            rospy.loginfo("Screenshot saved as %s", image_filename)
            with open(self.csv_filename, 'a') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([self.gps_data.latitude, self.gps_data.longitude,self.imu_data.acceleration.x, self.imu_data.acceleration.y, self.imu_data.acceleration.z,self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z])
            rospy.loginfo("GPS and IMU data written to CSV")
        else:
            rospy.logwarn("No image received to capture a screenshot.")
        return DataResponse(success=1)

def main():
    rospy.init_node('data_receiver_node')
    data=DataSaver()
    data_saver = rospy.Service('data_saver', Data, data.saver)
    rospy.spin()

if __name__ == '__main__':
    main()

