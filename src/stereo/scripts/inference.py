#!/usr/bin/env python
import std_msgs.msg
import rospy
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import NavSatFix
import numpy as np
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
from math import *
from sensors.msg import imu_data
from stereo.msg import yolotag

class YOLOObjectDetectionNode:
    def __init__(self,type):

        self.bridge = CvBridge()
        
        if type==1:
            self.model = YOLO("/home/mrm/MRM-URC2024-NavStack/src/stereo/models/engine/mallet.engine")
        elif type==2:
            self.model = YOLO("/home/mrm/MRM-URC2024-NavStack/src/stereo/models/engine/bottle.engine")
        # elif type==3:
        #     self.model = YOLO("/home/mrm/MRM-URC2024-NavStack/src/stereo/scripts/artag.pt")

        self.navimage = None
        self.hazimage = None
        self.image = None
        self.flag=0

        self.imu = None
        self.gps = None

        rospy.init_node('yolo')
        
        self.rate = rospy.Rate(20)
        self.image_pub = rospy.Publisher('/img/compressed',CompressedImage, queue_size=10)
        self.imagehaz_sub = rospy.Subscriber('/camera2/image_raw/compressed', CompressedImage, self.imgHazCallback)
        self.imagenav_sub = rospy.Subscriber('/camera1/image_raw/compressed', CompressedImage, self.imgNavCallback)
#       self.image_pub = rospy.Publisher('/img', CompressedImage, queue_size=10)
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.gpsCallback)
        self.obj_pub = rospy.Publisher('/aruco_detect', yolotag, queue_size=10)

    def gpsCallback(self,fix):
        self.gps=fix

    def imgNavCallback(self, Image):
        self.navimage = Image

    def imgHazCallback(self, Image):
        img = self.bridge.compressed_imgmsg_to_cv2(Image)
        rotimg=cv2.rotate(img,cv2.ROTATE_180)
        self.hazimage = self.bridge.cv2_to_compressed_imgmsg(rotimg)

        



    def detect_objects(self, frame):
        if frame is not None:
            color = (0,0,255)
            height, width, channels = frame.shape
            print(height,width)
            img_center_x = width / 2
            # img_center_y = height / 2
            # pil_frame = PILImage.fromarray(frame)
            pred = self.model.predict(frame,device='cuda')
            pred = pred[0]
            obj_msg=yolotag()
            values=[]
            isFound=False
            x=100
            y=100
            if len(pred.boxes) > 0:
                for value in pred.boxes:
                    box = value.xyxy
                    box = (list(list(box.cpu())[0]))
                    clss = float(list(value.cls.cpu())[0])

                    prob = float(list(value.conf.cpu())[0])
                    box_area = float((box[2] - box[0]) * (box[3] - box[1]))
                    if prob>0.5: 
                        isFound=True                       
                        
                        cv2.rectangle(frame,(int(box[0]),int(box[1])),(int(box[2]),int(box[3])),(200,0,0),1)
                        cv2.putText(frame,'Class: {cls}  Prob: {prob1:.3f}'.format(cls=clss,prob1=prob), org = (int(box[0])-20, int(box[1])-20) ,fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale = 0.5,color = (200, 0, 0),thickness = 2)     
                        # gps_info = f"GPS Lat:{self.gps.latitude}"
                        # gps_info1= f"GPS Lon:{self.gps.longitude}"
                        
                        # cv2.putText(frame, gps_info, (width-width/3, 40), cv2.FONT_HERSHEY_DUPLEX, 0.5, (200,0,0), 2)
                        # cv2.putText(frame, gps_info1, (width-width/3, 90), cv2.FONT_HERSHEY_DUPLEX, 0.5, (200,0,0), 2)                        

                        # obj_x=-9.862633167789*pow(10,-10)*pow(box_area,3)+6.17640361812*pow(10,-6)*pow(box_area,2)-0.0128755*box_area + 11.8973
                        
                        obj_x=3

                        box_center_x = int(box[0]) + (int(box[2] - box[0])) / 2
                        box_center_y = int(box[1]) + (int(box[3] - box[1])) / 2

                        obj_y=(box_center_x-img_center_x)/100
                        print(box_center_y)
                        
                        if self.flag==0 and 0.8*height<=box_center_y<=1*height:
                            self.flag=1
                        elif self.flag==1 and 0.83*height<=box_center_y<=1*height:
                            self.flag=2
                        elif self.flag==2:
                            obj_x=0
                            self.flag=0
                            # obj_y=0

                        values.append([sqrt(pow(obj_x,2)+pow(obj_y,2)),isFound, obj_x, obj_y])
                        # print(x_values)
                        # print(values)

            if len(values)>0:
                values.sort()
                obj_msg.isFound=values[0][1]
                obj_msg.x=values[0][2]
                obj_msg.y=values[0][3]*1.3
     
            self.obj_pub.publish(obj_msg)
            
            image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.image_pub.publish(image_msg)

    def run(self):
        # cap = cv2.VideoCapture(1)
        while not rospy.is_shutdown():
            # ret,frame = cap.read()
            print(self.flag)
            if self.flag==0:
                self.image=self.navimage
            # if self.flag==1:
            #     self.image=self.hazimage
            if self.image is not None:
                frame = self.bridge.compressed_imgmsg_to_cv2(self.image)
                self.detect_objects(frame)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        yolo_node = YOLOObjectDetectionNode(int(input("Enter model type: ")))
        yolo_node.run()
    except rospy.ROSInterruptException:
        pass


# sudo apt install ros-noetic-image-transport-plugins ros-noetic-compressed-image-transport ros-noetic-theora-image-transport ros-noetic-compressed-depth-image-transport
