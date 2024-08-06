# !/usr/bin/env python3

import std_msgs.msg
import rospy
from sensor_msgs.msg import Image, CompressedImage

from sensor_msgs.msg import NavSatFix
import numpy as np
from ultralytics import YOLO
import cv2 as cv
from cv_bridge import CvBridge
from math import *
from sensors.msg import imu_data
from stereo.msg import yolotag
from cv2 import aruco

class ArucoDetect:

    def __init__(self):
        # self.calib_data_path = r"/home/mrm/MRM-URC2024-NavStack/src/stereo/scripts/MultiMatrix.npz"
        # self.calib_data = np.load(self.calib_data_path)

        self.cam_mat = np.array([[527.277645, 0.0, 349.755466],
                            [0.0, 530.658217, 233.367262],
                            [0.0, 0.0, 1.0]])

        self.dist_coef = np.array([[0.236713 ,-0.169058, -0.010459, 0.021462, 0.000000]])
    
    #aruco
        # self.cam_mat = np.array([[820.3589856596768, 0.0, 539.5],
        #                     [0.0, 483.84537647631765, 959.5],
        #                     [0.0, 0.0, 1.0]])

        # dist_coef = np.array([[-1.3142589234883053e-10, -2.615728839048662e-07, -3.778678242834884e-12, -8.159479283769905e-14, 3.458838325714501e-11]])
    
    #chessboard, rospackage
        # self.cam_mat = np.array([[569.776749, 0.0, 422.286549],
        #                     [0.0, 560.773506, 243.958531],
        #                     [0.0, 0.0, 1.0]])

        # dist_coef = np.array([[0.088685, 0.055558, 0.004746, 0.058562, 0.000000]])    

        self.marker_size = 0.14
        self.marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.param_markers = cv.aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        
        self.model = YOLO("/home/mrm/MRM-URC2024-NavStack/src/stereo/models/engine/artag_v8s_final.engine")

        self.image = None

        rospy.init_node('aruco')
        
        self.rate = rospy.Rate(1)
        self.image_pub = rospy.Publisher('/arucoimg/compressed',CompressedImage, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera1/image_raw/compressed', Compresse    dImage, self.imgCallback)
        self.aruco_pub = rospy.Publisher('/aruco_detect', yolotag, queue_size=10)

    def imgCallback(self, Image):
        self.image = Image

    def detect_aruco(self, frame):
        if frame is not None:
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, _ = cv.aruco.detectMarkers(gray_frame, self.marker_dict, parameters=self.param_markers)
            aruco_msg=yolotag()
            values=[]
            isFound=False
            x=100
            y=100
            if marker_corners:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, self.marker_size, self.cam_mat, self.dist_coef)
                total_markers = range(0, marker_IDs.size)
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    isFound=True
                    cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 1, cv.LINE_AA)
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    top_left = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()
                    # print(tVec[i])
                    aruco_y = tVec[i][0][0]
                    aruco_x = tVec[i][0][2]
                    aruco_z = tVec[i][0][1]
                    print(aruco_x,aruco_y,aruco_z)
                    point = cv.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 1, 1)
                    cv.putText(frame, f"id: {ids[0]} Dist: {round(sqrt(pow(aruco_x,2)+pow(aruco_y,2)), 2)}", tuple(top_right), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1, cv.LINE_AA)
                    cv.putText(frame, f"x:{round(aruco_x,1)} y: {round(aruco_y,1)} ", tuple(bottom_right), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1, cv.LINE_AA)
                    
                    values.append([sqrt(pow(aruco_x,2)+pow(aruco_y,2)),isFound, aruco_x, aruco_y])

            if len(values)>0:
                values.sort()
                aruco_msg.isFound=values[0][1]
                aruco_msg.x=values[0][2]
                aruco_msg.y=values[0][3]*1.5

            self.aruco_pub.publish(aruco_msg)

            # frame = cv.resize(frame, (1920, 1080))
            image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.image_pub.publish(image_msg)
            # cv.imshow("ArUco Detection", display_img)
            # cv.waitKey(1)
            return isFound

    def detect_objects(self, frame):
        if frame is not None:
            color = (0,0,255)
            height, width, channels = frame.shape
            img_center_x = width / 2
            # img_center_y = height / 2
            # pil_frame = PILImage.fromarray(frame)
            pred = self.model.predict(frame,device='cuda')
            pred = pred[0]
            aruco_msg=yolotag()
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
                    if prob>0.4: 
                        isFound=True                       
                        
                        cv.rectangle(frame,(int(box[0]),int(box[1])),(int(box[2]),int(box[3])),(200,0,0),1)
                        cv.putText(frame,'Class: {cls}  Prob: {prob1:.3f}'.format(cls=clss,prob1=prob), org = (int(box[0])-20, int(box[1])-20) ,fontFace = cv.FONT_HERSHEY_DUPLEX,fontScale = 0.5,color = (200, 0, 0),thickness = 2)     
                        # gps_info = f"GPS Lat:{self.gps.latitude}"
                        # gps_info1= f"GPS Lon:{self.gps.longitude}"
                        
                        # cv.putText(frame, gps_info, (width-width/3, 40), cv.FONT_HERSHEY_DUPLEX, 0.5, (200,0,0), 2)
                        # cv.putText(frame, gps_info1, (width-width/3, 90), cv.FONT_HERSHEY_DUPLEX, 0.5, (200,0,0), 2)                        

                        # aruco_x=-9.862633167789*pow(10,-10)*pow(box_area,3)+6.17640361812*pow(10,-6)*pow(box_area,2)-0.0128755*box_area + 11.8973
                        
                        aruco_x=3

                        box_center_x = int(box[0]) + (int(box[2] - box[0])) / 2
                        box_center_y = int(box[1]) + (int(box[3] - box[1])) / 2

                        aruco_y=(box_center_x-img_center_x)/100
                        
                        
                        # if self.flag==0 and 0.8*height<=box_center_y<=1*height:
                        #     self.flag=1
                        # elif self.flag==1 and 0.8*height<=box_center_y<=1*height:
                        #     self.flag=2
                        # elif self.flag==2:
                        #     aruco_x=0
                        #     aruco_y=0

                        values.append([sqrt(pow(aruco_x,2)+pow(aruco_y,2)),isFound, aruco_x, aruco_y])
                        # print(x_values)
                        # print(values)

            if len(values)>0:
                values.sort()
                aruco_msg.isFound=values[0][1]
                aruco_msg.x=values[0][2]
                aruco_msg.y=values[0][3]
     
            self.aruco_pub.publish(aruco_msg)
            
            image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.image_pub.publish(image_msg)

            return isFound

    def run(self):
        # self.cap = cv.VideoCapture(2) 
        while not rospy.is_shutdown():
            # ret, frame = self.cap.read() 
            if self.image is not None:
                # print(frame.shape)
                frame = self.bridge.compressed_imgmsg_to_cv2(self.image)
                # frame=cv.resize(frame,(640,480),interpolation= cv.INTER_LINEAR)

                if self.detect_aruco(frame):
                    continue
            
                self.detect_objects(frame)

                

if __name__ == "__main__":
    aruco_obj = ArucoDetect()
    aruco_obj.run()