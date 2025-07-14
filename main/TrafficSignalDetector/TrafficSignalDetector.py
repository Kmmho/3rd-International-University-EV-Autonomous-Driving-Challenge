#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append('/root/catkin_ws/src/shark/src')

import cv2, os
from ultralytics import YOLO
import numpy as np
import time
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import PointCloud2, CompressedImage, Image
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

# username = os.environ.get('USER') or os.environ.get('USERNAME')

class TrafficLight():
    def __init__(self) -> None:
        self.model = YOLO('/home/lkh/catkin_ws/src/YOLO/lidar_expo/src/best.pt')   # model 입력
        self.cam_sub  = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.img_cb)
        self.cls_pub = rospy.Publisher('/traffic_signal', Int16, queue_size=1) # tf_cls publish
        self.result_img = None
        self.img = None
        self.msg = Int16()
        self.bridge = CvBridge()


    def img_cb(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    def run(self,img):
        """신호등 인식
            빨간:0, 초록:1, 노랑:5
        Args:
            img (_type_): _description_
        """
        self.results = self.model(img,verbose=False, conf= 0.55)
        self.result_img = self.results[0].plot()
        #print(np.shape(self.img))
        cv2.imshow('YOLO',self.result_img)
        cv2.waitKey(1)
        traffic_light_color = -1
        for r in self.results:
            size = r.boxes.xywh.cpu().numpy()
            if len(size) !=0:
                if size[0][0] > 170 and size[0][1]>20 and size[0][2]> 30:
                    self.msg.data = r.boxes.cls.cpu().numpy()
                    if len(self.msg.data) == 2:
                        if self.msg.data[0] == 0.0 or self.msg.data[1] == 0.0:
                            traffic_light_color = 0
                        elif self.msg.data[0] == 1.0 or self.msg.data[1] == 1.0:
                            traffic_light_color = 1
                        elif self.msg.data[0] == 5.0 or self.msg.data[1] == 5.0:
                            traffic_light_color = 5
                        else:
                            traffic_light_color = -1
                    if len(self.msg.data) == 1:
                        if self.msg.data[0] == 0.0:
                            traffic_light_color = 0
                        elif self.msg.data[0] == 1.0:
                            traffic_light_color = 1
                        elif self.msg.data[0] == 5.0:
                            traffic_light_color = 5
                        else:
                            traffic_light_color = -1
            else:
                self.msg.data = list()
            self.msg.data = traffic_light_color    
            self.cls_pub.publish(self.msg)
    
    def do(self):
        rospy.init_node('yolo_run')
        print('start yolo node')
        rate = rospy.Rate(12)
        
        while not rospy.is_shutdown():
            self.run(self.img)
            rate.sleep()
            rospy.loginfo(f'light: {self.msg.data}')
    
class Person():
    def __init__(self) -> None:
        self.model = YOLO('/root/catkin_ws/src/shark/src/yolo/person_s.pt')  # model 입력
        self.cls_pub = rospy.Publisher('/person', String, queue_size=1) # tf_cls publish
        
        self.result_img = None
        self.msg = String()
        self.msg.data = 'go'

    def run(self,img):
        self.results = self.model(img,verbose=False,classes=[0], conf=0.50)
        self.result_img = self.results[0].plot()
        cv2.imshow('YOLO',self.result_img)
        for r in self.results:
            size = r.boxes.xywh.cpu().numpy()
            # rospy.loginfo(r.boxes.cls.cpu().numpy())
            if len(size) != 0:
                x = size[0][0]
                y = size[0][1]
                w = size[0][2]
                h = size[0][3]
                if (x <480 and x > 185) and (y >230) and w > 20:
                    self.msg.data = 'stop'
                elif (x <550 and x > 80) and (y >180) and w > 5:
                    self.msg.data = 'slow'
                else:
                    self.msg.data = 'go'
                # rospy.loginfo(f'x:{x:0.1f}, y:{y:0.1f}, w:{w:0.1f}, h:{h:0.1f}')
            else:
                # rospy.loginfo('not detect persons')
                self.msg.data = 'go'                
        self.cls_pub.publish(self.msg)



if __name__ == '__main__':
    TSD = TrafficLight()
    TSD.do()