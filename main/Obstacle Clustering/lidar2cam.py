#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import math
import time
import rospkg
import os
import ros_numpy
import json
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import Float32MultiArray, String
import sensor_msgs.point_cloud2 as pc2
from ultralytics import YOLO
from std_msgs.msg import Float32, Int16,Float32MultiArray
parameters_cam = {
    "WIDTH": 1280, # image width
    "HEIGHT": 720, # image height
    "FOV": 55, # Field of view
    "X": 1.67, # meter
    "Y": 0,
    "Z": 1.28,
    "YAW": 0,
    "PITCH": 0.0,
    "ROLL": 0
}

parameters_lidar = {
    "X": 1.67, # meter
    "Y": 0,
    "Z": 1.20,
    "YAW": 0, # -7.5*math.pi/180.0. # radian
    "PITCH": 0.0,
    "ROLL": 0
}

def translationMtx(x, y, z):

    M = np.array([[1,        0,          0,          x],
                  [0,        1,          0,          y],
                  [0,        0,          1,          z],
                  [0,        0,          0,          1],
                  ])
    
    return M

def rotationMtx(yaw, pitch, roll):

    R_x = np.array([[1,      0,             0,               0],
                    [0,      math.cos(roll), -math.sin(roll) ,0],
                    [0,      math.sin(roll), math.cos(roll)  ,0],
                    [0,      0,             0,               1],
                    ])
    
    R_y = np.array([[math.cos(pitch),   0,  math.sin(pitch) , 0],
                    [0,                 1,  0               , 0],
                    [-math.sin(pitch), 0,  math.cos(pitch) , 0],
                    [0,                 0,  0               , 1],
                    ])
    
    R_z = np.array([[math.cos(yaw),     -math.sin(yaw) , 0 , 0],
                    [math.sin(yaw),      math.cos(yaw) , 0 , 0],
                    [0,                 0              , 1 , 0],
                    [0,                 0              , 0 , 1],
                    ])
    R = np.matmul(R_x, np.matmul(R_y, R_z))

    return R


def transformMTX_lidar2cam(params_lidar, params_cam):
    lidar_pos = [params_lidar.get(i) for i in (["X", "Y", "Z"])]
    cam_pos = [params_cam.get(i) for i in (["X", "Y", "Z"])]

    x_rel = cam_pos[0] - lidar_pos[0]
    y_rel = cam_pos[1] - lidar_pos[1]
    z_rel = cam_pos[2] - lidar_pos[2]

    R_T = np.matmul(translationMtx(x_rel, y_rel, z_rel), rotationMtx(np.deg2rad(-90), 0., 0.))
    R_T = np.matmul(R_T, rotationMtx(0, 0., np.deg2rad(-90)))

    R_T = np.linalg.inv(R_T)

    return R_T


def project2img_mtx(params_cam):
    fc_x = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    cx = params_cam["WIDTH"]/2
    cy = params_cam["HEIGHT"]/2

    R_f = np.array([[fc_x, 0, cx],
                    [0, fc_y, cy]])

    return R_f


class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):
        self.model = YOLO('/home/jeongwoo/catkin_ws/src/lidar/src/person_s.pt')  # model 입력
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)
        self.proj_mtx = project2img_mtx(params_cam)
        self.bridge = CvBridge()
        self.lidar_sub = rospy.Subscriber('/voxel',PointCloud2, self.lidar_cb)
        self.lidar_sub_land = rospy.Subscriber('/land',PointCloud2, self.lidar_cb_land)
        self.cam_sub  = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.img_cb)
        self.img_pub  = rospy.Publisher('/result_img', CompressedImage, queue_size=1)
        self.dist_pub = rospy.Publisher('/obs_dist', Float32MultiArray, queue_size=1)
        self.yolo_detect = rospy.Publisher('/yolo_data',Int16,queue_size=1)
        self.lane_num_pub = rospy.Publisher('/lane_num',String,queue_size=1)
        self.points_land = None
        self.angle = None
        self.lane_num = None
        self.points = None
        self.img = None
        self.results = None
        self.dist = Float32MultiArray()
        self.car_pub = rospy.Publisher('/car_info', Float32MultiArray, queue_size=1)
        self.car_info = Float32MultiArray()
        self.person_pub = rospy.Publisher('/person_info',Float32MultiArray,queue_size=1)
        self.person_info = Float32MultiArray()
        
    def lidar_cb(self, msg):
        self.points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    def lidar_cb_land(self, msg):
        self.points_land = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    def img_cb(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")


    def transform_lidar2cam(self, xyz_p):

        x, y = xyz_p[:,0], xyz_p[:,1]
        x, y = x[:,np.newaxis], y[:,np.newaxis]
        d = np.array([np.sqrt(x**2 + y**2) for x, y in xyz_p[:, :2]])
        theta = np.arccos(xyz_p[:, 0] / d)
        theta = theta * (180 / np.pi)
        self.angle = theta[:, np.newaxis]
        d = d[:, np.newaxis]
        up = np.concatenate([self.RT.T, np.zeros((4,2))], axis=1)
        down = np.concatenate([np.zeros((2,4)), np.eye(2, dtype=int)], axis=1)
        RT = np.vstack([up, down])
        xyz_c = np.matmul(np.concatenate([xyz_p, d, x, y], axis=1), RT)

        return xyz_c
    
    def project_pts2img(self, xyz_c, crop=True):

        xyz_c = xyz_c.T

        xc, yc, zc, d, xo, yo = xyz_c[0, :].reshape([1, -1]), xyz_c[1,:].reshape([1, -1]), xyz_c[2,:].reshape([1, -1]),\
                                 xyz_c[3,:].reshape([1, -1]), xyz_c[4,:].reshape([1, -1]), xyz_c[5,:].reshape([1, -1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = np.vstack([xyi, d, xo, yo, self.angle.T])
        #print(xyi)

        xyi = xyi[:, :].T

        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass
        #print(np.shape(xyi))
        return xyi
    def crop_pts(self, xyi):
        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:,0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:,1]<self.height), :]
        return xyi
    
    def draw_pts_img(self, img, xi, yi, di):
        point_np = img
        # 5m ~ 30m normalize
        # B, G, R
        for ctr in zip(xi, yi, di):
            ctp = (int(ctr[0]), int(ctr[1]))
            #print(ctr)
            dist = ctr[2]
            color = None
            if dist < 10:
                color = (0, 0, 255) # red
            elif dist < 20:
                color = (0, 255, 0) # green
            else:
                color = (255, 0, 0)

            point_np = cv2.circle(point_np, ctp, 2, color, -1)
        
        return point_np

    def detect_person(self, xyi, img):
        dists = [50]
        car_list = []
        person_list = []
        for r in self.results:
            num_box = len(r.boxes)
            size = r.boxes.xyxy.cpu().numpy()
            for i in range(num_box):
                if r.boxes.cls[i] == 0:
                    x = size[i][0]
                    y = size[i][1]
                    x_e = size[i][2]
                    y_e = size[i][3]
                    
                    xy_p = xyi[(xyi[:,0] > x) & (xyi[:,0] < x_e) & (xyi[:,1] > y) & (xyi[:,1] < y_e)]
                    
                    if len(xy_p) > 0:  
                        left_most_point = xy_p[xy_p[:,0].argmin()]
                        right_most_point = xy_p[xy_p[:,0].argmax()]
                        min_x_person = min(xy_p[:,3])
                        min_y_person = min(xy_p[:,4])
                        print("person")                        
                        print(f"{i}th box's left-most point x,y: {left_most_point[:2]}, right-most point x,y: {right_most_point[:2]}")
                        person_list.append(float(min_x_person))
                        person_list.append(float(min_y_person))
                        distance_x = right_most_point[0] - left_most_point[0]
                        distance_y = right_most_point[4] - left_most_point[4]
                        #print(f"{i}th box's horizontal distance: {distance_x}")
                    else:
                        print("No points found in the bounding box.")
               
                    # 최소 거리 계산
                    dist = min(xy_p[:,2]) if len(xy_p) > 0 else 50
                    #print("{0}th dist : {1}".format(i, dist))
                    dists.append(dist)
                    # YOLO bounding box에 거리 정보를 이미지에 추가  
                    cv2.putText(img, f"Dist: {dist:.2f}", (int(x), int(y_e) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                elif r.boxes.cls[i] == 2: # 차량
                    x = size[i][0]
                    y = size[i][1]
                    x_e = size[i][2]
                    y_e = size[i][3]
                    xy_p = xyi[(xyi[:,0] > x) & (xyi[:,0] < x_e) & (xyi[:,1] > y) & (xyi[:,1] < y_e + (y_e-y)/2)]
                    if len(xy_p) > 0:  
                        min_x = min(xy_p[:,3])
                        min_y = min(xy_p[:,4])
                        
                        print("car")
                        car_list.append(float(min_x))
                        car_list.append(float(min_y))

                    # 가장 왼쪽 포인트

        if len(car_list):
            if len(car_list) == 0:
                print("No car")
                car_list = [-1, -1]
            elif len(car_list) > 2:
                # 최소 거리 차량만 추출
                even_index_elements = car_list[::2]  
                min_value = min(even_index_elements)  
                min_index = even_index_elements.index(min_value) * 2  
                car_list = car_list[min_index:min_index+2] 
            
        else:
            print("No car")
            car_list = [-1, -1]
        self.car_info.data = car_list
        self.car_pub.publish(self.car_info)
        self.person_info.data = person_list
        self.person_pub.publish(self.person_info)
        self.dist.data = dists
        self.dist_pub.publish(self.dist)


    
    #-------------------------caculate lateral offset & lane_number display-----------------------


    def extract_yellow(self,img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([15, 100, 160])
        upper_yellow = np.array([40, 180, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        img = cv2.bitwise_and(img, img, mask=yellow_mask)
        return img    
    
    def extract_blue(self,img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([100, 100, 200])
        upper_yellow = np.array([140, 180, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        img = cv2.bitwise_and(img, img, mask=yellow_mask)
        return img 

    
    def filter_and_calculate_lateral_minimum_distance_and_show(self, img, xyi):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 노란색 범위 정의
        lower_yellow = np.array([15, 100, 160])
        upper_yellow = np.array([255, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        min_distance = float('inf')
        min_point = None
        num = None
        for i in range(xyi.shape[0]):
            x_img, y_img = int(xyi[i, 0]), int(xyi[i, 1])
            distance = xyi[i, 2]  # 거리(distance) 값 추출
            # 포인트가 노란색 차선 위에 있고, 최소 거리를 갖는지 확인
            if mask[y_img, x_img] > 0 and distance < min_distance:
                min_distance = distance
                min_point = xyi[i]


        if min_point is not None:
            x_img, y_img = int(min_point[0]), int(min_point[1])
            dis, theta =min_point[2], min_point[5]
            # lateral_minimum_distance 계산
            lateral_minimum_distance = abs(dis * math.sin(theta * (np.pi / 180)))

            # 이미지 위에 최소 거리에 해당하는 점 표시
            cv2.circle(img, (x_img, y_img), 10, (0, 255, 0), -1)
            
            # lateral_minimum_distance 값 이미지에 표시
            text = f"Lateral Min Dist: {lateral_minimum_distance:.2f}"
            if lateral_minimum_distance < 2.7:
                self.lane_num = '1st lane'
                print(self.lane_num)
            elif lateral_minimum_distance < 6 and lateral_minimum_distance > 3:
                self.lane_num = '2nd lane'
                print(self.lane_num)
            else:
                self.num = '3rd lane'
                print(self.lane_num)

            cv2.putText(img, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(img, self.lane_num, (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.imshow('lane_number', img)
            return num
        else:
            text2 = f"NO points in Lane"
            cv2.putText(img, text2, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            return None

    #======================line detection =======================
    def line(self,img):
        
        WIDTH, HEIGHT = 1280, 720
        lane_image = np.copy(img)
        yellow_mask = self.extract_yellow(lane_image)
        canny_image = self.canny(yellow_mask)
        #cv2.imshow('CANNY', canny_image)
        # cropped_image = self.roi(canny_image)
        # cv2.imshow('Crop', canny_image)
        #lines = cv2.HoughLinesP(cropped_image,2,np.pi/180, 100, np.array([]),minLineLength=50, maxLineGap=85)
        lines = cv2.HoughLinesP(canny_image,2,np.pi/180, 100, np.array([]),minLineLength=50, maxLineGap=85)
        
        
        if lines is not None:
               
            averaged_lines = self.average_slope_intercept(lane_image, lines)
            line_image = self.display_lines(lane_image,averaged_lines)
            combo_image = cv2.addWeighted(lane_image,0.8,line_image,1.1, 0.0)
            #cv2.imshow('combo!', combo_image)
            return combo_image
        else:
            return None
        
        
    def canny(self,img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        canny = cv2.Canny(blur,50,150)
        return canny
    
    def roi(self,img):
        WIDTH, HEIGHT = 1280, 720
        triangle = np.array([[(280,HEIGHT),(1100,HEIGHT),(620,280)]])
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, triangle, 255) 
        masked_img = cv2.bitwise_and(img,mask)
        #plt.imshow(masked_img)
        #cv2.imshow('gray',masked_img)
        return masked_img
    
    def display_lines(self,img,lines):
        line_image = np.zeros_like(img)
        if lines is not None:
            for line in lines:
                if line is not None:
                    try:
                        x1,y1,x2,y2 = line
                    except TypeError:
                        x1,y1,x2,y2 = line[0]
                        
                    cv2.line(line_image,(x2,y2), (x1,y1),(255,0,0),50)
        return line_image
               
    def make_coordinates(self,img,line_parameters):
        slope, intercept = line_parameters
        y1 = img.shape[0]
        y2 = int(y1*(3/5))
        if slope != 0:
            x1 = int((y1-intercept)/slope)
            x2 = int((y2-intercept)/slope)
        else:
            x1 = x2 = int((img.shape[1]/2))
        points = [x1,y1,x2,y2]
        
        return points
    
    def average_slope_intercept(self, img, lines):
        left_fit = []
        right_fit = []
        
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            if (x2 - x1) == 0:
                continue
            slope = ((y1 - y2) / (x1 - x2))

            intercept = y1 - slope * x1
            
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
                
        left_line = None
        right_line = None
        if left_fit:
            left_fit_average = np.average(left_fit, axis=0)
            left_line = self.make_coordinates(img, left_fit_average)
        
        lines = [left_line]
        return lines
    
#---------------------------------lane_number main func-------------------------------------
    def lane_number(self):
        # 라이다 -> 카메라 
    
        xyz_c_land = self.transform_lidar2cam(self.points_land)
        xyi_land = self.project_pts2img(xyz_c_land)
        xyi_land = self.crop_pts(xyi_land)
        img = self.line(self.img)
        img = self.extract_blue(img)
        self.filter_and_calculate_lateral_minimum_distance_and_show(img, xyi_land)
        self.lane_num_pub.publish(self.lane_num)
        cv2.waitKey(1)
    

    ###==============================================================

    def main(self):
        # 라이다 -> 카메라 
        xyz_c = self.transform_lidar2cam(self.points)
        xyi = self.project_pts2img(xyz_c)
        xyi = self.crop_pts(xyi)
        #print(np.shape(xyi[:,0]))

        # YOLO
        img_model = self.img
        self.results = self.model(img_model,verbose=False, conf= 0.55)
        result_img = self.results[0].plot()
        boxes = self.results[0].boxes
        num_boxes = len(boxes)
        # cv2.imshow('yolo', result_img)
        self.detect_person(xyi,result_img)
        img = self.draw_pts_img(result_img, xyi[:,0], xyi[:,1], xyi[:,2])
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
        # self.img_pub.publish(img_msg)
        if num_boxes != 0:
            self.yolo_detect.publish(int(boxes.cls[0]))
        rate.sleep()
        cv2.imshow('transform_result', img)
        cv2.waitKey(1)

    

if __name__=="__main__":
    rospy.init_node("lidar_transform")

    transformer = LIDAR2CAMTransform(parameters_cam, parameters_lidar)
    rospy.wait_for_message('/voxel', PointCloud2)
    rospy.wait_for_message('/land', PointCloud2)
    rospy.wait_for_message('/obs', PointCloud2)
    rospy.wait_for_message('/image_jpeg/compressed', CompressedImage)
    rospy.loginfo('start')
    

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        transformer.main()
        transformer.lane_number()
        rate.sleep()
    
    