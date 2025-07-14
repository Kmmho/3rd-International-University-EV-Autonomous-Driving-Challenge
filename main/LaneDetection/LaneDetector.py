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
from sensor_msgs.msg import PointCloud2, CompressedImage, Image
import sensor_msgs.point_cloud2 as pc2
from ultralytics import YOLO
import torch
torch.cuda.set_device(0)
# // down
# parameters_cam = {
#     "WIDTH": 1280, # image width
#     "HEIGHT": 720, # image height
#     "FOV": 55, # Field of view
#     "X": 3.90, # meter
#     "Y": 0,
#     "Z": 0.48,
#     "YAW": 0,
#     "PITCH": 0.0,
#     "ROLL": 0
# }

# parameters_lidar = {
#     "X": 3.90, # meter
#     "Y": 0,
#     "Z": 0.40,
#     "YAW": 0, # -7.5*math.pi/180.0. # radian
#     "PITCH": 0.0,
#     "ROLL": 0
# }

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

parameters_vehicle = {
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
        self.model = YOLO('/home/jeongwoo/catkin_ws/src/lidar_expo/src/person_s.pt')  # model 입력
        self.model.to('cuda')
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)
        self.proj_mtx = project2img_mtx(params_cam)
        self.bridge = CvBridge()
        self.lidar_sub = rospy.Subscriber('/voxel',PointCloud2, self.lidar_cb)
        self.lidar_sub_land = rospy.Subscriber('/land',PointCloud2, self.lidar_cb_land)

        self.cam_sub  = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.img_cb)
        # self.image_pub = rospy.Publisher('/result_img', Image, queue_size=1)
        self.points = None
        self.points_land = None
        self.img = None
        self.results = None
        self.angle = None

        # self.st = time.time()

    def lidar_cb(self, msg):
        self.points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        #self.points = self.points[:, [1, 0, 2]]
        #print(np.shape(self.points))

    def lidar_cb_land(self, msg):
        self.points_land = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    def img_cb(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # print(time.time()-self.st)
        # self.st = time.time()
        # cv2.imshow("img", self.img)
        # cv2.waitKey(1)

    # distance 정보를 xc에 concat함.
    def transform_lidar2cam(self, xyz_p):

        # lidar xy 열벡터로 저장
        x, y = xyz_p[:,0], xyz_p[:,1]
        x, y = x[:,np.newaxis], y[:,np.newaxis]
        # d 계산
        d = np.array([np.sqrt(x**2 + y**2) for x, y in xyz_p[:, :2]])
    
        # a (각도를 담을 변수) 계산
        # 이미 계산된 d를 이용하여 각도(theta) 계산
        theta = np.arccos(xyz_p[:, 0] / d)
        theta = theta * (180 / np.pi)
        
        # d와 theta를 열 벡터로 변환
        d = d[:, np.newaxis]
        self.angle = theta[:, np.newaxis]

        # R_T 6x6 행렬로 만들어주기
        up = np.concatenate([self.RT.T, np.zeros((4,2))], axis=1)
        down = np.concatenate([np.zeros((2,4)), np.eye(2, dtype=int)], axis=1)
        RT = np.vstack([up, down])

        # xyz_c 계산 시 d와 lidar의 x, y를 함께 concatenate
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
        #print(xyi)
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
        for r in self.results:
            size = r.boxes.xyxy.cpu().numpy()
            print("num cls :", len(size))
            if len(size) != 0:
                for i in range(len(size)):
                    x = size[i][0]
                    y = size[i][1]
                    x_e = size[i][2]
                    y_e = size[i][3]
                    
                    xy_p = xyi[(xyi[:,0] > x) & (xyi[:,0] < x_e) & (xyi[:,1] > y) & (xyi[:,1] < y_e)]
                    
                    if len(xy_p) > 0:  
                        left_most_point = xy_p[xy_p[:,0].argmin()]
                        right_most_point = xy_p[xy_p[:,0].argmax()]
                        
                        print(f"{i}th box's left-most point x,y: {left_most_point[3:5]}, right-most point x,y: {right_most_point[3:5]}")
                        print(left_most_point)
                        # lidar에서의 point cloud 거리
                        distance_y = abs(right_most_point[4] - left_most_point[4])
                        print(f"{i}th box's horizontal distance: {distance_y}")
                        # YOLO bounding box에 거리 정보를 이미지에 추가  
                        # 가장 왼쪽 포인트의 좌표를 문자열로 포맷하여 출력
                        most_left_str = f"({left_most_point[3]:.2f}, {left_most_point[4]:.2f})"
                        cv2.putText(img, f"most-left: {most_left_str}", (int(x), int(y_e) + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                        # 가장 오른쪽 포인트의 좌표를 문자열로 포맷하여 출력
                        most_right_str = f"({right_most_point[3]:.2f}, {right_most_point[4]:.2f})"
                        cv2.putText(img, f"most-right: {most_right_str}", (int(x), int(y_e) + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                        cv2.putText(img, f"p2p distance: {distance_y:.2f}", (int(x), int(y_e) + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                    else:
                        print("No points found in the bounding box.")
                    
                    # 최소 거리 계산
                    dist = min(xy_p[:,2]) if len(xy_p) > 0 else 0
                    print("{0}th dist : {1}".format(i, dist))

                    # 평균 각도 계산
                    angle = xy_p[:,5].mean() if len(xy_p) > 0 else 0
                    print("{0}th angle : {1}".format(i, angle))
                    
                    # YOLO bounding box에 거리 정보를 이미지에 추가  
                    cv2.putText(img, f"Dist: {dist:.2f}", (int(x), int(y_e) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # YOLO bounding box에 각도 정보를 이미지에 추가
                    cv2.putText(img, f"Angle: {angle:.2f}", (int(x), int(y_e) + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
        return img
    
    def cam2BEV(self, img):
        x1, y1 = 480, 400
        x2, y2 = 800, 400
        x3, y3 = 0, 550
        x4, y4 = 1280, 550

        # DOWN
        # x1, y1 = 480, 350
        # x2, y2 = 800, 350
        # x3, y3 = 0, 650
        # x4, y4 = 1280, 650
        width, height = 1280, 720

        # 이미지에 점을 찍습니다. (꼭짓점 좌표에 해당)
        # cv2.circle(img, (x1, y1), 5, (0, 0, 255), -1) # 빨간색 점
        # cv2.circle(img, (x2, y2), 5, (0, 255, 0), -1) # 초록색 점
        # cv2.circle(img, (x3, y3), 5, (255, 0, 0), -1) # 파란색 점
        # cv2.circle(img, (x4, y4), 5, (255, 255, 0), -1) # 노란색 점
        # cv2.imshow("Image with Points", img)
        # 원본 이미지에서 4개의 꼭짓점 좌표 (x, y)
        src = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], dtype = "float32")

        # 목표 이미지에서의 꼭짓점 좌표 (일반적으로 직사각형)
        dst = np.array([[0, 0], [width, 0], [0, height], [width, height]], dtype = "float32")

        # Homography Matrix 계산
        M = cv2.getPerspectiveTransform(src, dst)

        # 이미지 변환
        img = cv2.warpPerspective(img, M, (width, height))

        return img
    
    def BEV2cam(self, img):
        # BEV 변환 시 사용했던 원본 이미지의 꼭짓점 좌표
        x1, y1 = 480, 400
        x2, y2 = 800, 400
        x3, y3 = 0, 550
        x4, y4 = 1280, 550
        width, height = 1280, 720

        # 원본 이미지에서의 꼭짓점 좌표
        src = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], dtype="float32")

        # BEV 이미지에서의 꼭짓점 좌표
        dst = np.array([[0, 0], [width, 0], [0, height], [width, height]], dtype="float32")

        # 역 Homography Matrix 계산
        M_inverse = cv2.getPerspectiveTransform(dst, src)

        original_width, original_height = 1280, 720

        # BEV 이미지를 원본 이미지로 역변환
        img = cv2.warpPerspective(img, M_inverse, (original_width, original_height))

        return img
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
                num = '1st lane'
                print(num)
            elif lateral_minimum_distance < 6 and lateral_minimum_distance > 3:
                num = '2nd lane'
                print(num)
            else:
                num = '3rd lane'
                print(num)

            cv2.putText(img, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(img, num, (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            
            cv2.imshow('Result', img)
            cv2.waitKey(1)
          
            
            return lateral_minimum_distance
        else:
            text2 = f"NO points in Lane"
            cv2.putText(img, text2, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            return None

    ###============================================================
    def line(self,img):
        
        WIDTH, HEIGHT = 1280, 720
        lane_image = np.copy(img)
        yellow_mask = self.extract_yellow(lane_image)
        canny_image = self.canny(yellow_mask)
        cv2.imshow('CANNY', canny_image)
        # cropped_image = self.roi(canny_image)
        # cv2.imshow('Crop', canny_image)
        #lines = cv2.HoughLinesP(cropped_image,2,np.pi/180, 100, np.array([]),minLineLength=50, maxLineGap=85)
        lines = cv2.HoughLinesP(canny_image,2,np.pi/180, 100, np.array([]),minLineLength=50, maxLineGap=85)
        
        
        if lines is not None:
               
            averaged_lines = self.average_slope_intercept(lane_image, lines)
            line_image = self.display_lines(lane_image,averaged_lines)
            combo_image = cv2.addWeighted(lane_image,0.8,line_image,1.1, 0.0)
            cv2.imshow('combo!', combo_image)
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
        cv2.imshow('gray',masked_img)
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
    

    ###==============================================================
    
    def main(self):
        # 라이다 -> 카메라 
        xyz_c = self.transform_lidar2cam(self.points)
        xyi = self.project_pts2img(xyz_c)
        xyi = self.crop_pts(xyi)
        #print(np.shape(xyi[:,0]))

        # YOLO
        self.results = self.model(self.img,verbose=False, conf= 0.55)
        result_img = self.results[0].plot()
        # cv2.imshow('yolo', result_img)
        result_img = self.detect_person(xyi, result_img)
        img = self.draw_pts_img(result_img, xyi[:,0], xyi[:,1], xyi[:,2])
        # img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        # self.image_pub.publish(img_msg)
        # img = self.cam2BEV(img)

        cv2.imshow('transform_result', img)
        
        cv2.waitKey(1)


    def main2(self):
        # 라이다 -> 카메라 
    
        xyz_c = self.transform_lidar2cam(self.points_land)
        xyi = self.project_pts2img(xyz_c)
        xyi = self.crop_pts(xyi)
        #print(np.shape(xyi[:,0]))

        img = self.line(self.img)
        #img = self.cam2BEV(self.img)
        img = self.extract_blue(img)
        #img = self.BEV2cam(img)
        
        #cv2.imshow('BEV', img)
        # img = self.draw_pts_img(img, xyi[:,0], xyi[:,1], xyi[:,2])
        # img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        # self.image_pub.publish(img_msg)
        #img = self.cam2BEV(img)
        # cv2.imshow('BEV', img)
        lateral_min = self.filter_and_calculate_lateral_minimum_distance_and_show(img, xyi)
        print(lateral_min)
        
        cv2.waitKey(1)

if __name__=="__main__":
    rospy.init_node("lidar_transform")

    transformer = LIDAR2CAMTransform(parameters_cam, parameters_lidar)
    rospy.wait_for_message('/voxel', PointCloud2)
    rospy.wait_for_message('/land', PointCloud2)
    rospy.wait_for_message('/image_jpeg/compressed', CompressedImage)
    rospy.loginfo('start')
    

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        transformer.main2()
        rate.sleep()
    
    