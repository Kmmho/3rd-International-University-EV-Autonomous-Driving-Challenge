#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
from morai_msgs.msg import CtrlCmd, Lamps, GPSMessage
from morai_msgs.srv import MoraiEventCmdSrv
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int16,Float32MultiArray
from Kalman_v import KalmanPos2Vel2D
import time
import rospy
import os
import sys
import numpy as np
from pyproj import Proj
from tf.transformations import euler_from_quaternion
from Kalman_prev1 import KalmanPos2Vel1
from Kalman_prev import KalmanPos2Vel
from stanley_final import Stanley

class EgoState():
    velocity = 0
    x = 0
    y = 0
    heading = 0

class MoraiDriver():
    Course_1 = 1
    drive = 0
    GEAR_P = 1
    GEAR_R = 2
    GEAR_N = 3
    GEAR_D = 4
    def __init__(self):
        rospy.init_node('dku_driver')
        
        #----------------- subscribers -----------------#
        rospy.Subscriber('/gps',GPSMessage,self.cb_gps,queue_size=1)
        rospy.Subscriber('/imu',Imu,self.cb_imu,queue_size=1)
        rospy.Subscriber('car_info',Float32MultiArray,self.cb_car)
        rospy.Subscriber('/obs_dist',Float32MultiArray,self.cb_obs_dist,queue_size=1)
        rospy.Subscriber('yolo_data',Int16,self.cb_yolo,queue_size=1)
        rospy.Subscriber('/person_info',Float32MultiArray,self.cb_person)
        #----------------- publishers ------------------#
        self.pub_ctrl_cmd=rospy.Publisher('ctrl_cmd',CtrlCmd,queue_size=1)

        #----------------- services --------------------#
        self.srv_event_cmd = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)   
        
        self.event_cmd_srv_type = MoraiEventCmdSrv()
        self.event_cmd_srv_type.ctrl_mode = 0
        self.event_cmd_srv_type.gear = -1
        self.event_cmd_srv_type.set_pause = False
        self.event_cmd_srv_type.lamps = Lamps()
        self.event_cmd_srv_type.option = 0
        
        self.filter = KalmanPos2Vel2D(P0 = np.diag([1,1,1,1]), x0 = np.array([np.transpose([0,0,0,0])]))
        self.filter_p = KalmanPos2Vel(P0 = np.array([[1,0],[0,1]]), x0=np.array([[0],[0]]))
        self.filter_p1 = KalmanPos2Vel1(P0 = np.diag([1,1,1,1]), x0 = np.array([np.transpose([0,0,0,0])]))
        self.vel_x = 0
        self.vel_y = 0
        self.ctrl_cmd_type = CtrlCmd()
        self.ctrl_cmd_type.longlCmdType = 2
        
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.ref_tracker = Stanley()
        self.ego = EgoState()
        self.VRE = 0
        self.x = 0
        self.y = 0
        self.drive = 1
        self.distance_data = []
        self.cls = -1
        self.obs_dists = 50
        self.obs_dist = 50
        self.person_x = []
        self.person_y = []
        self.Collision = False




    def main(self):
        loop_hz = 40
        rate = rospy.Rate(loop_hz)
        
        rospy.loginfo('dku_driver: waiting for gps topics')
        rospy.wait_for_message('/gps',GPSMessage)
        rospy.loginfo('dku_driver: begin')
        
        self.cur_Course = self.Course_1
        self.set_course_path()
        self.set_gear(self.GEAR_D)
        
        ref_velocity = 0
        steering = 0
        idx = 0
        
        while not rospy.is_shutdown():

            ### Local Path planning 발동 조건 추가 ###
            ##  1. 정적 장애물일 때
            ##  2. 장애물과 일정 거리 이상으로 근접하거나 차선 미준수 가능 구역 들어왔을 때 (차선 미준수 가능 구역 들어오면 하는것도 괜찮은 듯 -> 조건 단순화)
            ##  3. 혹은, 설정해둔 global path 사이에 장애물의 좌표가 겹친다면 planning 하는 것도 괜찮은 방법 -> Local planning에서 이를 기준으로 모션계획을 하고 있기 떄문이다.

            # if self.Collision:
            #     ## 조건이 참일 때, local Path planning
            #     self.ref_tracker.set_local_path(self.ego)
            # else:
            #     ## 차량이 완전한 충돌회피를 완료했을 땐, 다시 Global path tracking & 장애물 시나리오 초기화
            #     self.ref_tracker.retracking()

            # self.ref_tracker.set_local_path(self.ego)


            self.ref_tracker.set_state(self.ego)
            ref_velocity, steering, idx = self.ref_tracker.main(self.drive, self.obs_dist, self.VRE, self.x, self.y, self.cls,self.person_x,self.person_y)
            self.set_cmd(steering,ref_velocity)
            rate.sleep()
            


    def set_course_path(self):
        self.ref_tracker.set_ref_path('bonsun_smoothing.csv')
        return
    
    def set_cmd(self,steer,vel):
        self.ctrl_cmd_type.velocity=vel
        self.ctrl_cmd_type.steering=steer
        self.pub_ctrl_cmd.publish(self.ctrl_cmd_type)



    def set_gear(self, gear):
        while abs(self.ego.velocity > 1):
            time.sleep(0.1)  
        self.event_cmd_srv_type.option = 2
        self.event_cmd_srv_type.gear = gear
        _ = self.srv_event_cmd(self.event_cmd_srv_type)

        
            #----------------- callback 함수 --------------------#
    def cb_gps(self,data):
        if data.longitude != 0:
            # self.ego.x, self.ego.y = self.proj_UTM(data.longitude, data.latitude)
            x, y = self.proj_UTM(data.longitude, data.latitude)
            self.ego.x = x  - data.eastOffset
            self.ego.y = y  - data.northOffset

        else:
            self.ego.x, self.ego.y, self.ego.velocity = 0, 0, 0
            

    def get_front_velocity(self):
        
        self.vel_x,self.vel_y = self.filter_p1.main(self.x,self.y)
        if self.vel_x > 0:
            self.VRE = np.sqrt(self.vel_x**2 + self.vel_y**2)
        else:
            self.VRE = -np.sqrt(self.vel_x**2 + self.vel_y**2)
        
    def cb_imu(self, data):
        euler = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.ego.heading = euler[2]
        self.ego.heading = 2*np.pi + self.ego.heading if -np.pi < self.ego.heading < -np.pi/2  else self.ego.heading
        #rospy.loginfo('Heading : %.3f', self.ego.heading)
        self.ego.ax      = data.linear_acceleration.x
        
    def cb_obs_dist(self,data):
        if data.data == None:
            obs_dists = 100
        else:
            obs_dists = data.data
        self.obs_dist = min(obs_dists)
        self.distance_data.append(self.obs_dist)
    
    def cb_car(self,data):
        if abs(data.data[0]-self.x) > 15 and (self.x != 0):
            self.filter_p.firstrun = True
        self.x = data.data[0]
        self.y = data.data[1]
        self.get_front_velocity()
    

    def cb_yolo(self,data):
        self.cls = data.data
        
    def cb_person(self,data):
         # data.data의 길이 확인
        if data.data and len(data.data) >= 2:
            # data.data가 유효하고, 최소 2개의 요소가 있을 경우
            self.person_x = float(data.data[0])
            self.person_y = float(data.data[1])
        else:
            # 그 외의 경우, 기본값 설정
            self.person_x = 100
            self.person_y = 100

        # 현재 person_x와 person_y 값 출력
        print(self.person_x, self.person_y)
if __name__=='__main__':
    morai_driver = MoraiDriver()
    morai_driver.main()