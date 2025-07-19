### 제3회 국제 대학생 EV 자율주행 경진대회 Virtual 부문(MORAI)
---  
### 프로젝트 소개
---  
자율주행 Virtual 시뮬레이터 MORAI를 활용하여 주행 환경에서 장애물 회피, 신호 인지 및 경로 추종 알고리즘 개발.  
예선과 본선으로 나누어 진행한다.  
||MAP|미션|
|:---:|:---:|:--:|
|예선|원주 운전면허시험장|안전주행, 동적 장애물 회피|  
|본선|서울 상암동|보행자 회피, 경로 준수, 신호등 인지 및 제어, GPS 음영구간에서의 장애물 회피 및 주행|  


### 차량  
- **Model** : 2023_Hyundai_ioniq5
<img width="365" height="564" alt="Image" src="https://github.com/user-attachments/assets/46a4dd06-24d7-4d78-9f74-48573df8ee9d" />  

- **Steer Angle** :  
  ○ Maximum Steer Angle (deg) : 40  
  ○ Minimum Turning Radius (m) : 5.97  

- **Exterior Dimensions** :  
  ○ Length (m) : 4.635  
  ○ Width (m) : 1.892  
  ○ Height (m) : 2.434  
  ○ Wheelbase (m) : 3.000  
  ○ Front Overhang (m) : 0.845  
  ○ Rear Overhang (m) : 0.7

### 센서  
- **GPS** :  
  ○ Data Rate : 40Hz      
  ○ Network : UDP    
- **IMU** :  
  ○ Data Rate : 40Hz  
  ○ Network : ROS  
- **3D LiDAR** :  
  ○ Intensity type : intensity  
  ○ Model : VLP16    
  ○ rotation Rate : 10Hz  
  ○ Network : UDP  
- **Camera** :
  ○ Ground truth : None
  ○ 해상도 : 1280 * 720
  ○ frame : 20Hz
  ○ Network : ROS    
---  
