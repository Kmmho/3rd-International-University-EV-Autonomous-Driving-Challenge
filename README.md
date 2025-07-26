## 제3회 국제 대학생 EV 자율주행 경진대회 Virtual 부문(MORAI)
---  
### 프로젝트 소개
---  
자율주행 Virtual 시뮬레이터 MORAI를 활용하여 주행 환경에서 장애물 회피, 신호 인지 및 경로 추종 알고리즘 개발.  
예선과 본선으로 나누어 진행한다.  
||MAP|미션|
|:---:|:---:|:--:|
|예선|원주 운전면허시험장|안전주행, 동적 장애물 회피|  
|본선|서울 상암동|경로 준수, 장애물 회피, ACC, 신호등 인지 및 제어, GPS 음영구간에서의 장애물 회피 및 주행|  


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

## 예선 Mission  
<table>
  <tr>
    <td style="vertical-align: top; text-align: left;">
      <img width="500" height="500" alt="Image" src="https://github.com/user-attachments/assets/4166fe41-4f86-4bd0-989a-2e0a831c0689" />
    </td>
    <td>
      [Driving]<br>
      1. GPS 센서를 이용하여 1~3번 경로의 좌표 및 경로를 구한다.<br>
      2. 종방향 제어(PID)를 이용하여 속도를 제어한다. -> 커브 구간에서는 속도를 줄여 안정성 ↑<br>
      3. 횡방향 제어는 Stanley 제어기를 이용하여 CTE를 계산하여 차량이 경로에서 벗어나지 않도록 한다.<br><br>
      [Obstacle Avoidance]<br>
      1. 차선에는 꼬깔콘이 존재하여 장애물로 인식<br>
      2. YOLO를 이용하여 EURO NCAP 오브젝트를 학습<br>
      3. LiDAR to Camera Calibration 과정을 통해 객체와의 거리 파악<br>
      4. 오브젝트를 회피하여 안정적인 주행
    </td>
  </tr>
</table>  

## 본선 Mission  
<table>
  <tr>
    <td style="vertical-align: top; text-align: left;">
      <p align="center">
      <img src="https://github.com/user-attachments/assets/2abb7c1a-c3c8-41d9-9242-101efda1be06">
      </p>
    </td>
    <td>
      [Adaptive Cruise Control]<br>
      1. LiDAR와 Camera를 이용하여 각 센서의 출력을 가중 평균<br>
      2. 칼만필터를 이용하여 시간 간격 동안의 거리 변화를 계산하고 상대속도 추정<br>
      3. 선행 차량과의 거리 및 상대 속도를 이용하여 PID 제어기를 통해 차간 거리 유지<br>
    </td>
  </tr>
  <tr>
    <td style="vertical-align: top; text-align: left;">
      <img width="400" height="400" alt="Image" src="https://github.com/user-attachments/assets/032678bd-9639-43ed-ba05-d6e245542992" />
  </tr>
</table>  

