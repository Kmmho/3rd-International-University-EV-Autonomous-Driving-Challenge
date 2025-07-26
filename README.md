## 제3회 국제 대학생 EV 자율주행 경진대회 Virtual 부문(MORAI)

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

## 1️⃣ **예선 Mission**  
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

## 2️⃣ **본선 Mission**    
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
      <img width="300" height="300" alt="Image" src="https://github.com/user-attachments/assets/032678bd-9639-43ed-ba05-d6e245542992" />
    </td>
    <td>
      [Traffic Signal Detection]<br>
      1. 카메라 센서 및 YOLO를 이용하여 신호등 학습<br>
      2. 빨간불은 1, 노란불은 2, 초록불은 3으로 지정하여 Topic 전달<br>
      3. 정지선 앞이 아닌 곳에서 신호등을 감지하면 멀리서도 정지하는 문제 발생<br>
      4. 경로가 담긴 특정 idx 조건을 이용하여 정지선 가까이에 차량이 정지하게 한다.<br>
    </td>
  </tr>
  <tr>
    <td style="vertical-align: top; text-align: left;">
      <img width="400" height="400" alt="Image" src="https://github.com/user-attachments/assets/eb27d2e7-50ab-4021-9749-1ce7a1daf324" /><br>
    <p align="center">
    <img src="https://github.com/user-attachments/assets/22458676-e809-4912-a61c-02904a8d5f99">
    </p>
    </td>
    <td>
      [GPS Blackout]<br>
      1. GPS Blackout 영역에서는 GPS가 작동하지 않기 때문에 현재 ego 차량의 좌표를 알 수 없음.<br>
      2. LiDAR 센서를 이용하여 맵을 구성하여 맵 기반 위치 추정<br>
      3. IMU 센서의 Odometry 정보를 이용하여 GPS Blackout가 시작되는 좌표에서의 차량의 위치를 파악.<br>
      4. CSV 파일의 idx 경로에서 가장 가까운 점을 찾고 mapping하여 해당 idx 획득<br>
    </td>
  </tr>
  <tr>
    <td style="vertical-align: top; text-align: left;">
      <img width="400" height="400" alt="Image" src="https://github.com/user-attachments/assets/d72d1a79-3327-4f52-b8a6-6efdaae96d91" /><br>
      <img width="300" height="300" alt="Image" src="https://github.com/user-attachments/assets/ef89ba58-5fe5-4ab6-9ae6-2fa22a6e9b7f" />
    </td>
    <td>
      [Obstacle Avoidance in GPS Blackout]<br>
      1. GPS Blackout 구역에서는 장애물 회피를 진행하고 다시 원래 경로를 추종하기 어려움<br>
      2. 장애물 회피를 진행하는 도중 중앙선 침범 문제 발생<br>
      3. 1차선에 장애물이 존재하면 중앙선 침범 방지를 위해 차량이 오른쪽으로 회피해야하는데 왼쪽으로 회피하여 침범하는 문제 발생<br>
      4. 3D LiDAR의 Pointcloud를 global path가 있는 UTM 좌표계로 변환<br>
      5. 3D LiDAR 및 Camera를 이용하여 중앙선을 검출 후 차선의 벡터와 pointcloud 벡터와의 내적 연산을 진행하여 θ를 획득<br>
      6. sin(θ) 연산을 통해 lateral_min_distance 값을 구하여 현재 차선을 판단<br>
      7. 현재 차선이 1차선이라면 중앙선을 넘어가지 않게 제한<br>
      <br>
      <img width="500" height="500" alt="Image" src="https://github.com/user-attachments/assets/1b8ceb4b-2d98-4045-8772-ca9a40d62018" />
    </td>
  </tr>
</table>  

### 3️⃣ **Test Scenario**  
<img width="400" height="400" alt="Image" src="https://github.com/user-attachments/assets/5d736fbf-d0ca-45ee-b723-37191ea65aa7" /><br>
-> 커브길, 사각지대, 급출현 지역 등 약 20가지의 장애물 Test Scenario를 json 파일로 생성하여 성능 검증 진행  

## ⭐Awards⭐  
<table>
  <tr>
    <td style="vertical-align: top; text-align: left;">
      <img width="250" height="250" alt="Image" src="https://github.com/user-attachments/assets/697e9880-299a-4102-94ae-b9be9262ab06" />
    </td>
    <td>
      <img width="400" height="400" alt="Image" src="https://github.com/user-attachments/assets/f9c894fa-6b50-4cb0-9e57-8bdc7aa74ad1" />
    </td>
  </tr>
