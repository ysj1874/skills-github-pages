🧭 TurtleBot3 Autonomous Driving System
UKF + LiDAR + DNN + NMPC 기반 ROS1 자율주행 프로젝트



📌 1. 프로젝트 개요
본 프로젝트는 TurtleBot3(Waffle Pi) 기반의
UKF 상태추정 → LiDAR 기반 인지 → DNN 위험도 예측 → NMPC 제어로 구성된
지능형 자율주행 시스템입니다.

ROS1 환경에서 실시간 주행이 가능하도록 모든 모듈이 통합되었습니다.

🚀 2. 전체 시스템 아키텍처



시스템 구성
Sensing: LiDAR / IMU / Odometry

State Estimation: UKF 기반 상태 추정

Risk Prediction: DNN 위험도 모델

Control: NMPC 기반 주행 제어

ROS1: Node 구성 및 토픽 통신

🧩 3. 주요 기능 요약
모듈 설명
UKF LiDAR + IMU + Odom 센서 융합, x/y/yaw/v 추정
LiDAR 인지 장애물 거리 분석 및 위험 Feature 생성
DNN 모델 Risk(0~1) 예측, 위험 상황 조기 탐지
NMPC 제어기 경로 추종 + 충돌 회피 최적 제어
ROS1 Integration /scan, /imu/data, /odom, /cmd_vel 기반 통신
🧮 4. UKF 설계
📘 상태벡터
x = [px, py, yaw, v]
📘 UKF 과정

Sigma Points 생성

비선형 운동모델 기반 예측(Prediction)

LiDAR/IMU를 통한 상태 갱신(Update)

공분산 안정화 처리 (Numerical Stability)

출력 토픽:

/ukf_state
🧠 5. DNN 기반 위험도 모델
모델 목적

최근 주행 이력(UKF + LiDAR)을 기반으로 위험도 0~1 예측

위험 시 NMPC 비용 가중 증가 → 감속/회피 강화

데이터 구성

주행 로그 기반 시계열 데이터셋 생성

위험/정상 상황 라벨링

Python 기반 전처리 스크립트 포함

⚙️ 6. NMPC 제어기 구조



목적 함수 구성
경로 오차 최소화

조향 변화 최소화

속도 변화 최소화

DNN 위험도 기반 penalty 동적 적용

제어 명령 출력
/cmd_vel

linear.x
angular.z
📂 7. 프로젝트 디렉토리 구조
📦 turtlebot-autonomous-driving/
├── ukf/
│ ├── ukf_node.py
│ ├── motion_model.py
│ ├── measurement_model.py
│ └── params.yaml
├── lidar/
│ ├── lidar_preprocess.cpp
│ └── obstacle_detector.cpp
├── dnn/
│ ├── train_risk_model.ipynb
│ ├── risk_model.py
│ └── dataset/
├── nmpc/
│ ├── nmpc_solver.py
│ └── cost_function.py
├── launch/
│ ├── ukf.launch
│ ├── nmpc.launch
│ └── full_system.launch
└── README.md
🧪 8. 실험 결과 (실 주행 기반)


✔ UKF 성능
Odometry 대비 yaw drift 45% 이상 감소

노이즈 환경에서도 안정적 추정

✔ DNN 성능

위험 상황 탐지 정확도 90%+

✔ NMPC 주행

장애물 회피 성공률 100%

경로 유지 오차 ±5cm 수준

👤 9. 내 기여도 (핵심)

본 프로젝트에서 가장 난도 높은 “인지 + 추정 + 제어” 전 구간 직접 구현

UKF 전체 설계 및 파라미터 튜닝

LiDAR 전처리 및 장애물 거리 기반 Feature 생성

DNN Risk Model 데이터셋 제작 및 학습

NMPC 목적 함수/제약 조건 구성

ROS1 노드 통합 및 Launch 시스템 구축

🔮 10. 향후 발전 방향

UKF → Factor-Graph 기반 Back-End 확장

NMPC → Learning-based MPC 추가

DNN → Transformer Encoder 기반 위험도 모델 고도화

TurtleBot → 실차 플랫폼으로 확장 가능
