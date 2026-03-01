🧭 UKF + LiDAR + DNN + NMPC 기반 TurtleBot3 자율주행 시스템
🧭 UKF + LiDAR + DNN + NMPC 기반 TurtleBot3 자율주행 시스템

Graduation Project — ROS1 Autonomous Driving System

📌 프로젝트 개요

본 프로젝트는 TurtleBot3(Waffle Pi) 기반으로
UKF 상태추정 + LiDAR 인지 + DNN 위험도 예측 + NMPC 최적 제어를 결합하여
안정적이며 지능적인 자율주행 알고리즘을 구현하는 것을 목표로 한다.

비선형 차량 모델 기반 UKF 상태추정

LiDAR 기반 거리 측정 보정

DNN 위험도 예측 모델 적용

NMPC 제어기와 결합하여 경로 추종 + 안정 주행

ROS1 환경에서 모듈 간 실시간 통합

🧱 시스템 아키텍처
[Sensors]
┌─────────────────────────┐
│ LiDAR | IMU | Odom │
└─────────────────────────┘
│
▼
[UKF State Estimator]
(x, y, yaw, velocity)
│
▼
[DNN Risk Prediction]
(Risk Score 0~1)
│
▼
[NMPC Controller]
(Optimal v, ω generation)
│
▼
[TurtleBot3 Motion]
🔍 주요 기능
✔ UKF (Unscented Kalman Filter)

IMU + Odom + LiDAR 기반 센서 융합

LiDAR 스파이크 제거 및 안정된 거리 추정

실험 결과에서 raw 대비 변동성이 크게 감소

✔ LiDAR 기반 인지

/scan 데이터 노이즈 제거

위험도 Feature 생성

근접 장애물 판단 성능 향상

✔ DNN 위험도 모델

최근 N초간의 상태 히스토리 입력

위험한 상황을 0~1 사이 score로 예측

NMPC 비용항에 가중치로 반영

✔ NMPC 제어기

경로 오차 최소화

속도/조향 변화 최소화

위험도 기반 제어 강도 자동 조정

📊 실험 결과
1) Raw LiDAR vs UKF LiDAR



결과 해석

Raw LiDAR는 0~1.2m 범위에서 큰 스파이크 발생

UKF 필터 후 안정적인 곡선 형태 유지

장애물 거리 측정 신뢰도 향상

2) NMPC 제어 안정성 비교



결과 해석

raw 명령(/cmd_vel_raw)은 급격한 진동

UKF 기반 NMPC는 기울기 변화가 부드러움

감속/가속 구간에서도 제어 입력이 안정적

3) DNN + NMPC 연산 성능 개선



결과 해석

DNN+NMPC 조합은 평균 연산시간 약 23% 단축

NMPC 단독 대비 진동 적고 제어 일관성 증가

실시간 제어 가능성 확인

🗂 프로젝트 디렉터리 구조
📦 turtlebot-autonomous-driving/
├── ukf/
│ ├── ukf_node.py
│ ├── motion_model.py
│ ├── measurement_model.py
├── lidar/
│ ├── lidar_preprocess.cpp
│ └── obstacle_detector.cpp
├── dnn/
│ ├── risk_model.py
│ ├── train_risk_model.ipynb
├── nmpc/
│ ├── nmpc_solver.py
│ └── cost_function.py
├── launch/
│ ├── ukf.launch
│ ├── nmpc.launch
│ └── full_system.launch
└── README.md
🔧 사용 기술 스택

ROS1 (melodic/noetic)

Python + C++

UKF (Unscented Kalman Filter)

Deep Neural Network (PyTorch/Keras)

NMPC (Nonlinear MPC)

TurtleBot3 Waffle Pi

👤 본인 기여도

이 프로젝트에서 가장 핵심이 되는 인지·추정·제어 파트를 전부 직접 설계 및 구현

UKF 상태 추정 전체 모델 구축

LiDAR 전처리 및 위험 Feature 설계

DNN Risk Model 학습

NMPC 비용항/제약조건 설계

ROS1 기반 전체 통합 및 launch 시스템 제작

주행 실험 및 결과 분석

🔮 향후 발전 방향

UKF → Factor-Graph 기반 Back-End 확장

DNN → Transformer 기반 위험도 예측

NMPC → 강화학습 기반 Adaptive MPC 적용

TurtleBot3 → 실차 규모로 확장
