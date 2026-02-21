#!/usr/bin/env python3
import rospy
import numpy as np
import casadi as ca
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
import tf.transformations as tft
import math
import time

class NMPCFilter:
    def __init__(self):
        # --- Parameters ---
        self.state_topic = rospy.get_param('~state_topic', 'amcl_pose')
        self.cmd_in_topic = rospy.get_param('~cmd_in_topic', 'cmd_vel_raw')
        self.cmd_out_topic = rospy.get_param('~cmd_out_topic', 'cmd_vel')

        self.path_topic = rospy.get_param('~path_topic', 'waypoints')
        self.N = int(rospy.get_param('~horizon', 10))
        self.dt = float(rospy.get_param('~dt', 0.05))
        self.v_max = float(rospy.get_param('~v_max', 0.22))
        self.w_max = float(rospy.get_param('~w_max', 2.84))
        # 🚨 [수정]: a_param을 0.0으로 초기 설정하여 일반적인 Differential Drive 모델을 따르도록 함.
        #           (회전 중심 오프셋을 사용하지 않음)
        self.a_param = float(rospy.get_param('~a_param', 0.0)) 

        # --- Cost Weights (튜닝 권장 값) ---
        # Q: [x, y, theta] 상태 도달 중요도 (theta 가중치 증가)
        self.Q = ca.diag([0.75, 0.75, 0.75]) 
        # R: 제어 입력 U 추종 중요도 (전체 값을 낮춰 U_ref를 더 잘 따르도록 함)
        self.R = ca.diag([0.5, 0.1]) 
        
        # --- State holders ---
        self.pose = None        
        self.last_amcl_time = 0.0
        self.AMCL_TIMEOUT = 1.0
        self.path = []
        self.last_U = np.zeros((2, self.N))
        
        self.nu = 2 # controls: v, w
        self.nx = 3 # states: x, y, theta
        self.ng = self.nx * (self.N + 1)

        # --- Build NMPC optimizer ---
        self._build_opt()

        # --- ROS I/O ---
        self.cmd_pub = rospy.Publisher(self.cmd_out_topic, Twist, queue_size=10)
        rospy.Subscriber(self.state_topic, PoseWithCovarianceStamped, self.cb_pose)    
        rospy.Subscriber(self.path_topic, Path, self.cb_path)
        rospy.Subscriber(self.cmd_in_topic, Twist, self.cb_cmd_in, queue_size=1)

        rospy.loginfo("NMPC Filter Node initialized (AMCL STUCK RECOVERY ENABLED).")

    def _quaternion_to_yaw(self, q_msg):
        return tft.euler_from_quaternion([q_msg.x, q_msg.y, q_msg.z, q_msg.w])[2]

    def cb_pose(self, msg):
        """AMCL 포즈 수신: 포즈를 갱신하고, 수신 시간을 갱신."""
        self.pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._quaternion_to_yaw(msg.pose.pose.orientation)
        ])
        self.last_amcl_time = time.time()
        rospy.logdebug_throttle(1.0, f"AMCL Pose received: {self.pose[0]:.2f}, {self.pose[1]:.2f}")

    def cb_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def cb_cmd_in(self, msg: Twist):
        
        # --- 1. 초기화 및 고착 확인 ---
        
        time_since_last_pose = time.time() - self.last_amcl_time
        
        if self.pose is None:
            rospy.logwarn_once("Pose (AMCL) not available yet. Publishing zero command.")
            self.cmd_pub.publish(Twist())
            return
        
        # AMCL 고착 시 강제 우회 로직
        if time_since_last_pose > self.AMCL_TIMEOUT: 
            if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
                 rospy.logwarn_throttle(0.5, "AMCL STUCK! Forcing raw command to /cmd_vel to revive AMCL.")
                 self.cmd_pub.publish(msg) # 원본 명령 강제 발행
                 return # NMPC 계산 건너뛰기
            else:
                 self.cmd_pub.publish(Twist())
                 return

        S0 = self.pose.copy() # 현재 AMCL 포즈 사용
        
        # 목표 G 설정 (경로의 마지막 지점)
        if self.path:
            gx, gy = self.path[-1]
            # 경로의 마지막 각도는 현재 위치에서 마지막 지점으로 향하는 각도로 설정
            gth = math.atan2(gy - S0[1], gx - S0[0]) 
        else:
            # 경로가 없으면 현재 위치를 목표로 설정 (정지)
            gx, gy, gth = S0 
        G = np.array([gx, gy, gth])
        
        # ------------------------------------------------------------------
        # 2. NMPC 최적화 실행
        # ------------------------------------------------------------------
        
        v_ref = msg.linear.x
        w_ref = msg.angular.z
        
        # 🚨 [수정]: U_ref를 파라미터로 전달하기 위해 2x1 형태로 reshape
        U_ref = np.array([v_ref, w_ref]).reshape(-1, 1)

        lbg = np.zeros(self.ng) 
        ubg = np.zeros(self.ng)

        # warm start (이전 결과를 한 칸 쉬프트)
        U_init = np.hstack([self.last_U[:, 1:], self.last_U[:, -1:]])
        S_init = np.zeros((3, self.N+1))
        S_init[:, 0] = S0
        # 🚨 [추가]: S_init의 나머지 상태도 forward simulation으로 대략 채워넣어 초기 해의 품질을 높임
        for k in range(self.N):
            S_init[:, k+1] = self._forward_sim(S_init[:, k], U_init[:, k])

        w0 = np.concatenate([U_init.flatten(), S_init.flatten()])

        # Solve NMPC
        try:
            sol = self.solver(x0=w0,
                              lbx=self.lbx, ubx=self.ubx,
                              lbg=lbg, ubg=ubg,
                              # 🚨 [수정]: U_ref를 1차원으로 펼쳐서 전달
                              p=np.concatenate([S0, G, U_ref.flatten()]))
            
            status = self.solver.stats()['return_status']
            
            if status not in ['success', 'acceptable', 'Solve_Succeeded']:
                 rospy.logerr_throttle(1.0, f"NMPC solver failed ({status}). Emergency stop.")
                 self.cmd_pub.publish(Twist()) 
                 return

            w_opt = np.array(sol['x']).flatten()
            U_opt = w_opt[:self.nu * self.N].reshape(self.nu, self.N) 

            v_cmd = float(U_opt[0, 0])
            w_cmd = float(U_opt[1, 0])
            self.last_U = U_opt
            
            out = Twist()
            out.linear.x = v_cmd
            out.angular.z = w_cmd
            self.cmd_pub.publish(out)
            rospy.logdebug_throttle(1.0, f"NMPC CMD: v={v_cmd:.3f}, w={w_cmd:.3f} | Pose: x={S0[0]:.2f}, y={S0[1]:.2f}")
            
        except Exception as e:
            rospy.logerr("NMPC solver CRITICAL exception: %s", e)
            self.cmd_pub.publish(Twist())

    # 🚨 [추가]: Warm start에 사용될 간단한 순방향 시뮬레이션 함수
    def _forward_sim(self, s, u):
        x, y, th = s
        v, w = u
        
        # Kinematic Differential Drive Model (a_param=0 가정)
        # s_dot = [v*cos(th), v*sin(th), w]
        x_next = x + v * np.cos(th) * self.dt
        y_next = y + v * np.sin(th) * self.dt
        th_next = th + w * self.dt
        return np.array([x_next, y_next, self._normalize_angle(th_next)])

    # 🚨 [추가]: 각도 오차 처리를 위한 -pi ~ pi 정규화
    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


    def _build_opt(self):
        # States/controls
        x = ca.SX.sym('x'); y = ca.SX.sym('y'); th = ca.SX.sym('th')
        v = ca.SX.sym('v'); w = ca.SX.sym('w')
        s = ca.vertcat(x, y, th)
        u = ca.vertcat(v, w)

        # --- 논문 (수식 1)에 따른 동역학 모델 ---
        def dyn_casadi(s, u):
            # a_param이 0.0이면 전통적인 Kinematic Differential Drive 모델이 됨:
            # s_dot = [u[0]*cos(s[2]), u[0]*sin(s[2]), u[1]]
            f_xu = ca.vertcat(ca.cos(s[2]), ca.sin(s[2]), 0)
            f_uw = ca.vertcat(-self.a_param * ca.sin(s[2]), self.a_param * ca.cos(s[2]), 1)
            f_xu_u = f_xu * u[0]
            f_uw_u = f_uw * u[1]
            return s + (f_xu_u + f_uw_u) * self.dt
        
        U = ca.SX.sym('U', 2, self.N)
        S = ca.SX.sym('S', 3, self.N+1)
        S0 = ca.SX.sym('S0', 3)
        G = ca.SX.sym('G', 3)
        # 🚨 [수정]: U_ref를 2x1 벡터로 정의. cb_cmd_in에서 flatten()으로 전달받을 것임.
        U_ref = ca.SX.sym('U_ref', 2, 1)

        g = [S[:, 0] - S0]
        J = 0

        # --- 논문 (수식 3)에 따른 손실 함수 및 U_ref 추종 항 추가 ---
        for k in range(self.N):
            g.append(S[:, k+1] - dyn_casadi(S[:, k], U[:, k]))
            
            # 1. 상태 오차 (Goal 도달)
            # 🚨 [수정]: 각도 오차를 -pi ~ pi로 정규화 (atan2 대신 atan2(sin/cos)를 이용)
            state_error = S[:, k] - G
            state_error[2] = ca.fmod(state_error[2] + ca.pi, 2*ca.pi) - ca.pi # 각도 정규화
            state_loss = state_error.T @ self.Q @ state_error
            
            # 2. 제어 입력 추종 오차 (U_ref 추종)
            # U_ref는 상수이므로 U_ref[:, 0] 대신 U_ref를 그대로 사용
            control_tracking_loss = (U[:, k] - U_ref).T @ self.R @ (U[:, k] - U_ref)
            
            # 3. 제어 입력 변화량 오차 (선택적: 부드러운 움직임)
            # control_rate_loss = 0.0
            
            J += state_loss + control_tracking_loss
            
        wvars = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(S, -1, 1))
        g = ca.vertcat(*g)

        # 🚨 [수정]: 파라미터 p 정의에 U_ref 추가
        nlp = {'x': wvars, 'f': J, 'g': g, 'p': ca.vertcat(S0, G, U_ref)}
        
        # IPOPT Solver
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        self.ng = self.nx*(self.N+1)

        # 제어 입력 U에 대한 하한/상한 제약 조건
        lbU = np.tile([-self.v_max, -self.w_max], self.N)    
        ubU = np.tile([self.v_max, self.w_max], self.N)
        # 상태 S에 대한 하한/상한 제약 조건 (없음)
        lbS = np.full(self.nx*(self.N+1), -np.inf)
        ubS = np.full(self.nx*(self.N+1),  np.inf)
        
        self.lbx = np.concatenate([lbU, lbS])
        self.ubx = np.concatenate([ubU, ubS])


def main():
    rospy.init_node("nmpc_filter_node")
    NMPCFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
