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
        self.a_param = float(rospy.get_param('~a_param', 0.0))  # Differential Drive Model

        # --- Cost Weights (Q_N은 터미널 비용 가중치) ---
        self.Q = ca.diag([0.75, 0.75, 0.75])  # 상태 도달 중요도 [x, y, theta]
        self.R = ca.diag([0.5, 0.1])  # 제어 입력 U 추종 중요도
        self.Q_N = 1
        * self.Q # 터미널 비용에 큰 가중치 부여 (안정성 증대)
        
        # --- State holders ---
        self.pose = None        
        self.last_amcl_time = 0.0
        # self.AMCL_TIMEOUT = 1.0 # 🚨 삭제: AMCL Stuck 감지 변수 제거
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

        rospy.loginfo("Pure NMPC Base Node initialized (Stuck Logic Removed).")

    def _quaternion_to_yaw(self, q_msg):
        return tft.euler_from_quaternion([q_msg.x, q_msg.y, q_msg.z, q_msg.w])[2]

    def _normalize_angle(self, angle):
        """각도 오차 처리를 위한 -pi ~ pi 정규화"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def cb_pose(self, msg):
        """AMCL 포즈 수신: 포즈를 갱신하고, 수신 시간을 갱신."""
        self.pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._quaternion_to_yaw(msg.pose.pose.orientation)
        ])
        self.last_amcl_time = time.time()
        rospy.logdebug_throttle(1.0, "AMCL Pose received: {0:.2f}, {1:.2f}".format(self.pose[0], self.pose[1]))

    def cb_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _forward_sim(self, s, u):
        """Warm start에 사용될 Kinematic 순방향 시뮬레이션 함수"""
        x, y, th = s
        v, w = u
        
        # Kinematic Differential Drive Model (a_param=0 가정)
        x_next = x + v * np.cos(th) * self.dt
        y_next = y + v * np.sin(th) * self.dt
        th_next = th + w * self.dt
        
        # Warm start의 th_next에도 정규화 적용
        return np.array([x_next, y_next, self._normalize_angle(th_next)])


    def cb_cmd_in(self, msg: Twist):
        # --- 1. 초기화 및 고착 확인 ---
        if self.pose is None:
            rospy.logwarn_once("Pose (AMCL) not available yet. Publishing zero command.")
            self.cmd_pub.publish(Twist())
            return
        
        # 🚨 AMCL Stuck 로직 제거: pose 업데이트 지연 시 마지막 포즈 사용
        # if time_since_last_pose > self.AMCL_TIMEOUT: ... (제거됨)

        S0 = self.pose.copy() # 현재 AMCL 포즈 사용
        
        # 목표 G 설정 (경로의 마지막 지점)
        if self.path:
            gx, gy = self.path[-1]
            gth = self._normalize_angle(math.atan2(gy - S0[1], gx - S0[0]))
        else:
            gx, gy, gth = S0
        G = np.array([gx, gy, gth])
        
        # ------------------------------------------------------------------
        # 2. NMPC 최적화 실행
        # ------------------------------------------------------------------
        v_ref = msg.linear.x
        w_ref = msg.angular.z
        U_ref = np.array([v_ref, w_ref]).reshape(-1, 1)

        lbg = np.zeros(self.ng)  
        ubg = np.zeros(self.ng)

        # warm start 
        U_init = np.hstack([self.last_U[:, 1:], self.last_U[:, -1:]])
        S_init = np.zeros((3, self.N+1))
        S_init[:, 0] = S0
        for k in range(self.N):
            S_init[:, k+1] = self._forward_sim(S_init[:, k], U_init[:, k])

        w0 = np.concatenate([U_init.flatten(), S_init.flatten()])

        # Solve NMPC
        try:
            start_time = time.time()
            sol = self.solver(x0=w0,
                              lbx=self.lbx, ubx=self.ubx,
                              lbg=lbg, ubg=ubg,
                              p=np.concatenate([S0, G, U_ref.flatten()]))
            end_time = time.time()
            solve_time_ms = (end_time - start_time) * 1000.0 # 밀리초(ms)로 변환
            
            status = self.solver.stats()['return_status']
            
            if status not in ['success', 'acceptable', 'Solve_Succeeded']:
                rospy.logerr_throttle(1.0, "NMPC solver failed ({0}). Emergency stop.".format(status))
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
            
            # 🚨 연산 시간 로그 출력 (compare_runtime.py가 인식할 [NMPC] 프리픽스 사용)
            rospy.loginfo_throttle(1.0, 
                "NMPC Solve Time: {0:.3f} ms | CMD: v={1:.3f}, w={2:.3f}".format(solve_time_ms, v_cmd, w_cmd)
            )
            
        except Exception as e:
            rospy.logerr("NMPC solver CRITICAL exception: %s", e)
            self.cmd_pub.publish(Twist())


    def _build_opt(self):
        # States/controls
        x = ca.SX.sym('x'); y = ca.SX.sym('y'); th = ca.SX.sym('th')
        v = ca.SX.sym('v'); w = ca.SX.sym('w')
        s = ca.vertcat(x, y, th)
        u = ca.vertcat(v, w)

        # Kinematic Differential Drive Model (Discrete-time)
        def dyn_casadi(s, u):
            f_xu = ca.vertcat(ca.cos(s[2]), ca.sin(s[2]), 0)
            f_uw = ca.vertcat(-self.a_param * ca.sin(s[2]), self.a_param * ca.cos(s[2]), 1)
            f_xu_u = f_xu * u[0]
            f_uw_u = f_uw * u[1]
            return s + (f_xu_u + f_uw_u) * self.dt
        
        U = ca.SX.sym('U', self.nu, self.N)
        S = ca.SX.sym('S', self.nx, self.N + 1)
        S0 = ca.SX.sym('S0', self.nx) # 현재 상태 파라미터
        G = ca.SX.sym('G', self.nx)   # 목표 상태 파라미터
        U_ref = ca.SX.sym('U_ref', self.nu, 1) # 참조 입력 파라미터

        # 제약 조건 리스트 (g)
        g = []
        
        # 1. 초기 상태 제약: S_0 = S0 (S[:, 0] - S0 = 0)
        g.append(S[:, 0] - S0) 

        # 비용 함수 (J)
        J = 0

        # --- 1. 예측 구간 비용 (Running Cost) ---
        for k in range(self.N):
            # 2. 동역학 제약: S_{k+1} - dyn(S_k, U_k) = 0
            g.append(S[:, k+1] - dyn_casadi(S[:, k], U[:, k]))
            
            # 상태 오차 비용
            state_error = S[:, k] - G
            state_error[2] = ca.fmod(state_error[2] + ca.pi, 2*ca.pi) - ca.pi # 각도 정규화
            J += state_error.T @ self.Q @ state_error
            
            # 제어 입력 추종 오차 비용
            control_tracking_loss = (U[:, k] - U_ref).T @ self.R @ (U[:, k] - U_ref)
            J += control_tracking_loss
            
        # --- 2. 터미널 비용 (Terminal Cost) ---
        # 예측 구간 끝 상태(S_N)에 대한 비용 추가 (안정성 강화)
        terminal_error = S[:, self.N] - G
        terminal_error[2] = ca.fmod(terminal_error[2] + ca.pi, 2*ca.pi) - ca.pi
        J += terminal_error.T @ self.Q_N @ terminal_error # Q_N 사용

        wvars = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(S, -1, 1))
        g_constraints = ca.vertcat(*g)

        # nlp dictionary
        nlp = {'x': wvars, 'f': J, 'g': g_constraints, 'p': ca.vertcat(S0, G, U_ref)}
        
        # IPOPT Solver
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        # wvars 경계 (변수 U, S에 대한 하한/상한 제약)
        lbU = np.tile([-self.v_max, -self.w_max], self.N)    
        ubU = np.tile([self.v_max, self.w_max], self.N)
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
