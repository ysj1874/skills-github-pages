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
        self.a_param = float(rospy.get_param('~a_param', 0.1))

        # --- Cost Weights ---
        self.Q = ca.diag([5.0, 5.0, 0.05])
        self.R = ca.diag([2.0, 0.5])
        
        # --- State holders ---
        self.pose = None        # 최종 포즈 (AMCL만 사용)
        self.last_amcl_time = 0.0 # AMCL 마지막 갱신 시간
        self.AMCL_TIMEOUT = 1.0   # 1초 이상 갱신 없으면 고착으로 간주
        self.path = []
        self.last_U = np.zeros((2, self.N))
        self.last_cmd = Twist() # 마지막 정상 명령 저장
        
        # 🚨 [수정]: self.ng, self.nu, self.nx를 미리 초기화
        self.nu = 2
        self.nx = 3
        self.ng = self.nx * (self.N + 1) # 이 값은 _build_opt에서 다시 계산되지만, 오류 방지를 위해 미리 할당

        # --- Build NMPC optimizer ---
        self._build_opt() # 이 함수 내에서 self.ng, self.lbx, self.ubx가 최종적으로 설정됨

        # --- ROS I/O ---
        self.cmd_pub = rospy.Publisher(self.cmd_out_topic, Twist, queue_size=10)
        # 구독자는 반드시 _build_opt 후에 설정되어야 안전함. (현재 순서 유지)
        rospy.Subscriber(self.state_topic, PoseWithCovarianceStamped, self.cb_pose) 
        rospy.Subscriber(self.path_topic, Path, self.cb_path)
        rospy.Subscriber(self.cmd_in_topic, Twist, self.cb_cmd_in)

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
        
        # AMCL이 1초 이상 갱신되지 않았을 때 (고착 시 강제 우회 로직 유지)
        if time_since_last_pose > self.AMCL_TIMEOUT: 
            if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
                 rospy.logwarn_throttle(0.5, "AMCL STUCK! Forcing raw command to /cmd_vel to revive AMCL.")
                 self.cmd_pub.publish(msg) # 원본 명령을 최종 토픽으로 강제 발행
                 return # NMPC 계산을 건너뛰고 강제 우회
            else:
                 # move_base 명령도 0이면 NMPC도 0을 발행하며 대기
                 self.cmd_pub.publish(Twist())
                 return

        # --- 2. 목표 도달 및 정지 확인 (로직 삭제) ---
        # 🚨 [수정]: 목표 도달 확인 로직 삭제. NMPC가 계산한 값을 신뢰하도록 함.
        
        S0 = self.pose.copy() # AMCL 포즈 사용
        
        if self.path:
            gx, gy = self.path[-1]
            gth = math.atan2(gy - S0[1], gx - S0[0])
        else:
            gx, gy, gth = S0
        G = np.array([gx, gy, gth])
        
        # ------------------------------------------------------------------
        # 3. AMCL 정상 작동 & 목표 도달 여부와 관계없이 NMPC 최적화 실행
        # ------------------------------------------------------------------
        
        v_ref = msg.linear.x
        w_ref = msg.angular.z
            
        U_ref = np.array([v_ref, w_ref])

        lbg = np.zeros(self.ng) 
        ubg = np.zeros(self.ng)

        # warm start
        U_init = np.hstack([self.last_U[:, 1:], self.last_U[:, -1:]]) if self.last_U.size else np.zeros((2, self.N))
        S_init = np.zeros((3, self.N+1))
        S_init[:, 0] = S0
        w0 = np.concatenate([U_init.flatten(), S_init.flatten()])

        # Solve NMPC
        try:
            sol = self.solver(x0=w0,
                              lbx=self.lbx, ubx=self.ubx,
                              lbg=lbg, ubg=ubg,
                              p=np.concatenate([S0, G, U_ref]))
            
            # 솔버 성공 상태 확인
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
            
            # NMPC 계산 결과가 0에 가깝더라도 그대로 발행 (치환 금지)
            out = Twist()
            out.linear.x = v_cmd
            out.angular.z = w_cmd
            self.cmd_pub.publish(out)
            rospy.logdebug_throttle(1.0, f"NMPC CMD: v={v_cmd:.3f}, w={w_cmd:.3f} | Pose: x={S0[0]:.2f}, y={S0[1]:.2f}")
            
        except Exception as e:
            rospy.logerr("NMPC solver CRITICAL exception: %s", e)
            self.cmd_pub.publish(Twist())


    def _build_opt(self):
        # States/controls
        x = ca.SX.sym('x'); y = ca.SX.sym('y'); th = ca.SX.sym('th')
        v = ca.SX.sym('v'); w = ca.SX.sym('w')
        s = ca.vertcat(x, y, th)
        u = ca.vertcat(v, w)

        # --- 논문 (수식 1)에 따른 동역학 모델 ---
        def dyn(s, u):
            f_xu = ca.vertcat(ca.cos(s[2]), ca.sin(s[2]), 0)
            f_uw = ca.vertcat(-self.a_param * ca.sin(s[2]), self.a_param * ca.cos(s[2]), 1)
            f_xu_u = f_xu * u[0]
            f_uw_u = f_uw * u[1]
            return s + (f_xu_u + f_uw_u) * self.dt
        
        U = ca.SX.sym('U', 2, self.N)
        S = ca.SX.sym('S', 3, self.N+1)
        S0 = ca.SX.sym('S0', 3)
        G = ca.SX.sym('G', 3)
        U_ref = ca.SX.sym('U_ref', 2, 1)

        g = [S[:, 0] - S0]
        J = 0

        # --- 논문 (수식 3)에 따른 손실 함수 ---
        for k in range(self.N):
            g.append(S[:, k+1] - dyn(S[:, k], U[:, k]))
            state_loss = (S[:, k] - G).T @ self.Q @ (S[:, k] - G)
            control_loss = U[:, k].T @ self.R @ U[:, k]
            J += state_loss + control_loss
        
        wvars = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(S, -1, 1))
        g = ca.vertcat(*g)

        nlp = {'x': wvars, 'f': J, 'g': g, 'p': ca.vertcat(S0, G, ca.reshape(U_ref, -1, 1))}
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        # 🚨 [수정]: self.ng, self.nu, self.nx는 __init__에서 미리 정의되었으므로 여기서는 계산만 수행
        self.ng = self.nx*(self.N+1)

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
