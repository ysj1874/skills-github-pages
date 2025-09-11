#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# filterpy가 없다면: pip3 install --user filterpy
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class PerBeamUKF:
    """각 각도(beam)마다 1차원 UKF를 운용: r_k = r_{k-1} + w, z_k = r_k + v"""
    def __init__(self, n_beams, q=0.02, r=0.05, init_range=1.0):
        self.n = 1
        self.n_beams = n_beams
        self.filters = []
        self.q = q
        self.r = r

        def fx(x, dt):  # 상태전이: 항등 + 잡음
            return x

        def hx(x):      # 관측: 항등
            return x

        for _ in range(n_beams):
            sigmas = MerweScaledSigmaPoints(n=self.n, alpha=0.3, beta=2.0, kappa=0.0)
            f = UKF(dim_x=self.n, dim_z=self.n, fx=fx, hx=hx, dt=0.05, points=sigmas)
            f.x = np.array([init_range], dtype=float)
            f.P *= 0.3
            f.Q = np.array([[self.q]], dtype=float)
            f.R = np.array([[self.r]], dtype=float)
            self.filters.append(f)

    def update_scan(self, ranges):
        """ranges(list/np.array) -> UKF로 필터링된 ranges 반환"""
        out = np.array(ranges, dtype=float, copy=True)

        for i in range(self.n_beams):
            z = out[i]
            # Inf/NaN 은 최신 추정으로 대체(관측 업데이트 없이 예측만)
            if np.isfinite(z):
                self.filters[i].predict()
                self.filters[i].update([z])
                out[i] = float(self.filters[i].x[0])
            else:
                self.filters[i].predict()
                out[i] = float(self.filters[i].x[0])

        # 음수/말도 안 되는 값 clip
        out = np.clip(out, 0.0, np.nanmax(out[np.isfinite(out)]) if np.any(np.isfinite(out)) else 10.0)
        return out.tolist()


class UKFScanFilterNode:
    def __init__(self):
        rospy.init_node("ukf_scan_filter", anonymous=False)

        # 파라미터
        self.in_topic  = rospy.get_param("~in_scan_topic", "/scan")
        self.out_topic = rospy.get_param("~out_scan_topic", "/scan_ukf")
        self.q = float(rospy.get_param("~process_noise_q", 0.02))
        self.r = float(rospy.get_param("~meas_noise_r", 0.05))
        self.init_range = float(rospy.get_param("~init_range", 1.0))

        self.pub_scan = rospy.Publisher(self.out_topic, LaserScan, queue_size=1)
        self.pub_diag = rospy.Publisher("/turtlebot3_diagnostics", DiagnosticArray, queue_size=1)

        self.filter_bank = None
        self.last_stamp = None

        rospy.Subscriber(self.in_topic, LaserScan, self.cb_scan, queue_size=1)
        rospy.loginfo("[ukf_scan_filter] subscribe: %s -> publish: %s", self.in_topic, self.out_topic)

    def cb_scan(self, msg: LaserScan):
        n_beams = len(msg.ranges)

        if self.filter_bank is None:
            self.filter_bank = PerBeamUKF(
                n_beams=n_beams, q=self.q, r=self.r, init_range=self.init_range
            )
            rospy.loginfo("[ukf_scan_filter] UKF bank initialized: %d beams", n_beams)

        # 필터링
        filtered = self.filter_bank.update_scan(msg.ranges)

        # LaserScan 메시지 재구성
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = filtered
        out.intensities = msg.intensities

        self.pub_scan.publish(out)

        # 진단 정보 발행(스캔 품질 지표 몇 가지)
        finite = np.isfinite(np.array(msg.ranges))
        invalid_ratio = 1.0 - float(np.sum(finite)) / float(len(msg.ranges))
        mean_raw = float(np.nanmean(np.array(msg.ranges, dtype=float))) if np.any(finite) else float('nan')
        mean_flt = float(np.nanmean(np.array(filtered, dtype=float)))

        da = DiagnosticArray()
        da.header.stamp = rospy.Time.now()
        st = DiagnosticStatus()
        st.name = "ukf_scan_filter"
        st.hardware_id = "turtlebot3_lidar"
        st.level = DiagnosticStatus.OK if invalid_ratio < 0.1 else DiagnosticStatus.WARN
        st.message = "scan filtered"
        st.values = [
            KeyValue(key="invalid_ratio", value=f"{invalid_ratio:.3f}"),
            KeyValue(key="mean_raw", value=f"{mean_raw:.3f}"),
            KeyValue(key="mean_filtered", value=f"{mean_flt:.3f}"),
            KeyValue(key="q(process_noise)", value=f"{self.q}"),
            KeyValue(key="r(meas_noise)", value=f"{self.r}")
        ]
        da.status.append(st)
        self.pub_diag.publish(da)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = UKFScanFilterNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
