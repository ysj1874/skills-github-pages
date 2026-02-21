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
    """각 각도(beam)마다 1차원 UKF 운용: r_k = r_{k-1} + w, z_k = r_k + v"""
    def __init__(self, q=0.02, r=0.05, init_range=1.0):
        self.n = 1
        self.filters = []
        self.q = q
        self.r = r
        self.init_range = init_range

        def fx(x, dt):  # 상태전이
            return x

        def hx(x):      # 관측
            return x

        self.fx = fx
        self.hx = hx

    def _make_filter(self, init_value=None):
        sigmas = MerweScaledSigmaPoints(n=self.n, alpha=0.3, beta=2.0, kappa=0.0)
        f = UKF(dim_x=self.n, dim_z=self.n, fx=self.fx, hx=self.hx, dt=0.05, points=sigmas)
        f.x = np.array([self.init_range if init_value is None else init_value], dtype=float)
        f.P *= 0.3
        f.Q = np.array([[self.q]], dtype=float)
        f.R = np.array([[self.r]], dtype=float)
        return f

    def ensure_size(self, n_beams):
        """필터 뱅크 개수를 스캔 길이에 맞춤(증가/감소 둘 다 처리)"""
        cur = len(self.filters)
        if cur < n_beams:
            last = float(self.filters[-1].x[0]) if self.filters else self.init_range
            for _ in range(n_beams - cur):
                self.filters.append(self._make_filter(last))
        elif cur > n_beams:
            # 남는 필터는 버림
            self.filters = self.filters[:n_beams]

    def update_scan(self, ranges):
        """ranges(list/np.array) -> UKF로 필터링된 ranges 반환"""
        out = np.array(ranges, dtype=float, copy=True)
        n = out.shape[0]
        self.ensure_size(n)

        # 실제 길이만큼만 순회 (index out 방지)
        for i, z in enumerate(out):
            f = self.filters[i]
            if np.isfinite(z):
                f.predict()
                f.update([z])
                out[i] = float(f.x[0])
            else:
                f.predict()
                out[i] = float(f.x[0])

        # clip(모두 NaN인 경우 대비)
        finite = np.isfinite(out)
        if np.any(finite):
            out = np.clip(out, 0.0, float(np.nanmax(out[finite])))
        else:
            out[:] = 0.0
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

        self.filter_bank = PerBeamUKF(q=self.q, r=self.r, init_range=self.init_range)

        rospy.Subscriber(self.in_topic, LaserScan, self.cb_scan, queue_size=1)
        rospy.loginfo("[ukf_scan_filter] subscribe: %s -> publish: %s", self.in_topic, self.out_topic)

    def cb_scan(self, msg: LaserScan):
        # 필터링 (내부에서 길이 자동 동기화)
        filtered = self.filter_bank.update_scan(msg.ranges)

        # LaserScan 재구성
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

        # Diagnostics 발행
        raw_np = np.asarray(msg.ranges, dtype=float)
        flt_np = np.asarray(filtered, dtype=float)
        finite = np.isfinite(raw_np)
        invalid_ratio = 1.0 - (float(np.sum(finite)) / float(raw_np.size) if raw_np.size else 1.0)
        mean_raw = float(np.nanmean(raw_np)) if np.any(finite) else float('nan')
        mean_flt = float(np.nanmean(flt_np)) if flt_np.size else float('nan')

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
            KeyValue(key="r(meas_noise)", value=f"{self.r}"),
            KeyValue(key="beams", value=str(len(filtered))),
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
