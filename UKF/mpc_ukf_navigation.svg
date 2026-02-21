import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class PerBeamUKF:
    def __init__(self, n_beams=0, q=0.02, r=0.05, init_range=1.0):
        self.q = q
        self.r = r
        self.filters = []   # 개수는 들어오는 스캔 길이에 맞춰 동기화
        self.init_range = init_range

    def _make_ukf(self):
        points = MerweScaledSigmaPoints(n=1, alpha=0.1, beta=2.0, kappa=0.0)
        ukf = UKF(dim_x=1, dim_z=1, dt=1.0,
                  fx=lambda x, dt: x,
                  hx=lambda x: x,
                  points=points)
        ukf.x = np.array([self.init_range], dtype=float)
        ukf.P *= 0.5
        ukf.Q *= float(self.q)
        ukf.R *= float(self.r)
        return ukf

    def _sync_filters(self, n):
        """들어온 스캔 길이 n에 맞춰 필터 개수를 동기화"""
        if len(self.filters) < n:
            for _ in range(n - len(self.filters)):
                self.filters.append(self._make_ukf())
        elif len(self.filters) > n:
            self.filters = self.filters[:n]

    def update_scan(self, ranges):
        """ranges 길이에 맞춰 안전하게 업데이트하고 같은 길이의 out 반환"""
        rng = np.asarray(ranges, dtype=float)
        n = len(rng)
        self._sync_filters(n)

        out = rng.copy()
        # zip으로 길이 자동 맞춰 안전하게 순회
        for i, (z, f) in enumerate(zip(rng, self.filters)):
            # 유효하지 않은 측정은 스킵 (NaN/Inf/비양수)
            if not np.isfinite(z) or z <= 0.0:
                continue
            f.predict()
            f.update(np.array([z], dtype=float))
            out[i] = float(f.x[0])

        return out.tolist()
