"""Scalar (1-D) Kalman filter — smooth a noisy signal without fixed-EMA lag.

Used to de-jitter the encoder pulse rate (conveyor mechanical vibration) on the
Pi before it drives interception timing. Pure math, no I/O -> unit-testable.
"""


class KalmanFilter1D:
    """Random-walk scalar Kalman filter.

    State = the estimated value; the process model is a random walk (the true
    value is ~constant between samples, with process-noise variance q). Each
    measurement z carries variance r. The gain adapts from the q/r ratio instead
    of a fixed EMA alpha, so it rejects jitter without lagging on genuine
    belt-speed changes. Steady-state gain ~ sqrt(q/r) for q << r.

    q : process-noise variance (how much the true value may drift per step)
    r : measurement-noise variance (sensor / vibration jitter)
    """

    def __init__(self, q, r, p0=1.0):
        self.q = float(q)
        self.r = float(r)
        self.x = 0.0          # state estimate
        self.p = float(p0)    # estimate-error variance
        self._seeded = False

    def reset(self, p0=1.0):
        """Forget the estimate; the next update() reseeds from its measurement."""
        self.x = 0.0
        self.p = float(p0)
        self._seeded = False

    def update(self, z):
        """Fuse measurement z and return the filtered estimate."""
        z = float(z)
        if not self._seeded:
            # Seed on the first real sample so the estimate doesn't crawl from 0.
            self.x = z
            self.p = self.r
            self._seeded = True
            return self.x
        # Predict (random walk: state unchanged, variance grows by q)
        self.p += self.q
        # Update (fuse the measurement)
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1.0 - k)
        return self.x

    @property
    def value(self):
        return self.x
