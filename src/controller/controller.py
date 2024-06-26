
# CODE from the uuv_controller
# https://github.com/uuvsimulator/uuv_simulator/blob/master/uuv_control/uuv_control_cascaded_pids/src/PID/PIDRegulator.py
import numpy


class PIDRegulator:
    """A very basic 1D PID Regulator."""
    def __init__(self, p, i, d, sat):
        self.p = p
        self.i = i
        self.d = d
        self.sat = sat

        self.integral = 0
        self.prev_err = 0
        self.prev_t = -1.0

    def __str__(self):
        msg = 'PID controller:'
        msg += '\n\tp=%f' % self.p
        msg += '\n\ti=%f' % self.i
        msg += '\n\td=%f' % self.d
        msg += '\n\tsat=%f' % self.sat
        return msg
    
    def set_parameters(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d

    def regulate(self, x1, x2, t):
        err = x2 - x1
        derr_dt = 0.0
        dt = t - self.prev_t
        if self.prev_t > 0.0 and dt > 0.0:
            derr_dt = (err - self.prev_err)/dt
            self.integral += 0.5*(err + self.prev_err)*dt

        u = self.p*err + self.d*derr_dt + self.i*self.integral

        self.prev_err = err
        self.prev_t = t

        if (numpy.linalg.norm(u) > self.sat):
            # controller is in saturation: limit outpt, reset integral
            u = self.sat*u/numpy.linalg.norm(u)
            self.integral = 0.0

        return u