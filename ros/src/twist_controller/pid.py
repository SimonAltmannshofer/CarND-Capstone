
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = 0.0
        self.last_error = 0.0

    def reset(self):
        self.int_val = 0.0
        self.last_error = 0.0

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        val = self.kp * error + self.ki * integral + self.kd * derivative

        # clip output value to actor-bounds
        val = max(self.min, min(self.max, val))

        # proper saturation of integrator
        if self.ki > 0:
            # stop integration if clamping is active
            if val <= self.min or val >= self.max:
                self.int_val = self.int_val
            else:
                self.int_val = integral

        # update current error
        self.last_error = error

        return val
