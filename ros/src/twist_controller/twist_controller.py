from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
JERK_MAX =  10.0
JERK_MIN = -10.0


class Controller(object):
    def __init__(self,
                 wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                 decel_limit, accel_limit, r, m, p, d):

        # constants
        self.r = r
        self.m = m
        self.p = p
        self.d = d

        # default yaw controller from Udacity
        self.steering_ctr = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # tuned PID controller for the throttle
        self.throttle_ctr = PID(0.292, 0.2552, 0.02631, decel_limit, accel_limit)
        self.velocity_ctr = PID(3.34,  0.195,  0.0,     decel_limit, accel_limit)
        self.acceleration_ctr = PID(0.2, 0.0, 0.0,     -5.0, 1.0)

        # NOT USED YET: SHOULD WE REMOVE THEM ENTIRELY?
        self.acceleration_fit = LowPassFilter(0.25, 1.0/50.0)
        self.velocity_filt    = LowPassFilter(0.25, 1.0/50.0)
        self.steering_filt    = LowPassFilter(0.25, 1.0/50.0)

        # some values
        self.a_des_old = None

    def reset(self):
        self.throttle_ctr.reset()
        self.velocity_filt.reset()
        self.steering_filt.reset()

    def control(self, linear_velocity, angular_velocity, current_velocity, current_accel, dt):
        if self.a_des_old is None:
            self.a_des_old = 0

        # steering controller followed by filter
        steering = self.steering_ctr.get_steering(linear_velocity, angular_velocity, current_velocity)
        steering = self.steering_filt.filt(steering)

        # target velocity filter followed by pid-controller
        # linear_velocity = self.velocity_filt.filt(linear_velocity)
        # throttle = self.throttle_ctr.step(linear_velocity - current_velocity, dt)

        # cascaded velocity and acceleration control with feedforward
        a_des = self.velocity_ctr.step(linear_velocity - current_velocity, dt)

        # when stopping
        if linear_velocity == 0.0 and 1e-4 < current_velocity < 1.0:
            a_des = -2

        # rate limiter
        a_delta = a_des - self.a_des_old
        a_delta = max(min(JERK_MAX, a_delta), JERK_MIN)
        a_des = self.a_des_old + a_delta
        # saturation
        a_des = max(min(self.velocity_ctr.max, a_des), self.velocity_ctr.min)
        self.a_des_old = a_des
        # print("a_des: {}".format(a_des))

        # feed forward control
        throttle_FF = self.r/self.p*(self.m*a_des + self.d*current_velocity)
        # feed back control
        throttle_FB = self.acceleration_ctr.step(a_des - current_accel, dt)
        throttle = 1*throttle_FF + 1*throttle_FB

        # Return throttle, brake, steer
        return throttle, steering
