from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                 decel_limit, accel_limit):

        # default yaw controller from Udacity
        self.steering_ctr = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # tuned PID controller for the throttle
        self.throttle_ctr = PID(0.292, 0.2552, 0.02631, decel_limit, accel_limit)

        # NOT USED YET: SHOULD WE REMOVE THEM ENTIRELY?
        self.velocity_filt = LowPassFilter(0.25, 1.0/50.0)
        self.steering_filt = LowPassFilter(0.25, 1.0/50.0)

    def reset(self):
        self.throttle_ctr.reset()
        self.velocity_filt.reset()
        self.steering_filt.reset()

    def control(self, linear_velocity, angular_velocity, current_velocity, dt):
        # steering controller followed by filter
        steering = self.steering_ctr.get_steering(linear_velocity, angular_velocity, current_velocity)
        # steering = self.steering_filt.filt(steering)

        # target velocity filter followed by pid-controller
        # linear_velocity = self.velocity_filt.filt(linear_velocity)
        throttle = self.throttle_ctr.step(linear_velocity - current_velocity, dt)

        # Return throttle, brake, steer
        return throttle, steering
