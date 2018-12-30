from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
JERK_MAX = 10.0
JERK_MIN = -10.0


class Controller(object):
    def __init__(self,
                 wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                 decel_limit, accel_limit, r, m, p_throttle, p_brake, d):

        # constants
        self.r = r  # wheel radius [m]
        self.m = m  # total vehicle mass [kg]
        self.p_throttle = p_throttle  # engine power factor [Nm/1]
        self.p_brake = p_brake  # brake power ratio [Nm/Nm]
        self.d = d  # velocity dependant resistance  [N/(m/s)]
        self.decel_limit = decel_limit

        # default yaw controller from Udacity
        self.steering_ctr = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # Controller A not used anymore: tuned PID controller for the throttle
        self.throttle_ctr = PID(0.292, 0.2552, 0.02631, decel_limit, accel_limit)

        # Controller B currently in use: cascaded PID controllers with feed-forward
        # 1st stage: maps velocity_error to desired_acceleration
        self.velocity_ctr = PID(3.0,  0.24,  0.0,     decel_limit, accel_limit)
        # 2nd stage: maps acceleration_error to throttle
        self.acceleration_ctr = PID(300, 0.0, 0.0, -200000, 2200)

        # filter for steering (steering is calculated by inverse kinematics)
        # self.steering_filt = LowPassFilter(0.25, 1.0/50.0)

        # remember last desired acceleration for jerk-calculation
        self.a_des_old = None

    def reset(self):
        # Reset of controller A:
        self.throttle_ctr.reset()
        # Reset of controller B:
        self.velocity_ctr.reset()
        self.acceleration_ctr.reset()

    def control(self, desired_velocity, angular_velocity, current_velocity, current_accel, dt):
        if self.a_des_old is None:
            self.a_des_old = 0

        # steering controller followed by filter
        steering = self.steering_ctr.get_steering(desired_velocity, angular_velocity, current_velocity)
        # steering = self.steering_filt.filt(steering)

        # Controller A:
        # TotalWheelTorque = self.throttle_ctr.step(linear_velocity - current_velocity, dt)

        # Controller B: cascaded velocity and acceleration control with feed-forward
        a_des = self.velocity_ctr.step(desired_velocity - current_velocity, dt)

        # when stopping
        # if desired_velocity == 0.0 and 1e-4 < current_velocity < 1.0:
        #     a_des = max(-2, self.decel_limit)

        # rate limiter
        a_delta = a_des - self.a_des_old
        a_delta = max(min(JERK_MAX*dt, a_delta), JERK_MIN*dt)
        a_des = self.a_des_old + a_delta
        # saturation
        a_des = max(min(self.velocity_ctr.max, a_des), self.velocity_ctr.min)
        self.a_des_old = a_des

        # feed forward control
        TotalWheelTorque_FF = self.r*(self.m*a_des + self.d*current_velocity)
        # feed back control
        TotalWheelTorque_FB = self.acceleration_ctr.step(a_des - current_accel, dt)
        TotalWheelTorque = 1*TotalWheelTorque_FF + 1*TotalWheelTorque_FB

        return TotalWheelTorque, steering, a_des
