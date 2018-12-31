from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
JERK_MAX = 10.0
JERK_MIN = -10.0
USE_CTR_A = True
RATE = 50.0
ACCEL_LIMIT_TOL = 1.05


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
        self.accel_limit = accel_limit

        # default yaw controller from Udacity
        self.steering_ctr = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # Controller A not used anymore: tuned PID controller for the torque
        self.torque_ctl = PID(500, 50, 3, decel_limit*r*m*1.5, p_throttle)

        # Controller B currently in use: cascaded PID controllers with feed-forward
        # 1st stage: maps velocity_error to desired_acceleration
        self.velocity_ctr = PID(3.0,  0.24,  0.0,     decel_limit, accel_limit)
        # 2nd stage: maps acceleration_error to throttle
        self.acceleration_ctr = PID(670, 0.0, 0.0,     -5.0, 1.0)

        # filter for steering (steering is calculated by inverse kinematics)
        self.steering_filt = LowPassFilter(0.25, 1.0/RATE)

        # remember last desired acceleration for jerk-calculation
        self.a_des_old = 0.0
        self.v_des_old = 0.0

    def reset(self):
        # Reset of controller A:
        self.torque_ctl.reset()
        # Reset of controller B:
        self.velocity_ctr.reset()
        self.acceleration_ctr.reset()

        self.a_des_old = 0.0
        self.v_des_old = 0.0

    def control(self, desired_velocity, angular_velocity, current_velocity, current_accel, dt):
        # rate limiter of desired velocity
        v_delta = desired_velocity - self.v_des_old
        v_delta = max(self.decel_limit * dt * ACCEL_LIMIT_TOL, min(self.accel_limit * dt * ACCEL_LIMIT_TOL, v_delta))
        desired_velocity = self.v_des_old + v_delta
        self.v_des_old = desired_velocity

        # steering controller followed by filter
        steering = self.steering_ctr.get_steering(desired_velocity, angular_velocity, current_velocity)
        # steering = self.steering_filt.filt(steering)

        # first stage of controller B
        a_des = self.velocity_ctr.step(desired_velocity - current_velocity, dt)

        # when stopping
        # if desired_velocity == 0.0 and 1e-4 < current_velocity < 1.0:
        #     a_des = max(-2, self.decel_limit)

        # rate limiter
        a_delta = a_des - self.a_des_old
        a_delta = max(min(JERK_MAX, a_delta), JERK_MIN)
        a_des = self.a_des_old + a_delta
        # saturation
        a_des = max(min(self.velocity_ctr.max*dt, a_des), self.velocity_ctr.min*dt)
        self.a_des_old = a_des

        if USE_CTR_A:
            # Controller A:
            TotalWheelTorque = self.torque_ctl.step(desired_velocity - current_velocity, dt)

        else:
            # Controller B: cascaded velocity and acceleration control with feed-forward
            # feed forward control
            TotalWheelTorque_FF = self.r*(self.m*a_des + self.d*current_velocity)
            # feed back control
            TotalWheelTorque_FB = self.acceleration_ctr.step(a_des - current_accel, dt)
            TotalWheelTorque = 1*TotalWheelTorque_FF + 1*TotalWheelTorque_FB

        return TotalWheelTorque, steering, a_des
