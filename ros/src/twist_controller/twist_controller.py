from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
JERK_MAX = 10.0
JERK_MIN = -10.0
USE_PID_CTL = True
RATE = 50.0  # controller sampling rate (required for fixed frequency filters)
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

        # Controller A: tuned PID controller for the torque
        self.torque_ctl = PID(500, 50, 3, decel_limit * r * m * 1.5, p_throttle)

        # Controller B: cascaded PID controllers with feed-forward
        # 1st stage: maps velocity_error to desired_acceleration
        self.velocity_ctr = PID(3.0, 0.24, 0.0, decel_limit, accel_limit)
        # 2nd stage: maps acceleration_error to throttle
        self.acceleration_ctr = PID(670, 0.0, 0.0, -5.0, 1.0)

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
        # rate limiter of desired_velocity
        # required because the waypoint_follower sends stairs (discontinuous)
        v_delta = desired_velocity - self.v_des_old
        v_delta = max(self.decel_limit * dt * ACCEL_LIMIT_TOL, min(self.accel_limit * dt * ACCEL_LIMIT_TOL, v_delta))
        desired_velocity = self.v_des_old + v_delta
        self.v_des_old = desired_velocity

        # steering controller
        steering = self.steering_ctr.get_steering(desired_velocity, angular_velocity, current_velocity)

        # first stage of controller B: calculation of desired acceleration
        a_des = self.velocity_ctr.step(desired_velocity - current_velocity, dt)

        # limit jerk and clip acceleration to min/max
        # rate limiter
        a_delta = a_des - self.a_des_old
        a_delta = max(min(JERK_MAX, a_delta), JERK_MIN)
        a_des = self.a_des_old + a_delta
        # saturation
        a_des = max(min(self.velocity_ctr.max * dt, a_des), self.velocity_ctr.min * dt)
        self.a_des_old = a_des

        if USE_PID_CTL:
            # Controller A: simple PID controller: velocity_error -> torque
            total_wheel_torque = self.torque_ctl.step(desired_velocity - current_velocity, dt)

        else:
            # Controller B: cascaded velocity and acceleration control with feed-forward
            # feed forward control
            total_wheel_torque_ff = self.r * (self.m * a_des + self.d * current_velocity)
            # feed back control
            total_wheel_torque_fb = self.acceleration_ctr.step(a_des - current_accel, dt)
            total_wheel_torque = 1 * total_wheel_torque_ff + 1 * total_wheel_torque_fb

        return total_wheel_torque, steering, a_des
