#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from lowpass import LowPassFilter
import csv
import os

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
RECORD_CSV = False  # write states and actor-values to a csv-file
RECORD_TIME_TRIGGERED = False  # if True one row will be written in each dbw-cycle, otherwise upon cur-velocity-callback
CREEPING_TORQUE = 800  # minimum braking torque during standstill (avoid creeping)
P_THROTTLE = 2000  # engine power factor [Nm/1]: torque = P_THROTTLE * throttle
D_RESIST = 110  # velocity dependant resistance [N/(m/s)]
P_BRAKE = 1.0  # brake factor [Nm/Nm]

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.decel_limit = decel_limit
        self.accel_limit = accel_limit

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Initialize constants for feed-forward-control
        self.mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius
        rospy.loginfo("DriveByWire with Feed Forward Control: mass={}kg and wheel_radius={}m".format(
            self.mass, self.wheel_radius
        ))

        # Create controller object
        min_speed = 0.1
        self.controller = Controller(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                                     decel_limit, accel_limit, wheel_radius, self.mass, P_THROTTLE, P_BRAKE, D_RESIST)

        # optional logging to a csv-file
        self.csv_fields = ['time', 'x', 'y', 'v_raw', 'v', 'v_d', 'a', 'v_des', 'v_d_des', 'a_des', 'throttle', 'throttle_des', 'brake', 'brake_des', 'steer']
        if RECORD_CSV:
            base_path = os.path.dirname(os.path.abspath(__file__))
            base_path = os.path.dirname(base_path)
            base_path = os.path.dirname(base_path)
            base_path = os.path.dirname(base_path)
            base_path = os.path.join(base_path, 'data', 'records')
            if not os.path.exists(base_path):
                os.makedirs(base_path)
            csv_file = os.path.join(base_path, 'driving_log.csv')

            rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
            rospy.Subscriber('/vehicle/steering_report', SteeringReport, self.steering_callback)
            rospy.Subscriber('/vehicle/throttle_report', Float32, self.throttle_callback)
            rospy.Subscriber('/vehicle/brake_report', Float32, self.brake_callback)
            self.fid = open(csv_file, 'w')
            self.csv_writer = csv.DictWriter(self.fid, fieldnames=self.csv_fields)
            self.csv_writer.writeheader()
            self.csv_data = {key: 0.0 for key in self.csv_fields}

            rospy.logwarn("created logfile for manual driving: " + self.fid.name)

        # Subscribe to all required topics
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback, queue_size=2)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=2)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback, queue_size=1)

        # lowpass filter
        self.velocity_filt = LowPassFilter(0.1, 1.0/50.0)
        self.acceleration_filt = LowPassFilter(0.1, 1.0 / 50.0)
        self.throttle_filt = LowPassFilter(0.06, 1.0 / 50.0)
        self.brake_filt = LowPassFilter(0.06, 1.0 / 50.0)


        # additional variables
        self.dbw_enabled = False
        self.current_linear_velocity = None
        self.angular_velocity = None
        self.desired_linear_velocity = None
        self.desired_angular_velocity = None
        self.last_loop = None
        self.linear_velocity_old = None
        self.current_acceleration = None
        self.desired_velocity_old = 0.0

        self.dbw_ready = False
        self.acc_ready = False

        self.loop()

    def pose_callback(self, msg):
        self.csv_data['x'] = msg.pose.position.x
        self.csv_data['y'] = msg.pose.position.y

    def steering_callback(self, msg):
        self.csv_data['steer'] = msg.steering_wheel_angle_cmd

    def throttle_callback(self, msg):
        self.csv_data['throttle'] = msg.data

    def brake_callback(self, msg):
        self.csv_data['brake'] = msg.data

    def twist_cmd_callback(self, data):
        # callback of desired velocities
        self.desired_linear_velocity = data.twist.linear.x

        if self.desired_linear_velocity < 0:
            rospy.logwarn("negative linear.x velocity requested: reverse is not available yet")
            self.desired_linear_velocity = 0.0

        self.desired_angular_velocity = data.twist.angular.z

        if RECORD_CSV:
            self.csv_data['a_des'] = self.desired_angular_velocity

    def current_velocity_callback(self, data):
        # callback of current vehicle velocities
        if self.current_linear_velocity is not None:
            self.linear_velocity_old = self.current_linear_velocity
        else:
            self.linear_velocity_old = 0

        self.current_linear_velocity = self.velocity_filt.filt(data.twist.linear.x)
        self.angular_velocity = data.twist.angular.z

        if RECORD_CSV:
            self.csv_data['v_raw'] = data.twist.linear.x
            self.csv_data['v'] = self.current_linear_velocity
            self.csv_data['a'] = self.angular_velocity

            if not RECORD_TIME_TRIGGERED:
                self.csv_data['time'] = rospy.get_time()
                self.csv_writer.writerow(self.csv_data)

    def dbw_enabled_callback(self, tf):
        self.dbw_enabled = tf

    def loop(self):
        rate = rospy.Rate(50)  # Carla wants 50Hz

        while not rospy.is_shutdown():
            now = rospy.get_time()

            if self.acceleration_calc_ready():
                # calculate current acceleration (derivative of velocity)
                dt = now - self.last_loop
                acceleration_raw = (self.current_linear_velocity - self.linear_velocity_old) / dt
                self.current_acceleration = self.acceleration_filt.filt(acceleration_raw)
            else:
                self.current_acceleration = 0.0

            if self.autopilot_ready():
                # rate limiter for velocity
                desired_velocity_change = self.desired_linear_velocity - self.desired_velocity_old
                allowed_velocity_change = max(min(self.accel_limit * dt, desired_velocity_change),
                                                  self.decel_limit*dt)
                self.desired_velocity_old = self.desired_velocity_old + allowed_velocity_change

                total_wheel_torque, steering, v_d_des = self.controller.control(self.desired_velocity_old,
                                                             self.desired_angular_velocity,
                                                             self.current_linear_velocity,
                                                             self.current_acceleration,
                                                             now - self.last_loop)

                if total_wheel_torque > 0:
                    # accelerate or coast along at constant speed
                    brake = 0.0
                    throttle = total_wheel_torque/P_THROTTLE

                elif total_wheel_torque > -300: # -self.brake_deadband*P_THROTTLE:
                    # a small deadband to avoid braking while coasting
                    if self.desired_linear_velocity <= 1.0:
                        # minimum braking torque in case of a stop
                        brake = CREEPING_TORQUE
                        # reset controller when standing
                        # self.controller.reset()
                    else:
                        brake = 0.0
                    # of course we do not step on the gas while braking
                    throttle = 0.0

                else:
                    # we are decelerating (braking)
                    # SIMULATOR: fitted coefficient between throttle and acceleration
                    if self.desired_linear_velocity <= 0.1 and self.current_linear_velocity <= 1.0:
                        brake = CREEPING_TORQUE
                        # reset controller when standing
                        self.controller.reset()
                    else:
                        # mass * acceleration = force | force = torque / radius
                        brake = - total_wheel_torque/P_BRAKE

                    throttle = 0.0

                # throttle = self.throttle_filt.filt(throttle)
                # brake = self.brake_filt.filt(brake)
                self.publish(throttle, brake, steering)

            else:
                # do not publish drive-by-wire commands if we are driving manually
                self.controller.reset()
                v_d_des = 0.0
                throttle = 0.0
                brake = 0.0

            if RECORD_CSV:
                self.csv_data['v_d'] = self.current_acceleration
                self.csv_data['v_des'] = self.desired_velocity_old
                self.csv_data['v_d_des'] = v_d_des
                self.csv_data['throttle_des'] = throttle
                self.csv_data['brake_des'] = brake
                if RECORD_TIME_TRIGGERED:
                    self.csv_data['time'] = now
                    self.csv_writer.writerow(self.csv_data)

            self.last_loop = now
            rate.sleep()

        if RECORD_CSV:
            self.fid.close()

    def autopilot_ready(self):
        # returns true if the drive-by-wire is fully initialized and ready
        if not self.dbw_ready:
            current_okay = self.current_linear_velocity is not None and self.angular_velocity is not None
            desired_okay = self.desired_linear_velocity is not None and self.desired_angular_velocity is not None
            timing_okay = self.last_loop is not None

            self.dbw_ready = current_okay and desired_okay and timing_okay

        return self.dbw_enabled and self.dbw_ready

    def acceleration_calc_ready(self):
        # return True if all data required for the acceleration calculation is available
        if not self.acc_ready:
            timing_okay = self.last_loop is not None
            velocity_okay = self.current_linear_velocity is not None and self.linear_velocity_old is not None

            self.acc_ready = timing_okay and velocity_okay

        return self.acc_ready

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
