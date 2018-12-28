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
RECORD_CSV = False
CREEPING_TORQUE = 1000
P_THROTTLE = 2200
D_RESIST = 140

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

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        # constants
        self.mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius

        min_speed = 0.1

        self.controller = Controller(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                                     decel_limit, accel_limit, wheel_radius, self.mass, P_THROTTLE, D_RESIST)

        self.csv_fields = ['time', 'x', 'y', 'v', 'v_d', 'a', 'v_des', 'a_des', 'throttle', 'brake', 'steer']
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

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback, queue_size=2)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=2)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback, queue_size=1)

        # lowpass filter
        self.velocity_filt = LowPassFilter(0.1, 1.0/50.0)
        self.acceleration_filt = LowPassFilter(0.05, 1.0 / 50.0)

        # additional variables
        self.dbw_enabled = False
        self.linear_velocity = None
        self.angular_velocity = None
        self.desired_linear_velocity = None
        self.desired_angular_velocity = None
        self.last_loop = None
        self.linear_velocity_old = None
        self.acceleration = None

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
            self.csv_data['v_des'] = self.desired_linear_velocity
            self.csv_data['a_des'] = self.desired_angular_velocity

    def current_velocity_callback(self, data):
        if self.linear_velocity is not None:
            self.linear_velocity_old = self.linear_velocity
        else:
            self.linear_velocity_old = 0

        self.linear_velocity = self.velocity_filt.filt(data.twist.linear.x)
        #self.linear_velocity = data.twist.linear.x
        self.angular_velocity = data.twist.angular.z

        if RECORD_CSV:
            self.csv_data['v'] = self.linear_velocity
            self.csv_data['a'] = self.angular_velocity

    def dbw_enabled_callback(self, tf):
        self.dbw_enabled = tf

    def loop(self):
        rate = rospy.Rate(50)  # Carla wants 50Hz

        while not rospy.is_shutdown():
            now = rospy.get_time()

            if self.last_loop is not None and self.linear_velocity is not None and self.linear_velocity_old is not None:
                dt = now - self.last_loop
                accel = self.acceleration_filt.filt((self.linear_velocity - self.linear_velocity_old) / dt)
                #accel = ((self.linear_velocity - self.linear_velocity_old) / dt)
                self.acceleration = accel
                # rospy.loginfo('dt: %s', dt)
            else:
                accel = 0

            # rospy.loginfo('velocity: %s', self.linear_velocity)
            # rospy.loginfo('acceleration: %s', accel)

            if self.autopilot_ready():
                throttle, steering = self.controller.control(self.desired_linear_velocity,
                                                             self.desired_angular_velocity,
                                                             self.linear_velocity,
                                                             self.acceleration,
                                                             now - self.last_loop)

                if throttle > 0:
                    # accelerate or coast along at constant speed
                    brake = 0.0

                elif throttle > -self.brake_deadband:
                    # a small deadband to avoid braking while coasting
                    if self.desired_linear_velocity <= 0:
                        # minimum braking torque in case of a stop
                        brake = CREEPING_TORQUE
                    else:
                        brake = 0.0
                    # of course we do not step on the gas while braking
                    throttle = 0.0

                else:
                    # we are decelerating (braking)
                    # SIMULATOR: fitted coefficient between throttle and acceleration
                    # mass * acceleration = force | force = torque / radius
                    brake = - P_THROTTLE*throttle
                    brake = max(brake, CREEPING_TORQUE)

                    throttle = 0.0

                self.publish(throttle, brake, steering)

            else:
                # do not publish drive-by-wire commands if we are driving manually
                self.controller.reset()

            if RECORD_CSV:
                self.csv_data['time'] = now
                self.csv_data['v_d'] = accel
                self.csv_writer.writerow(self.csv_data)

            self.last_loop = now
            rate.sleep()

        if RECORD_CSV:
            self.fid.close()

    def autopilot_ready(self):
        current_okay = self.linear_velocity is not None and self.angular_velocity is not None
        desired_okay = self.desired_linear_velocity is not None and self.desired_angular_velocity is not None
        timing_okay = self.last_loop is not None

        return self.dbw_enabled and current_okay and desired_okay and timing_okay

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
