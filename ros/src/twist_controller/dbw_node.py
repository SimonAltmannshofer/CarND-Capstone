#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

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
        min_speed = 0.1

        self.controller = Controller(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                                     decel_limit, accel_limit)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback, queue_size=2)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=2)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback, queue_size=1)

        # additional variables
        self.dbw_enabled = False
        self.linear_velocity = None
        self.angular_velocity = None
        self.desired_linear_velocity = None
        self.desired_angular_velocity = None
        self.last_loop = None

        # constants
        self.mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius

        self.loop()

    def twist_cmd_callback(self, data):
        # callback of desired velocities
        self.desired_linear_velocity = data.twist.linear.x

        if self.desired_linear_velocity < 0:
            rospy.logwarn("negative linear.x velocity requested: reverse is not available yet")
            self.desired_linear_velocity = 0.0

        self.desired_angular_velocity = data.twist.angular.z

    def current_velocity_callback(self, data):
        self.linear_velocity = data.twist.linear.x
        self.angular_velocity = data.twist.angular.z

    def dbw_enabled_callback(self, tf):
        self.dbw_enabled = tf

    def loop(self):
        rate = rospy.Rate(50)  # Carla wants 50Hz

        while not rospy.is_shutdown():
            now = rospy.get_time()

            if self.autopilot_ready():
                acceleration, steering = self.controller.control(self.desired_linear_velocity,
                                                                 self.desired_angular_velocity,
                                                                 self.linear_velocity,
                                                                 now - self.last_loop)

                if acceleration > 0:
                    brake = 0.0
                    throttle = acceleration

                elif acceleration > -self.brake_deadband:
                    brake = 0.0
                    throttle = 0.0

                else:
                    # mass * acceleration = force | force = torque / radius
                    brake = -acceleration * self.mass * self.wheel_radius
                    throttle = 0.0

                rospy.logwarn("current speed {} vs. target speed {}, acceleration {}: throttle {} and brake {}".format(
                    self.linear_velocity, self.desired_linear_velocity, acceleration, throttle, brake
                ))

                self.publish(throttle, brake, steering)

            else:
                # do not publish drive-by-wire commands if we are driving manually
                self.controller.reset()

            self.last_loop = now
            rate.sleep()

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
