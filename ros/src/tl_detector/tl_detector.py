#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import PIL
import os
import math

STATE_COUNT_THRESHOLD = 2
NUM_WP_STOP_AFTER_STOPLINE = 1
LIMIT_CAMERA_FPS = 4
MAX_DUTY_CYCLE = 0.75
SAVE_CAMERA_IMAGES_TO = None  # '/home/USER/CarND-Capstone/data/tl_test_simulator'
CENTER_TO_BUMPER = 2.5
FORCE_RED_LIGHT_SECONDS = 0.0  # Force stop at every stop-line for at least XX seconds
VERBOSE = False  # increased debug messages

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.car_waypoint = None
        self.waypoints = None
        self.lights = None
        self.config = None

        # get stoplines and update waypoints
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.update_stopline_waypoints()

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.config['is_site'])
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_frame = rospy.get_time()
        self.last_finish = rospy.get_time()
        self.last_publish = rospy.get_time() - 1.0

        self.busy = False
        self.last_stop_wp = -1
        self.state_count = 0

        self.is_ready = False

        self.time_forced_stop = rospy.get_time()
        self.forced_stop_wp = -1

        # counter for saving camera images
        self.k = 0

        self.stop_line_waypoints = None

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        rospy.Subscriber('/current_waypoint', Int32, self.current_waypoint_cb)

        if SAVE_CAMERA_IMAGES_TO is not None:
            if not os.path.exists(SAVE_CAMERA_IMAGES_TO):
                os.makedirs(SAVE_CAMERA_IMAGES_TO)

        rospy.spin()

    def current_waypoint_cb(self, msg):
        self.car_waypoint = msg.data

        if FORCE_RED_LIGHT_SECONDS > 0 and self.ready():
            # Forced stop at every stop-line for debugging reasons
            now = rospy.get_time()
            light_index, light_wp = self.next_traffic_light()
            forced_stop_duration = now - self.time_forced_stop

            if light_wp - self.car_waypoint > 5:
                self.time_forced_stop = now
                self.forced_stop_wp = light_wp
                forced_stop_duration = 0.0

            elif forced_stop_duration < FORCE_RED_LIGHT_SECONDS:
                self.forced_stop_wp = light_wp

            else:
                self.forced_stop_wp = -1

            # publish, rate limiting is done by self.publish() method
            done = self.publish(force=False)

            if done and VERBOSE:
                # Console output
                print("Test stop at self.red_light_wp: {}".format(self.stop_line_waypoints))
                print(" --> closest_wp           : {}".format(self.car_waypoint))
                print(" --> inference stop wp    : {}".format(self.last_stop_wp))
                print(" --> self.force_stop_wp   : {}".format(self.forced_stop_wp))
                print(" --> forced stop duration : {}".format(now-self.time_forced_stop))

            if done and self.forced_stop_wp > self.last_stop_wp and forced_stop_duration > 0:
                rospy.logwarn("forced stop of FORCE_RED_LIGHT_SECONDS={}s: {}".format(FORCE_RED_LIGHT_SECONDS,
                                                                                      forced_stop_duration))

    def waypoints_cb(self, static_lane):
        if self.waypoints is None:
            self.waypoints = static_lane.waypoints
            self.update_stopline_waypoints()

    def update_stopline_waypoints(self):
        if self.waypoints is not None and self.config is not None:
            self.stop_line_waypoints = []

            for xy in self.config['stop_line_positions']:
                stop_wp = self.find_stop_waypoint(xy[0], xy[1])
                self.stop_line_waypoints.append(stop_wp)
                rospy.loginfo("stop line waypoint xy = ({}, {}) at waypoint {}".format(xy[0], xy[1], stop_wp))

    def traffic_cb(self, msg):
        # ground truth of traffic lights
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        now = rospy.get_time()

        if self.busy:
            rospy.logwarn("traffic light detection: SKIP still busy with last frame")
            self.publish(force=True)
            return
        elif now - self.last_frame < 1.0/LIMIT_CAMERA_FPS:
            # limit camera frames to a reasonable level
            self.publish(force=True)
            return
        elif now - self.last_finish < (1.0 - MAX_DUTY_CYCLE)/LIMIT_CAMERA_FPS:
            rospy.logwarn("traffic light detection: SKIP duty above {}%".format(round(100*MAX_DUTY_CYCLE)))
            self.publish(force=True)
            return
        else:
            self.busy = True
            self.last_frame = now

        # process camera image using tensorflow inference model
        state = self.process_camera_image(msg)
        # get the next traffic light index and waypoint
        light_index, light_wp = self.next_traffic_light()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_stop_wp = light_wp
            self.publish(force=True)
        else:
            self.publish(force=True)
        self.state_count += 1

        self.last_finish = rospy.get_time()
        self.busy = False

    def publish(self, force=False):
        stop_wp = max(self.forced_stop_wp, self.last_stop_wp)
        now = rospy.get_time()

        # limit publish rate to camera fps
        if force or now - self.last_publish > 1.0/30.0:
            self.upcoming_red_light_pub.publish(Int32(stop_wp))
            self.last_publish = now
            return True
        else:
            return False

    def find_stop_waypoint(self, x, y):
        # initialize search
        dmin = 1e12
        index = -1

        def distance(i_wp1, i_wp2):
            """ returns the distance between to waypoints"""
            dx = self.waypoints[i_wp1].pose.pose.position.x - self.waypoints[i_wp2].pose.pose.position.x
            dy = self.waypoints[i_wp1].pose.pose.position.y - self.waypoints[i_wp2].pose.pose.position.y
            return math.sqrt(dx**2 + dy**2)

        # simple linear search as in path-planning-project
        for k, wp in enumerate(self.waypoints):
            dx = wp.pose.pose.position.x - x
            dy = wp.pose.pose.position.y - y
            d2 = dx * dx + dy * dy

            if d2 < dmin:
                dmin = d2
                index = k

        # search backwards in order to stop before and not on the stop-line
        delta = 0.0
        while index >= 0 and delta < CENTER_TO_BUMPER:
            delta += distance(index, index-1)
            index -= 1

        return index

    def process_camera_image(self, camera_image):
        cv_image = self.bridge.imgmsg_to_cv2(camera_image, "rgb8")

        if SAVE_CAMERA_IMAGES_TO is not None:
            img = PIL.Image.fromarray(cv_image)
            self.k += 1
            jpg_file = os.path.join(SAVE_CAMERA_IMAGES_TO, "site_{:05d}.jpg".format(self.k))
            img.save(jpg_file)
            rospy.loginfo("saved camera frame to: " + jpg_file)

        # Get classification
        return self.light_classifier.get_classification(cv_image)

    def next_traffic_light(self):

        # find the closest visible traffic light (if one exists)
        if self.ready():
            # copy current waypoint to be safe in case of a waypoint interrupt/callback
            car_waypoint = self.car_waypoint

            light_index = -1
            light_wp = -1
            numel_ahead = len(self.waypoints)
            for index, wp in enumerate(self.stop_line_waypoints):
                if wp > car_waypoint-NUM_WP_STOP_AFTER_STOPLINE and wp-car_waypoint < numel_ahead:
                    light_wp = wp
                    light_index = index
                    numel_ahead = wp - car_waypoint

            return light_index, light_wp

    def ready(self):
        if not self.is_ready:
            # check if all callbacks arrived
            waypoints_okay = self.waypoints is not None and self.stop_line_waypoints is not None
            lights_okay = self.lights is not None
            pose_okay = self.car_waypoint is not None
            self.is_ready = waypoints_okay and lights_okay and pose_okay

        return self.is_ready


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
