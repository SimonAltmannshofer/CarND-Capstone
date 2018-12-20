#!/usr/bin/env python

import os
import PIL

import rospy
from sensor_msgs.msg import Image
from styx_msgs.msg import TrafficLightArray, TrafficLight
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier

LIMIT_CAMERA_FPS = 5
MAX_DUTY_CYCLE = 0.75
IS_SITE = True

class TrafficLightTestNode(object):
    def __init__(self):
        rospy.init_node('tl_test_node')

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(IS_SITE)

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_frame = rospy.get_time()
        self.last_finish = rospy.get_time()
        self.busy = False

        self.has_image = False
        self.camera_image = None

        self.img_dir = '/home/morieris/CarND-Capstone/data/tl_test_onsite'

        if not os.path.exists(self.img_dir):
            os.makedirs(self.img_dir)

        self.k = 0

        rospy.Subscriber('/image_raw', Image, self.image_cb, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()

        # do something

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        now = rospy.get_time()
        self.camera_image = msg
        self.has_image = msg is not None

        if self.busy:
            rospy.logwarn("traffic light detection: SKIP still busy with last frame")
            return
        elif now - self.last_frame < 1.0/LIMIT_CAMERA_FPS:
            # limit camera frames to a reasonable level
            return
        elif now - self.last_finish < (1.0 - MAX_DUTY_CYCLE)/LIMIT_CAMERA_FPS:
            rospy.logwarn("traffic light detection: SKIP duty above {}%".format(round(100*MAX_DUTY_CYCLE)))
            return
        else:
            self.busy = True
            self.last_frame = now

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        img = PIL.Image.fromarray(cv_image)
        self.k += 1
        jpg_file = os.path.join(self.img_dir, "site_{:05d}.jpg".format(self.k))

        img.save(jpg_file)

        state = self.light_classifier.get_classification(cv_image)
        self.last_finish = rospy.get_time()
        self.busy = False


if __name__ == '__main__':
    TrafficLightTestNode()
