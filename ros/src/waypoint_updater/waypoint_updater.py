#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
PLAN_ACCELERATION = 1.0
OVERRIDE_VELOCITY = None
NUM_WP_STOP_AFTER_STOPLINE = 1  # some tolerance if we did not stop before the stop-line
NUM_WP_STOP_BEFORE_STOPLINE = 1  # stop a little bit before the stop-line


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.current_waypoint_pub = rospy.Publisher('/current_waypoint', Int32, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.car_pose = None
        self.car_pose = None
        self.last_index = None
        self.red_light_wp = -1
        self.force_update = True
        self.last_car_waypoint = None

        # maximum allowed velocity
        if OVERRIDE_VELOCITY is None:
            self.velocity = rospy.get_param('/waypoint_loader/velocity')
            self.velocity = (self.velocity * 1000.) / (60. * 60.)
        else:
            self.velocity = OVERRIDE_VELOCITY

        # pre calculated
        self.waypoint_distances = None

        rospy.spin()

    def pose_cb(self, msg):
        self.car_pose = msg.pose

        if self.waypoints is not None:
            # we already have some map data
            self.publish_final_waypoints()

    def waypoints_cb(self, static_lane):
        if self.waypoints is None:
            self.waypoints = static_lane.waypoints
            self.last_car_waypoint = None
            # update waypoint distances
            self.update_distances()

    def traffic_cb(self, msg):
        if msg.data != self.red_light_wp:
            # changed traffic light detection
            self.red_light_wp = msg.data
            self.force_update = True

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

    def update_distances(self):
        self.waypoint_distances = []

        for i in range(1, len(self.waypoints)):
            a = self.waypoints[i-1].pose.pose.position
            b = self.waypoints[i].pose.pose.position
            delta = self.distance(a, b)
            self.waypoint_distances.append(delta)

    def closest_waypoint(self):
        """
        finds the closest waypoint to our current position.
        To speed things up the search starts from the last known waypoint if possible.
        The search is stopped once the waypoint-distances start increasing again.
        """

        # ego car position in map-coordinates
        car_x = self.car_pose.position.x
        car_y = self.car_pose.position.y

        # initialize search
        num_waypoints = len(self.waypoints)
        if self.last_car_waypoint is None:
            # last waypoint not know, start in the middle of the track
            self.last_car_waypoint = int(num_waypoints/2)
            # force complete search
            test_all = True
        else:
            # relative search with possible early exit
            test_all = False

        def dist_squared(i):
            """ returns the squared distance to waypoint[i]"""
            dx = self.waypoints[i].pose.pose.position.x - car_x
            dy = self.waypoints[i].pose.pose.position.y - car_y
            return dx**2 + dy**2

        index = self.last_car_waypoint
        d_min = dist_squared(index)

        # force check of all waypoints in case we are way off our last known position
        test_all = test_all or d_min > 10**2

        # search ahead
        for k in range(self.last_car_waypoint+1, num_waypoints):
            d_k = dist_squared(k)

            if d_k < d_min:
                index = k
                d_min = d_k
            elif not test_all:
                break

        # search previous
        for k in range(self.last_car_waypoint-1, -1, -1):
            d_k = dist_squared(k)

            if d_k < d_min:
                index = k
                d_min = d_k
            elif not test_all:
                break

        self.last_car_waypoint = index
        return index

    def next_waypoint(self):
        """
        returns next waypoint ahead of us
        Python copy of the Udacity code from Path-Planning project
        :param waypoints:
        :param pose:
        :return:
        """

        closest = self.closest_waypoint()

        map_x = self.waypoints[closest].pose.pose.position.x
        max_y = self.waypoints[closest].pose.pose.position.y

        car_x = self.car_pose.position.x
        car_y = self.car_pose.position.y

        heading = math.atan2(max_y - car_y, map_x - car_x)

        # we need the yaw angle, transform quaternion
        # https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        q = self.car_pose.orientation
        yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)

        # check angle and go to next waypoint if necessary
        angle = abs(yaw - heading)
        angle = min(2*math.pi - angle, angle)

        if angle > math.pi / 4.0:
            closest += 1
            if closest == len(self.waypoints):
                closest = 0

        return closest

    def publish_final_waypoints(self):
        if self.waypoints is None or self.car_pose is None:
            # early exit due to missing data
            return

        next_wp = self.next_waypoint()

        # publish current waypoint
        self.current_waypoint_pub.publish(Int32(next_wp-1))

        same_wp = self.last_index is not None and self.last_index == next_wp

        if same_wp and not self.force_update:
            # no update of waypoints required
            return

        last_wp = next_wp + LOOKAHEAD_WPS
        num_waypoints = len(self.waypoints)

        # get next stop waypoint, either end of track or red-light
        if last_wp >= num_waypoints - 1:
            last_wp = num_waypoints - 1

            if next_wp-NUM_WP_STOP_AFTER_STOPLINE <= self.red_light_wp < last_wp:
                stop_wp = self.red_light_wp - NUM_WP_STOP_BEFORE_STOPLINE
                stop_wp = max(stop_wp, next_wp)
            else:
                stop_wp = last_wp - NUM_WP_STOP_BEFORE_STOPLINE

        else:
            if next_wp-NUM_WP_STOP_AFTER_STOPLINE <= self.red_light_wp:
                stop_wp = self.red_light_wp - NUM_WP_STOP_BEFORE_STOPLINE
                stop_wp = max(stop_wp, next_wp)
            else:
                stop_wp = -1

        traj_waypoints = self.waypoints[next_wp:last_wp]
        traj_stop_wp = stop_wp - next_wp

        dist_next = self.distance(self.car_pose.position, traj_waypoints[0].pose.pose.position)
        traj_distances = self.waypoint_distances[next_wp:last_wp-1]
        traj_distances.insert(0, dist_next)

        # convert path to trajectory (plan ahead with constant acceleration/deceleration)
        traj_waypoints = self.path_to_trajectory(traj_waypoints, traj_distances, traj_stop_wp)

        lane = Lane()
        lane.header.frame_id = '/trajectory'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = traj_waypoints

        # update waypoints
        self.force_update = False
        self.final_waypoints_pub.publish(lane)

    def path_to_trajectory(self, waypoints, distances, stop_index=-1):

        num_waypoints = len(waypoints)
        current_velocity = self.get_waypoint_velocity(waypoints[0])

        if stop_index < 0:
            # accelerate to target speed
            x_traj = current_velocity ** 2 / (2 * PLAN_ACCELERATION)

            for i in range(num_waypoints):
                delta = distances[i]
                x_traj += delta

                v_traj = math.sqrt(2*x_traj*PLAN_ACCELERATION)
                self.set_waypoint_velocity(waypoints, i, min(self.velocity, v_traj))

        else:
            # stop at stop-line
            dist_rem = sum(distances[0:stop_index])

            for i in range(num_waypoints):

                if dist_rem > 0:
                    v_traj = math.sqrt(2*dist_rem*PLAN_ACCELERATION)
                else:
                    v_traj = 0.0

                v_traj = min(self.velocity, v_traj)

                self.set_waypoint_velocity(waypoints, i, v_traj)

                delta = distances[i]
                dist_rem -= delta

        return waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
