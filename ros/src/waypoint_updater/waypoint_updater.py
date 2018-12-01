#!/usr/bin/env python

import rospy
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.car_pose = None
        self.last_index = None

        rospy.spin()

    def pose_cb(self, msg):
        self.car_pose = msg.pose

        if self.waypoints is not None:
            # we already have some map data
            self.publish_final_waypoints()

    def waypoints_cb(self, static_lane):
        if self.waypoints is None:
            self.waypoints = static_lane.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def closest_waypoint(self, waypoints=None, pose=None):
        if waypoints is None:
            waypoints = self.waypoints
        if pose is None:
            pose = self.car_pose

        # ego car position in map-coordinates
        car_x = pose.position.x
        car_y = pose.position.y

        # initialize search
        dmin = 1e12
        index = -1

        # simple linear search as in path-planning-project
        for k, wp in enumerate(waypoints):
            dx = wp.pose.pose.position.x - car_x
            dy = wp.pose.pose.position.y - car_y

            d2 = dx*dx + dy*dy

            if d2 < dmin:
                dmin = d2
                index = k

        return index

    def next_waypoint(self, waypoints=None, pose=None):
        """
        returns next waypoint ahead of us
        Python copy of the Udacity code from Path-Planning project
        :param waypoints:
        :param pose:
        :return:
        """
        if waypoints is None:
            waypoints = self.waypoints
        if pose is None:
            pose = self.car_pose

        closest = self.closest_waypoint(waypoints, pose)

        map_x = waypoints[closest].pose.pose.position.x
        max_y = waypoints[closest].pose.pose.position.y

        car_x = pose.position.x
        car_y = pose.position.y

        heading = math.atan2(max_y - car_y, map_x - car_x)

        # we need the yaw angle, transform quaternion
        q = pose.orientation
        yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)

        # check angle and go to next waypoint if necessary
        angle = abs(yaw - heading)
        angle = min(2*math.pi - angle, angle)

        if angle > math.pi / 4.0:
            closest += 1
            if closest == len(waypoints):
                closest = 0

        return closest

    def publish_final_waypoints(self):
        if self.waypoints is None or self.car_pose is None:
            # early exit due to missing data
            return

        next = self.next_waypoint()

        if self.last_index is not None and self.last_index == next:
            # no update of waypoints required
            return

        last = next + LOOKAHEAD_WPS

        rospy.loginfo("waypoints from {} until {}".format(next, last))

        num_waypoints = len(self.waypoints)

        if last >= num_waypoints:
            # use negative indexing for wrap-around of waypoints
            next -= num_waypoints
            last -= num_waypoints

        final_waypoints = self.waypoints[next:last]

        # TODO: settings of proper speeds ... just approx 10mph fixed for now
        for i in range(len(final_waypoints)):
            final_waypoints[i].twist.twist.linear.x = 5.0

        lane = Lane()
        lane.header.frame_id ='/seppi'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = final_waypoints

        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
