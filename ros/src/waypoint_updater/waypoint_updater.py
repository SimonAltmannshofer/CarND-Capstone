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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
PLAN_ACCELERATION = 1.0
OVERRIDE_VELOCITY = None


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.car_pose = None
        self.car_pose = None
        self.last_index = None
        self.red_light_wp = -1
        self.force_update = True

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
            # update waypoint distances
            self.update_distances()

    def traffic_cb(self, msg):
        if msg.data != self.red_light_wp:
            # new red light detection
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

        next_wp = self.next_waypoint()
        same_wp = self.last_index is not None and self.last_index == next_wp

        if same_wp and not self.force_update:
            # no update of waypoints required
            return

        last_wp = next_wp + LOOKAHEAD_WPS
        num_waypoints = len(self.waypoints)

        # get next stop waypoint, either end of track or red-light
        if last_wp >= num_waypoints:
            last_wp = num_waypoints - 1

            if next_wp <= self.red_light_wp < last_wp:
                stop_wp = self.red_light_wp
            else:
                stop_wp = last_wp

        else:
            stop_wp = self.red_light_wp

        # if last >= num_waypoints:
        #     # use negative indexing for wrap-around of waypoints
        #     next -= num_waypoints
        #     last -= num_waypoints

        traj_waypoints = self.waypoints[next_wp:last_wp]
        traj_stop_wp = stop_wp - next_wp

        dist_next = self.distance(self.car_pose.position, traj_waypoints[0].pose.pose.position)
        traj_distances = self.waypoint_distances[next_wp:last_wp-1]
        traj_distances.insert(0, dist_next)

        # rospy.loginfo("current waypoint {}, trajectory until {}, with stopping at {} and red-light at {}".format(next_wp, last_wp, stop_wp, self.red_light_wp))

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
                # waypoints[i].twist.twist.linear.x = v_traj
                self.set_waypoint_velocity(waypoints, i, min(self.velocity, v_traj))

            # rospy.loginfo("resume with target speed, we have {} waypoints and stop is {}".format(num_waypoints, stop_index))

        else:
            # stop at stop-line
            dist_rem = sum(distances[0:stop_index])
            # rospy.loginfo("remaining brake distance: {} reducing traj-speed from {}".format(dist_rem, current_velocity))

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
