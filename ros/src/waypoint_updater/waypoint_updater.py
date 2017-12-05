#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from enum import Enum

import math
import sys
import tf
import numpy as np

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

#LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
BRAKE_DISTANCE = 40
MARGIN_TO_LIGHT = 10
TARGET_SPEED = 10
STATE_DELAY_COUNTER = 14 # approx 0.25 s

class CarState(Enum):
    DRIVING = 1
    STOPPING = 2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.current_pose = None
        self.upcoming_red_light = -1
        self.theta = 0.0
        self.velocity = 0.0
        self.last_wp = 0
        self.waypoints_array = None
        self.loop = 0

        self.state = CarState.DRIVING
        self.state_count = 0

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.publish_waypoints()
            rate.sleep()
        rospy.spin()

    def velocity_cb(self, msg):
        self.velocity = msg.twist.linear.x

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        #rospy.loginfo("wp_updater: Current position %f, %f", self.current_pose.position.x, self.current_pose.position.y)
        orientation = self.current_pose.orientation
        self.theta = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        self.loop += 1
        #self.publish_waypoints()

    def waypoints_cb(self, lane):
        self.base_waypoints = lane.waypoints
        self.waypoints_array = np.asarray([(w.pose.pose.position.x, w.pose.pose.position.y) for w in lane.waypoints])
        rospy.loginfo("wp_updater: Waypoints array size %i %i", len(self.base_waypoints), len(self.waypoints_array))

        ##self.publish_waypoints()

    def traffic_cb(self, msg):
        if msg.data >= 0:
            rospy.loginfo("wp_updater: %i traffic wp %i, car wp %i", self.loop, msg.data, self.last_wp)
        if msg.data == -1 and self.state == CarState.STOPPING and self.state_count < STATE_DELAY_COUNTER:
            self.state_count += 1
        else:
            self.upcoming_red_light = msg.data
            self.state_count = 0
            if self.upcoming_red_light >= 0:
                self.state = CarState.STOPPING
            else:
                self.state = CarState.DRIVING

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def publish_waypoints(self):
        if self.base_waypoints and self.current_pose:
            car_p = self.current_pose
            base_wp = self.base_waypoints

            closest_wp_idx = self.get_closest_waypoint(car_p.position.x, car_p.position.y)
            self.last_wp = closest_wp_idx

            # Waypoint closest to car is identified and in closest_wp_idx
            deceleration = 0.0
            if self.loop % 50 == 0:
                rospy.loginfo("wp_updater: upcoming_red_light %i, closest_wp_idx %i, distance %f", self.upcoming_red_light, closest_wp_idx,
                        self.distance(self.base_waypoints, closest_wp_idx, self.upcoming_red_light))

            # Set all waypoints to target speed
            for i in range(len(base_wp)):
                self.set_waypoint_velocity(base_wp, i, TARGET_SPEED)

            # Red light identified
            if self.state == CarState.STOPPING:
                distance_to_tl = self.distance(self.base_waypoints, closest_wp_idx, self.upcoming_red_light)
                if distance_to_tl < BRAKE_DISTANCE: # if closer than 40 meters to red
                    deceleration = abs(1.1 * self.velocity / (distance_to_tl - MARGIN_TO_LIGHT))
                    wp_speed = self.velocity
                    if self.loop % 50 == 0:
                        rospy.loginfo("wp_updater: closer than 40, car wp %i, tl wp %i, deceleration: %f, wp_speed: %f", closest_wp_idx, self.upcoming_red_light, deceleration, wp_speed)
                    delta_to_tl = distance_to_tl
                    for i in range(closest_wp_idx, self.upcoming_red_light):
                        delta = self.distance(self.base_waypoints, i, i+1)
                        wp_speed -= delta * deceleration
                        self.set_waypoint_velocity(base_wp, i, max(wp_speed, 0))
                        delta_to_tl -= delta
                        if self.loop % 50 == 0:
                            rospy.loginfo(" -- wp_updater: distance to tl %f, set wp %i to speed %f", delta_to_tl, i, max(wp_speed, 0))

            final_waypoints = Lane()
            for i in range(LOOKAHEAD_WPS):
                final_waypoints.waypoints.append(base_wp[(i+closest_wp_idx)%len(base_wp)])

            final_waypoints.header.stamp = rospy.Time(0)
            self.final_waypoints_pub.publish(final_waypoints)


    def get_closest_waypoint(self, position_x, position_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        index = -1
        if self.waypoints_array is None:
            return index
        position = np.asarray([position_x, position_y])
        dist_squared = np.sum((self.waypoints_array - position)**2, axis=1)
        index = np.argmin(dist_squared)
        return index

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0.0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
