#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import sys
import tf
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
BRAKE_DISTANCE = 25
MARGIN_TO_LIGHT = 1.0
TARGET_SPEED = 20

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

        rospy.spin()

    def velocity_cb(self, msg):
        self.velocity = msg.twist.linear.x

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        #rospy.loginfo("wp_updater: Current position %f, %f", self.current_pose.position.x, self.current_pose.position.y)
        orientation = self.current_pose.orientation
        self.theta = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        self.publish_waypoints()

    def waypoints_cb(self, lane):
        self.base_waypoints = lane.waypoints
        self.publish_waypoints()

    def traffic_cb(self, msg):
        rospy.loginfo("wp_updater: Traffic waypoint received %i", msg.data)
        self.next_stop = msg.data
        self.publish_waypoints()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def publish_waypoints(self):
        if self.base_waypoints and self.current_pose:
            car_p = self.current_pose
            base_wp = self.base_waypoints
            closest_wp = self.closest_waypoint(base_wp, car_p)
            mapx = base_wp[closest_wp].pose.pose.position.x
            mapy = base_wp[closest_wp].pose.pose.position.y
            heading = math.atan2(mapy-car_p.position.y, mapx-car_p.position.x)
            angle = abs(self.theta - heading)
            if angle > math.pi/4:
                closest_wp += 1
            rospy.loginfo("wp_updater: Next wp for car %f, %f is %i", car_p.position.x, car_p.position.y, closest_wp)

            deceleration = 0.0
            if self.upcoming_red_light >= 0:
                distance_to_stop = self.distance(self.base_waypoints, closest_wp, self.upcoming_red_light) - MARGIN_TO_LIGHT
                if self.distance(self.base_waypoints, closest_wp, self.upcoming_red_light) < BRAKE_DISTANCE:
                    deceleration = (self.velocity / distance_to_stop)*1.1

            for i in range(len(base_wp)):
                self.set_waypoint_velocity(base_wp, i, TARGET_SPEED)
            #FIXME: In case stop at wp is a low number, and closes wp is large, but they are close
            for i in range(closest_wp, self.upcoming_red_light):
                delta = self.distance(self.base_waypoints, closest_wp, i)
                self.set_waypoint_velocity(base_wp, i, max(TARGET_SPEED-deceleration*delta,0))

            final_waypoints = Lane()
            for i in range(LOOKAHEAD_WPS):
                final_waypoints.waypoints.append(base_wp[(i+closest_wp)%len(base_wp)])

            final_waypoints.header.stamp = rospy.Time(0)
            self.final_waypoints_pub.publish(final_waypoints)

    def closest_waypoint(self, waypoints, car_pose):
        dist = sys.maxsize
        idx = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        for i in range(len(waypoints)):
            d = dl(waypoints[i].pose.pose.position, car_pose.position)
            if d < dist:
                dist = d
                idx = i
        #rospy.loginfo("wp_updater: Closest waypoint for car %f, %f is index %i", car_pose.position.x, car_pose.position.y, idx)
        return idx 

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
