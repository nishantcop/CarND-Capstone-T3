#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

from scipy.spatial import cKDTree, distance

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add other member variables you need below
                
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints = None
        self.waypoints_xy = None
        self.waypoints_build_cKDTree = None
        self.pose = None
        self.stopline_wd_idx = -1

        self.set_rate()
    
    def set_rate(self):
	# Set looping rate to 50Hz
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement        
        self.pose = msg

    def waypoints_cb(self, waypoints):	# waypoints callback function
        # TODO: Implement
	# Store waypoints
	self.base_waypoints = waypoints
	# Find 200 closest waypoints ahead of the car
	# Use k-nearest-neighbor-algorithm to find closest waypoints (cKDTree)
	# Waypoint_xy has to be initialized before subscriber 
	if not self.waypoints_xy:
	    # Get x and y position of each waypoint
	    self.waypoints_xy = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.base_waypoints.waypoints]
	    self.waypoints_build_cKDTree = cKDTree(self.waypoints_xy)
        
    def calculate_final_waypoints(self):
	# Query the tree for closest nearest neighbor
	_, index_closest = self.waypoints_build_cKDTree.query([self.pose.pose.position.x, self.pose.pose.position.y], k=1)
	# Check if closest waypoint is ahead of car
	# Get closest waypoints x and y coordinates
	closest_xy = self.waypoints_xy[index_closest]
	previous_xy = self.waypoints_xy[index_closest - 1]
	# Use cosine similarity to check if closest waypoint is ahead of car
	cosine_similarity = 1 - distance.cosine(previous_xy, closest_xy)
	# If cosine_similarity = [0,1] -> closest waypoint is ahead of car
	if not cosine_similarity >= 0 and cosine_similarity <= 1:
	    index_closest = index_closest + 1
	return index_closest

    def publish_waypoints(self):
	final_lane = self.generate_lane()
	self.final_waypoints_pub.publish(final_lane)
        
       
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wd_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
    
    def generate_lane(self):
        lane = Lane()

        nearest_idx = self.calculate_final_waypoints()
        farthest_idx = nearest_idx + LOOKAHEAD_WPS

        base_waypoints = self.base_waypoints.waypoints[nearest_idx:farthest_idx]

        if self.stopline_wd_idx == -1 or (self.stopline_wd_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, nearest_idx)
        
        return lane
    
    def decelerate_waypoints(self, waypoints, nearest_idx):
        temp = []

        for i, wp in enumerate(waypoints):
            p = Waypoint()

            p.pose = wp.pose

            stop_idx = max(self.stopline_wd_idx - nearest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)

            if vel < 1:
                vel = 0
            
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        
        return temp



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
