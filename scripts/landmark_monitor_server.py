#! /usr/bin/env python3

import rospy
from tbot_msgs.srv import (GetClosest, GetClosestResponse, GetDistance, GetDistanceResponse)
from nav_msgs.msg import Odometry
import math

class LandmarkMonitor(object):
	def __init__(self):
		self.landmarks = {
			"Cube": (31, -.099),
			"Dumpster": (.11, -2.42),
			"Cylinder": (-1.14, -2.88),
			"Barrier": (-2.59, -.83),
			"Bookshelf":(-.09, .53)
		} #dict of landmarks and locations
		self._x = 0
		self._y = 0 #current x and y of the robot

	def get_closest(self, req):
		rospy.loginfo('GetClosest called')
		best_landmark = ''
		best_distance = -2
		for name, (x,y) in self.landmarks.items():
			dx = x -self._x
			dy = y - self._y
			sq_dist = dx*dx + dy*dy
			if (best_distance == -2 or sq_dist < best_distance):
				best_distance = sq_dist
				best_landmark = name
		response = GetClosestResponse()
		response.name = best_landmark
		return response #return the response for a service, or nothing for failure


	def get_distance(self, req):
		rospy.loginfo('GetDistance called with {}'.format(req.name))
		if req.name not in self.landmarks:
			rospy.logerr('unknown landmark "{}"'.format(req.name))
			return None
		x,y = self.landmarks[req.name]
		dx = x - self._x
		dy = y - self._y
		response = GetDistanceResponse()
		response.distance = math.sqrt(dx*dx + dy*dy)
		return response

	def odom_callback(self, msg):
		self._x = msg.pose.pose.position.x
		self._y = msg.pose.pose.position.y



def main():
	rospy.init_node('landmark_server')
	monitor = LandmarkMonitor()
	get_closest = rospy.Service('get_closest', GetClosest, monitor.get_closest)
	get_distance = rospy.Service('get_distance', GetDistance, monitor.get_distance)
	sub = rospy.Subscriber('/odom', Odometry ,monitor.odom_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
