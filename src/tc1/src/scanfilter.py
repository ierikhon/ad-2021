#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from math import fabs


class ScanFilter:
	def __init__(self):
		rospy.init_node('scan_filter')
		rospy.Subscriber('/base_scan', LaserScan, self.filter)
		self.pub = rospy.Publisher('/base_scan_filtered', LaserScan, queue_size=1)

	def filter(self, msg):
		ranges = list(msg.ranges)
		filtered_ranges = []
		mean = 0
		for i in range(3, len(ranges)):
			mean = (ranges[i-3] + ranges[i-2] + ranges[i-1])/3
			if not fabs(ranges[i] - mean) > 0.2 and ranges[i] < msg.range_max and ranges[i] > msg.range_min:
				filtered_ranges.append(ranges[i])
				ranges[i] = mean
			
		msg.ranges = tuple(filtered_ranges)
		self.pub.publish(msg)


ScanFilter()
rospy.spin()
