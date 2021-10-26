#! /usr/bin/python

import rospy
import random
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point
from math import fabs, cos, sin, floor


class ScanFilter:
	def __init__(self):
		rospy.init_node('scan_filter')
		rospy.Subscriber('/base_scan', LaserScan, self.filter)
		self.pub = rospy.Publisher('/base_scan_filtered', LaserScan, queue_size=1)
		self.pub2 = rospy.Publisher('/base_ocmap', OccupancyGrid, queue_size=1)

	def filter(self, msg):
		ranges = list(msg.ranges)
		filtered_ranges = []
		mean = 0			
		for i in range(4, len(ranges)):
			_sum = 0
			for j in range(1, 5):
				_sum += ranges[i-j]
			mean = _sum / 4
			if not fabs(ranges[i] - mean) > 0.1 and ranges[i] < msg.range_max and ranges[i] > msg.range_min:
				filtered_ranges.append(ranges[i])
			else:
				filtered_ranges.append(-1)
		msg.ranges = tuple(filtered_ranges)
		self.pub.publish(msg)

		oc_msg = OccupancyGrid()

		active_cells = []
		point_counts = []
		for i in range(100):
			point_counts.append(-1)
		for i in range(len(filtered_ranges)):
			point_x = int(floor(filtered_ranges[i] * cos(msg.angle_min + i*msg.angle_increment)) + 5)
			point_y = int(floor(filtered_ranges[i] * sin(msg.angle_min + i*msg.angle_increment)) + 5)
			if point_x in range(10) and point_y in range(10):
				active_cell = {'x': point_x, 'y': point_y}
				point_counts[point_y*10 + point_x] += 1
				if active_cell not in active_cells:
					active_cells.append(active_cell)

		msg_data = []
		for i in range(100):
			msg_data.append(0)	
		for cell in active_cells:
			x = cell['x']
			y = cell['y']
			if point_counts[y*10 + x] > 50:
				msg_data[y*10 + x] = 100
			else:
				msg_data[y*10 + x] = point_counts[y*10 + x]*2
		oc_msg.data = msg_data	
		oc_msg.info.resolution = 1
		oc_msg.info.width = 10
		oc_msg.info.height = 10
		oc_msg.info.origin = Pose()
		oc_msg.info.origin.position = Point()
		oc_msg.info.origin.position.x = -5
		oc_msg.info.origin.position.y = -5
		self.pub2.publish(oc_msg)


ScanFilter()
rospy.spin()
