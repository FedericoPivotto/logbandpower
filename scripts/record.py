#!/usr/bin/env python
import rospy, rosbag
from rosneuro_msgs.msg import NeuroFrame, NeuroEvent

def eeg_filtered_callback():
	pass

def eeg_bandpower_callback():
	pass

def events_bus_callback():
	pass

def main():
	# Init the node
	rospy.init_node('record')
	
	# ROS bag
	global bag
	bag = rosbag.Bag('result.bag', 'w')

    # Topics to record
	eeg_filtered = rospy.get_param('eeg_filtered', 'eeg/filtered')
	eeg_bandpower = rospy.get_param('eeg_bandpower', 'eeg/bandpower')
	events_bus = rospy.get_param('events_bus', 'events/bus')

	# Subscribers
	rospy.Subscriber(eeg_filtered, NeuroFrame, eeg_filtered_callback)
	rospy.Subscriber(eeg_bandpower, NeuroFrame, eeg_bandpower_callback)
	rospy.Subscriber(events_bus, NeuroEvent, events_bus_callback)

	# Spin callbacks
	rospy.spin()

if __name__ == '__main__':
	main()