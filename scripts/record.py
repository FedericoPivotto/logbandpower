#!/usr/bin/env python
import rospy, rosbag, os
from rosneuro_msgs.msg import NeuroFrame, NeuroEvent

def eeg_filtered_callback(data: NeuroFrame):
	# Global variables
	global bag, eeg_filtered
	# Write bag
	bag.write(eeg_filtered, data)

def eeg_bandpower_callback(data: NeuroFrame):
	# Global variables
	global bag, eeg_bandpower
	# Write bag
	bag.write(eeg_bandpower, data)

def events_bus_callback(data: NeuroEvent):
	# Global variables
	global bag, events_bus
	# Write bag
	bag.write(events_bus, data)

def main():
	# Initialize the node
	rospy.init_node('record')

	# Global variables
	global bag, eeg_filtered, eeg_bandpower, events_bus
	
	# Create bag
	script_dir = os.path.dirname(os.path.realpath(__file__))
	bag = rosbag.Bag(script_dir + '/../record/result.bag', 'w')

	# Get topics to record
	eeg_filtered = rospy.get_param('eeg_filtered', 'eeg/filtered')
	eeg_bandpower = rospy.get_param('eeg_bandpower', 'eeg/bandpower')
	events_bus = rospy.get_param('events_bus', 'events/bus')

	# Initialize the subscribers
	rospy.Subscriber(eeg_filtered, NeuroFrame, eeg_filtered_callback)
	rospy.Subscriber(eeg_bandpower, NeuroFrame, eeg_bandpower_callback)
	rospy.Subscriber(events_bus, NeuroEvent, events_bus_callback)

	try:
		# Spin
		rospy.spin()
	finally:
		# Close bag
		bag.close()

if __name__ == '__main__':
	main()