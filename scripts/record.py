#!/usr/bin/env python
import rospy, rosbag
from rosneuro_msgs.msg import NeuroFrame, NeuroEvent

def eeg_filtered_callback(data: NeuroFrame):
	# ROS bag
	global bag, eeg_filtered
	try:
		# Write bag
		bag.write(eeg_filtered, data)
	except:
		# Close bag
		bag.close()

def eeg_bandpower_callback(data: NeuroFrame):
	# ROS bag
	global bag, eeg_bandpower
	try:
		# Write bag
		bag.write(eeg_bandpower, data)
	except:
		# Close bag
		bag.close()

def events_bus_callback(data: NeuroEvent):
	# ROS bag
	global bag, events_bus
	try:
		# Write bag
		bag.write(events_bus, data)
	except:
		# Close bag
		bag.close()

def main():
	# Init the node
	rospy.init_node('record')
	
	# ROS bag
	global bag, eeg_filtered, eeg_bandpower, events_bus
	bag = rosbag.Bag('../record/result.bag', 'w')

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