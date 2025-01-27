#!/usr/bin/env python
import rospy, rosbag, os
from rosbag import ROSBagException, ROSBagFormatException
from rosneuro_msgs.msg import NeuroFrame, NeuroEvent

def eeg_filtered_callback(data: NeuroFrame):
	# Write bag
	global bag, eeg_filtered
	try:
		bag.write(eeg_filtered, data)
	except ValueError:
		print('ValueError: argument is invalid')

def eeg_bandpower_callback(data: NeuroFrame):
	# Write bag
	global bag, eeg_bandpower
	try:
		bag.write(eeg_bandpower, data)
	except ValueError:
		print('ValueError: argument is invalid')

def events_bus_callback(data: NeuroEvent):
	# Write bag
	global bag, events_bus
	try:
		bag.write(events_bus, data)
	except ValueError:
		print('ValueError: argument is invalid')

def main():
	# Initialize the node
	rospy.init_node('record')

	# Global variables
	global bag, eeg_filtered, eeg_bandpower, events_bus

	try:
		# Create bag
		script_dir = os.path.dirname(os.path.realpath(__file__))
		bag = rosbag.Bag(script_dir +'/../record/result.bag', 'w') 
	except ValueError:
		print('ValueError: argument is invalid')
	except ROSBagException:
		print('ROSBagException: error occurs opening file')
	except ROSBagFormatException:
		print('ROSBagFormatException: bag format is corrupted')

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