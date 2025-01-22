#!/usr/bin/env python
import rospy

def main():
	# Init the node
	rospy.init_node('record')
	
    # Topics to record
	eeg_filtered = rospy.get_param('eeg_filtered', 'eeg/filtered')
	eeg_bandpower = rospy.get_param('eeg_bandpower', 'eeg/bandpower')
	events_bus = rospy.get_param('events_bus', 'events/bus')

	# Subscribers


if __name__ == '__main__':
	main()