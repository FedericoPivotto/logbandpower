#!/usr/bin/env python
import rospy
import numpy as np
from rosneuro_msgs.msg import NeuroFrame, NeuroEvent

def callback(data: NeuroFrame):
	# Load the global variables
	global threshold, channel_n, pub

	# Focalize on the specific channel 
	message_length = int(data.eeg.info.nsamples)
	channel_data = np.array(data.eeg.data[ (channel_n - 1) * message_length : (channel_n) * message_length ])
	
	# Check for the blink
	for i in range(channel_data.size):
		if(channel_data[i] < threshold):
			description = "Threshold reached on channel {} with value {}".format(channel_n, channel_data[i])
			# TODO: Debug print to remove before delivering
			print(description)
			neuro_event = generate_new_message(data, description)
			pub.publish(neuro_event)
			break

def generate_new_message(data, description):
	# Construct NeuroEvent
	neuro_event = NeuroEvent()
	neuro_event.header = data.header
	neuro_event.neuroheader = data.neuroheader
	neuro_event.version = "1.0"
	neuro_event.description = description

	return neuro_event

def main():
	# Init the node
	rospy.init_node('thresholding')
	# Setup the parameters
	global threshold, channel_n, pub
	
	# Here the value 9 is a fallback if the param 'threshold' is not found
	threshold = rospy.get_param('threshold', -0.4)
	channel_n = rospy.get_param('channel', 9)

	# Publisher
	pub = rospy.Publisher('events/bus', NeuroEvent, queue_size=1)

	# Setup the callback
	rospy.Subscriber('eeg/bandpower', NeuroFrame, callback)
	# Wait for the data
	rospy.spin()

if __name__ == '__main__':
	main()