#!/usr/bin/env python
import rospy, numpy as np
from rosneuro_msgs.msg import NeuroFrame, NeuroEvent

def callback(data: NeuroFrame):
	# Global variables
	global threshold, channel_n, pub

	# Focalize on the specific channel 
	message_length = int(data.eeg.info.nsamples)
	channel_data = np.array(data.eeg.data[ (channel_n - 1) * message_length : (channel_n) * message_length ])
	
	# Check for the threshold
	for i in range(channel_data.size):
		if(channel_data[i] < threshold):
			# Publish NeuroEvent
			description = "Threshold reached on channel {} with value {}".format(channel_n, channel_data[i])
			print(description)
			neuro_event = generate_new_message(data, description)
			pub.publish(neuro_event)
			break

def generate_new_message(data: NeuroFrame, description: str):
	# Construct NeuroEvent
	neuro_event = NeuroEvent()
	neuro_event.header = data.header
	neuro_event.version = "1.0"
	neuro_event.description = description

	return neuro_event

def main():
	# Initialize the node
	rospy.init_node('thresholding')

	# Global variables
	global threshold, channel_n, pub
	# Get parameters for thresholding
	threshold = rospy.get_param('threshold', -0.4)
	channel_n = rospy.get_param('channel', 9)

	# Initialize the publisher
	pub = rospy.Publisher('events/bus', NeuroEvent, queue_size=1)
	# Initialize the subscriber
	rospy.Subscriber('eeg/bandpower', NeuroFrame, callback)
	
	# Spin
	rospy.spin()

if __name__ == '__main__':
	main()