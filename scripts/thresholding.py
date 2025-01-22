#!/usr/bin/env python
import rospy
import numpy as np
from rosneuro_msgs.msg import NeuroFrame
from std_msgs.msg import Float32MultiArray

def callback(data: NeuroFrame):
	# Load the global variables
	global count, threshold, channel_n

	# Focalize on the specific channel 
	message_length = int(data.eeg.info.nsamples)
	channel_data = np.array(data.eeg.data[ (channel_n - 1) * message_length : (channel_n) * message_length ])
	
	# Compute the mean
	mean_channel = np.mean(channel_data)
	
	# TODO: to remove
	print("Mean: ", mean_channel)
	print("Channel data: ", channel_data)

	# Check for the blink
	for i in range(channel_data.size):
		if( (channel_data[i] - mean_channel) > threshold):
			count = count + 1
			print(count, ": Threshold reached!", " [", channel_data[i], "]")
			break

def main():
	# Init the node
	rospy.init_node('thresholding', anonymous=True)
	# Setup the parameters
	global count, threshold, channel_n
	count  = 0
	# Here the value 9 is a fallback if the param 'threshold' is not found
	threshold = rospy.get_param('threshold', -0.3)
	channel_n = rospy.get_param('channel', 9)

	# Setup the callback
	rospy.Subscriber('eeg/bandpower', NeuroFrame, callback)
	# Wait for the data
	rospy.spin()

if __name__ == '__main__':
  main()