#!/usr/bin/env python
import rospy, numpy as np
from rosneuro_msgs.msg import NeuroFrame, NeuroDataInfo, NeuroDataFloat

def callback(data: NeuroFrame):
	global new_data, current_frame
	# Save the new data
	current_frame = data
	new_data = True

def buffered_bandpower(data: NeuroFrame):
	# Global variables
	global buffer, seq, avgs
	# Initialization
	if(seq==0):
		# Create a zero filled matrix
		buffer  = np.zeros((data.eeg.info.nchannels, data.sr))
		# Create a zero filled array
		avgs = np.zeros((data.eeg.info.nchannels,))
		
	# Update buffer with the new data
	eeg_data = np.array(data.eeg.data).reshape((data.eeg.info.nchannels, data.eeg.info.nsamples))
	# Remove the old data from the buffer
	buffer = np.delete(buffer,[index for index in range(data.eeg.info.nsamples)], axis=1)
	# Add the new data to the buffer
	buffer = np.hstack((buffer, eeg_data))

	# Update the sequence number
	seq = seq + 1

	# If the buffer is filled
	if(seq * data.eeg.info.nsamples >= data.sr):
		# Processing
		for index in range(data.eeg.info.nchannels):
			# Compute the bandpower
			current_row = np.multiply(buffer[index], buffer[index])
			current_row = np.log10(current_row)
			avgs[index] = np.average(current_row)
	
	return tuple(avgs)

def generate_new_message(data: tuple, frame: NeuroFrame):
	# Construct NeuroDataInfo message
	neuro_data_info = NeuroDataInfo()
	neuro_data_info = frame.eeg.info
	neuro_data_info.nsamples = 1
	neuro_data_info.stride = 0

	# Construct NeuroDataFloat message
	neuro_data = NeuroDataFloat()
	neuro_data.info = neuro_data_info
	neuro_data.data = data 

	# Construct NeuroFrame message
	neuro_frame = NeuroFrame()
	neuro_frame.header = frame.header
	neuro_frame.sr = frame.sr
	neuro_frame.eeg = neuro_data

	return neuro_frame

def main():
	# Initialize the node
	rospy.init_node('bandpower')

	# Avoid warnings when applying log10
	np.seterr(divide = 'ignore')
	
	# Global variables
	global new_data, current_frame, seq
	# Initialization
	new_data = False
	seq = 0	

	# Initialize the publisher
	pub = rospy.Publisher('eeg/bandpower', NeuroFrame, queue_size=1)	
	# Initialize the subscriber
	rospy.Subscriber('eeg/filtered', NeuroFrame, callback)
	
	# Loop bandpower publish
	hz = rospy.get_param('rate', 16)
	rate = rospy.Rate(hz)
	while not rospy.is_shutdown():
		# Wait until new data arrives
		if new_data:
			new_data = buffered_bandpower(current_frame)
			neuro_frame = generate_new_message(new_data, current_frame)
			pub.publish(neuro_frame)
			new_data = False
		rate.sleep()
		
if __name__ == '__main__':
    main()