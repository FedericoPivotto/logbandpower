#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import tkinter  as tk 
from tkinter import *
from tkinter import ttk

def callback(data):
	global channel_value,channel
	#UPDATE THE VALUE OF THE BAR
	channel_value.set((data.data[channel-1]+1)/2)

def main():
	global channel_value,channel
	rospy.init_node('bar_plot', anonymous=True)
	rospy.Subscriber('bandpower',Float32MultiArray, callback)
	channel=rospy.get_param('~channel',1)
	print(channel)
	
	#CREATE THE WINDOW
	window = tk.Tk()
	window.geometry("400x200")
	window.title('Bandpower of channel ' + str(channel))
	
	#CREATE THE VARIABLE WHICH WILL CONTAIN THE VALUE OF THE BAR
	channel_value = DoubleVar()
	
	#CREATE THE DYNAMIC BAR
	progress_bar = ttk.Progressbar(window,orient = HORIZONTAL, variable=channel_value, maximum=1, mode ="determinate")
	progress_bar.pack(fill=X, expand=1)
	
	#CREATE THE LABELS
	label_0 = Label( window, text='0')
	label_0.place(relx=.01,rely=.3)
	label_1 = Label( window, text='1')
	label_1.place(relx=.97,rely=.3)
	
	#KEEP THE WINDOW OPEN
	window.mainloop()

if __name__ == '__main__':
	main()