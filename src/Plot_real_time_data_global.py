#!/usr/bin/env python
import rospy
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import style
from sensor_msgs.msg import Range
import pdb
'''
This node subscribes to the topic 'range', gets it's values from . This data is then plotted real time 
'''

fig,ax1 = plt.subplots(1)
#ani = 1
range_values = []
time_values = []

def Range_callback(distance):
	global range_values
	global time_values
	range_values.append(distance.range)
	#rospy.loginfo(__dict__.distance.header.stamp)
	time_values.append(distance.header.stamp.secs)
	#time_values.append(distance.header.stamp)
	#rospy.loginfo(time_values)

def animate(frames):
	global ax1
	global range_values
	global time_values
	rospy.loginfo("In Animate \n")	
	#rospy.loginfo(frames)
	if(len(range_values)>0):
		#rospy.loginfo(type(range_values[0]))
		#rospy.loginfo(time_values[frames])	
		#x_axis = np.arange(len(range_values))
		ax1.plot(time_values, range_values)

if __name__ == '__main__':
	rospy.init_node('range_plot', anonymous=True)
	rospy.Subscriber("range", Range, Range_callback)
	rate = rospy.Rate(10)
	rospy.loginfo("In Main \n")
	ani = animation.FuncAnimation(fig,animate,frames = None,interval = 50)
	while not rospy.is_shutdown():
		plt.show()
		rate.sleep()




