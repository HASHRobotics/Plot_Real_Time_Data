#!/usr/bin/env python
import rospy
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import style
from sensor_msgs.msg import Range
from statistics import mean
import pdb
'''
This node subscribes to the topic 'range', gets it's values from . This data is then plotted real time 
'''

fig,ax1 = plt.subplots(1)
ax1.set_title(r'Range VS Time')
#ani = 1
range_values = []
time_values = []
max_value = -1

def Range_callback(distance):
	global range_values
	global time_values
	global max_value
	range_values.append(distance.range)
	if(distance.range>max_value):
		max_value = distance.range
	time_values.append(distance.header.stamp.secs)

def animate(frames):
	global ax1
	global range_values
	global time_values
	global max_value
	rospy.loginfo("In Animate \n")	
	if(len(range_values)>0):
		average = mean(range_values) #This can be optimised by keeping a running average rather than computing evry single time.
		string_to_display_on_graph = 'Mean: ' + str(round(average,2))  #Add appropriate units
		#rospy.loginfo(string_to_display_on_graph)
		ax1.clear()
		ax1.plot(time_values, range_values,color='blue')
		ax1.annotate(string_to_display_on_graph,xy=(0.5, 0.95), xycoords="axes fraction")

if __name__ == '__main__':
	rospy.init_node('range_plot', anonymous=True)
	rospy.Subscriber("range", Range, Range_callback)
	rate = rospy.Rate(10)
	rospy.loginfo("In Main \n")
	ani = animation.FuncAnimation(fig,animate,frames = None,interval = 50)
	while not rospy.is_shutdown():
		plt.xlabel('time(s)')
		plt.ylabel('Range(m)')
		plt.show()
		rate.sleep()




