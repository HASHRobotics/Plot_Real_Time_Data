#!/usr/bin/env python
import rospy
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import style
from sensor_msgs.msg import Range
from statistics import mean
from nav_msgs.msg import Odometry
import pdb

fig,(ax1,ax2,ax3) = plt.subplots(3,1)
#ax1.set_title(r'Range VS Time')
#ani = 1
range_values = []
range_count = 0 	#to see how any entries range values has and then plot only when new values come
bearing_values = []
bearing_count = 0	#to see how any entries bearing values has and then plot only when new values come
time_values = []
odom_Y_values = []
odom_X_values = []
max_value = -1

def Range_callback(distance):
	global range_count
	global range_values
	global time_values 
	global max_value
	range_values.append(distance.range)
	if(distance.range>max_value):
		max_value = distance.range
	time_values.append(distance.header.stamp.secs)

def odom_callback(odom):
	global odom_Y_values
	global odom_X_values
	odom_Y_values.append((odom.pose.pose.position.y) + random.randint(1,10))
	odom_X_values.append((odom.pose.pose.position.x) + random.randint(1,20))

def animate(frames):
	global ax1
	global ax2
	global ax3
	global range_values
	global time_values
	global odom_Y_values
	global odom_X_values
	global bearing_values
	global max_value
	global range_count
	global bearing_count
	rospy.loginfo("In Animate \n")	
	if(len(range_values)>range_count):
		range_count+=1
		average = mean(range_values) #This can be optimised by keeping a running average rather than computing evry single time.
		string_to_display_on_graph = 'Mean: ' + str(round(average,2))  #Add appropriate units
		#rospy.loginfo(string_to_display_on_graph)
		ax1.clear()
		ax1.plot(time_values, range_values,color='blue')
		ax1.annotate(string_to_display_on_graph,xy=(0.5, 0.9), xycoords="axes fraction")
		#max value can also be plotted. max_value is being maintained

	if(len(odom_X_values)>0):
		#ax1.clear()
		rospy.loginfo("In Animate \n")
		ax3.plot(odom_X_values, odom_Y_values,color='red',label='Odometry')

def set_axis_labels():
	global ax1
	global ax2
	global ax3
	ax1.set_ylabel('Range')
	ax1.set_xlabel('Time')
	ax2.set_ylabel('Bearing')
	ax2.set_xlabel('Time')
	ax3.set_ylabel('Rover Position')
	ax3.set_xlabel('Time')
	# ax1.set_title(r'Range VS Time')
	# ax2.set_title(r'Bearing VS Time')
	# ax3.set_title(r'Rover Position VS Time')

if __name__ == '__main__':
	rospy.init_node('transform-frames', anonymous=True)
    # rover1_start_angle = rospy.get_param("rover1_start_angle")
    # rover2_start_angle = rospy.get_param("rover2_start_angle")
	rospy.Subscriber("range", Range, Range_callback)
	rospy.Subscriber("/ak1/odometry/filtered", Odometry, odom_callback)
	rover1_start_angle = rospy.get_param("/Real_time_Plotting/transform_frames/rover1_start_angle")
	rover2_start_angle = rospy.get_param("/Real_time_Plotting/transform_frames/rover2_start_angle")
	rate = rospy.Rate(10)
	rospy.loginfo("In Main \n")
	set_axis_labels()
	ani = animation.FuncAnimation(fig,animate,frames = None,interval = 50)
	while not rospy.is_shutdown():
		# plt.xlabel('time(s)')
		# plt.ylabel('Range(m)')
		plt.show()
		rate.sleep()




