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
from bearing_estimator.msg import bearing_msg
import pdb
from math import cos,sin,radians

'''
Notes to self:
1) The time for range,bearing and odom is kept same as of now. It should be kept separate as range & bearing are not continous. 
2) Note: A different time for range and bearing is maintained, since it's quite possible that bearing might not get captured or range might not get captured sometimes.

'''

fig,(ax1,ax2,ax3) = plt.subplots(3,1)
#ax1.set_title(r'Range VS Time')
#ani = 1
range_values = []
range_count = 0 	#to see how any entries range values has and then plot only when new values come
bearing_values = []
bearing_count = 0	#to see how any entries bearing values has and then plot only when new values come
range_time_values = []
bearing_time_values = []
odom_Y_values = []
odom_X_values = []
max_value = -1
H_rover12rtk = np.identity(3)
H_rover22rtk = np.identity(3)

def Range_callback(distance):
	global range_values
	global range_time_values 
	global max_value
	range_values.append(distance.range + random.randint(1,10))
	if(distance.range>max_value):
		max_value = distance.range
	range_time_values.append(distance.header.stamp.secs)

def Bearing_callback(bearing):
	global bearing_values
	global bearing_time_values
	bearing_values.append(bearing.bearing)
	bearing_time_values.append(bearing.header.stamp.secs)


def odom_callback(odom):
	global odom_Y_values
	global odom_X_values
	odom_Y_values.append((odom.pose.pose.position.y) + random.randint(1,10))
	odom_X_values.append(odom.pose.pose.position.x) 


# def rtk_callback(rtk):
# 	# global odom_Y_values
# 	# global odom_X_values
	# global H_rover12rtk
	# global H_rover22rtk
# 	rtk.counter += 1
	# if(rtk.counter == 1):
	# 	H_rover12rtk = getH(rtk1_x,rtk1_y,1)
	# 	H_rover22rtk = getH(rtk2_x,rtk2_y,2)   #Simply pass the rtk1 and rtk2 values
		
# rtk.counter = 0		#Don't delete. This is for static int based method of RTK

def animate(frames):
	global ax1
	global ax2
	global ax3
	global range_values
	global range_time_values
	global bearing_time_values
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
		rospy.loginfo(time_values)
		#rospy.loginfo(range_count)
		ax1.clear()
		ax1.scatter(range_time_values, range_values,color='blue')
		ax1.annotate(string_to_display_on_graph,xy=(0.5, 0.9), xycoords="axes fraction")
		#max value can also be plotted. max_value is being maintained

	if(len(bearing_values)>bearing_count):
		bearing_count+=1
		rospy.loginfo("In bearing plot!")
		ax2.clear()
		ax2.scatter(bearing_time_values, bearing_values,color='green')

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
	ax3.set_ylabel('Rover Position in Y axis')
	ax3.set_xlabel('Rover Position in X axis')
	# ax1.set_title(r'Range VS Time')
	# ax2.set_title(r'Bearing VS Time')
	# ax3.set_title(r'Rover Position VS Time')

def getH(rtk_x,rtk_y,rover,odom_x = 0,odom_y = 0):
	if(rover==1):
		theta = radians(double(rospy.get_param("/Real_time_Plotting/transform_frames/rover1_start_angle")))
	if(rover==2):
		theta = radians(double(rospy.get_param("/Real_time_Plotting/transform_frames/rover2_start_angle")))
	tx = rtk_x - odom_x*cos(theta) + odom_y*sin(theta) 
	ty = rtk_y - odom_x*sin(theta) - odom_y*cos(theta)
	H = np.array([[cos(theta),-sin(theta),tx],[sin(theta),cos(theta),ty],[0,0,1]])
	return H 

if __name__ == '__main__':
	rospy.init_node('transform-frames', anonymous=True)
	rospy.Subscriber("/range", Range, Range_callback)
	#rospy.Subscriber("/ak1/odometry/filtered", Odometry, odom_callback)
	rospy.Subscriber("/odom1", Odometry, odom_callback)
	rospy.Subscriber("/bearing", bearing_msg, Bearing_callback)
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




