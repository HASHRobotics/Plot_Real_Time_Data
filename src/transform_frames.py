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
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovariance
import pdb
from math import cos,sin,radians

'''
Notes to self:
1) The time for range,bearing and odom is kept same as of now. It should be kept separate as range & bearing are not continous.
2) Note: A different time for range and bearing is maintained, since it's quite possible that bearing might not get captured or range might not get captured sometimes.

'''

fig,(ax1,ax2,ax3) = plt.subplots(4,1)
#ax1.set_title(r'Range VS Time')
#ani = 1

range_values = []
range_count = 0 	#to see how any entries range values has and then plot only when new values come
range_time_values = []

ground_truth_range_values = []
ground_truth_range_count = 0
ground_truth_range_time_values = []

range_error = []
bearing_error = []

bearing_values = []
bearing_count = 0	#to see how any entries bearing values has and then plot only when new values come
bearing_time_values = []

ground_truth_bearing_values = []
ground_truth_bearing_count = 0
ground_truth_bearing_time_values = []

odom_Y1_values = []
odom_X1_values = []
odom_Y2_values = []
odom_X2_values = []
#max_value = -1

pose_array_x_1 = []
pose_array_y_1 = []
pose_array_x_2 = []
pose_array_y_2 = []

rtk_x_1 = []
rtk_y_1 = []
rtk_x_2 = []
rtk_y_2 = []

# H_rover12rtk = None
# H_rover22rtk = None
# H_rover12rtk = np.identity(3)
# H_rover22rtk = np.identity(3)


def range_callback(distance):
	global range_values
	global range_time_values
	#global max_value
	#global range_count #To be removed
	range_values.append(distance.range)
	# if(distance.range>max_value):
	# 	max_value = distance.range
	#range_time_values.append(range_count)  #To be removed
	range_time_values.append(distance.header.stamp.secs)   #To be uncommented

def ground_truth_range_callback(distance):
	global ground_truth_range_values
	global ground_truth_range_time_values
	ground_truth_range_values.append(distance.range) #Change to appropriate data-type
	ground_truth_range_time_values.append(distance.header.stamp.secs)   #To be uncommented

def bearing_callback(bearing):
	global bearing_values
	global bearing_time_values
	bearing_values.append(bearing.bearing)
	bearing_time_values.append(bearing.header.stamp.secs)

def ground_truth_bearing_callback(bearing):
	global grount_truth_bearing_values
	global ground_truth_bearing_time_values
	ground_truth_bearing_values.append(bearing.bearing)
	ground_truth_bearing_time_values.append(bearing.header.stamp.secs) 	#To be uncommented

def odom1_callback(odom):
	global odom_Y1_values
	global odom_X1_values
	odom_Y1_values.append(odom.pose.pose.position.x)
	odom_X1_values.append(odom.pose.pose.position.y)

	#global H_rover12rtk
	# if(H_rover12rtk is not None):
	# 	odom_pos = np.array([[odom.pose.pose.position.x],[odom.pose.pose.position.y],[1]])
	# 	odom_transformed = np.dot(H_rover12rtk,odom_pos)
	# 	odom_Y1_values.append(odom_transformed[1])
	# 	odom_X1_values.append(odom_transformed[0])

def odom2_callback(odom):
	global odom_Y2_values
	global odom_X2_values
	odom_Y2_values.append(odom.pose.pose.position.x)
	odom_X2_values.append(odom.pose.pose.position.y)
	# global H_rover22rtk
	# if(H_rover22rtk is not None):
	# 	odom_pos = np.array([[odom.pose.pose.position.x],[odom.pose.pose.position.y],[1]])
	# 	odom_transformed = np.dot(H_rover22rtk,odom_pos)
	# 	odom_Y2_values.append(odom_transformed[1])
	# 	odom_X2_values.append(odom_transformed[0])

def pose1_callback(msg):
	global pose_array_x_1
	global pose_array_y_1
	pose_array_x_1 = []
	pose_array_y_1 = []
	Poses = msg.poses
	for elt in Poses: #Since poses is an array
		pose_array_x_1.append(elt.position.x)
		pose_array_y_1.append(elt.position.y)

	# if(H_rover12rtk is not None):
	# 	Poses = msg.poses
	# 	for elt in Poses: #Since poses is an array
	# 		updated_pose = odom_pos = np.array([[odom.pose.pose.position.x],[odom.pose.pose.position.y],[1]])
	# 		pose_array_x_1.append(elt.position.x)
	# 		pose_array_y_1.append(elt.position.y)

def pose2_callback(msg):
	global pose_array_x_2
	global pose_array_y_2
	pose_array_x_2 = []
	pose_array_y_2 = []
	Poses = msg.poses
	for elt in Poses: #Since poses is an array
		pose_array_x_2.append(elt.position.x)
		pose_array_y_2.append(elt.position.y)

def rtk1_callback(msg): #Copy this for rtk2_callback
	global rtk_x_1
	global rtk_y_1
	rtk_x_1.append(msg.pose.position.x)
	rtk_y_1.append(msg.pose.position.y)

def rtk2_callback(msg): #Copy this for rtk2_callback
	global rtk_x_2
	global rtk_y_2
	rtk_x_2.append(msg.pose.position.x)
	rtk_y_2.append(msg.pose.position.y)

def rtk_callback(rtk):
	# global odom_Y_values
	# global odom_X_values
	global H_rover12rtk
	global H_rover22rtk
	rtk.counter += 1
	if(rtk.counter == 1):
		H_rover12rtk = getH(rtk1_x,rtk1_y,1)
		H_rover22rtk = getH(rtk2_x,rtk2_y,2)   #Simply pass the rtk1 and rtk2 values

rtk.counter = 0		#Don't delete. This is for static int based method of RTK

def animate(frames):
	global ax1
	global ax2
	global ax3

	global range_values
	global range_time_values
	global range_count

	global ground_truth_range_values
	global ground_truth_range_time_values
	global ground_truth_range_count

	global bearing_time_values
	global bearing_values
	global bearing_count

	global ground_truth_bearing_time_values
	global ground_truth_bearing_values
	global ground_truth_bearing_count

	global odom_Y1_values
	global odom_X1_values
	global odom_Y2_values
	global odom_X2_values

	global range_error
	global bearing_error

	global pose_array_x_1
	global pose_array_y_1
	global pose_array_x_2
	global pose_array_y_2

	global rtk_x_1
	global rtk_y_1
	global rtk_x_2
	global rtk_y_2

	#global max_value
	rospy.loginfo("In Animate \n")

	if(len(range_values)>range_count and len(ground_truth_range_values)>ground_truth_range_count):
		range_error.append(abs(range_values[-1]-ground_truth_range_values[-1]))
		average = mean(range_error) #This can be optimised by keeping a running average rather than computing evry single time.
		string_to_display_on_graph = 'Mean Error in Range: ' + str(round(average,2)) + ' m'  #Add appropriate units
		ax1.clear()
		ax1.annotate(string_to_display_on_graph,xy=(0.5, 0.9), xycoords="axes fraction")

	if(len(range_values)>range_count):
		range_count+=1
		ax1.scatter(range_time_values, range_values,color='blue')
		#ax2.set_yticks(np.arange(min(range_values),max(range_values)+1))
		#max value can also be plotted. max_value is being maintained

	if(len(ground_truth_range_values)>ground_truth_range_count):
		ground_truth_range_count+=1
		ax1.scatter(ground_truth_range_time_values, ground_truth_range_values,color='red')

	if(len(bearing_values)>bearing_count and len(ground_truth_bearing_values)>ground_truth_bearing_count):
		bearing_error.append(abs(bearing_values[-1]-ground_truth_bearing_values[-1]))
		average = mean(bearing_error) #This can be optimised by keeping a running average rather than computing evry single time.
		string_to_display_on_graph = 'Mean Error in Bearing: ' + str(round(average,2)) + ' m'  #Add appropriate units
		ax2.clear()
		ax2.annotate(string_to_display_on_graph,xy=(0.5, 0.9), xycoords="axes fraction")

	if(len(bearing_values)>bearing_count):
		bearing_count+=1
		ax2.scatter(bearing_time_values, bearing_values,color='blue')

	if(len(ground_truth_bearing_values)>ground_truth_bearing_count):
		ground_truth_bearing_count+=1
		ax2.scatter(ground_truth_bearing_time_values, ground_truth_bearing_values,color='red')

	if(len(odom_X1_values)>0 or len(rtk_x_1)>0): #Change it for or condition on odom data/RTK data
		ax3.clear()
		if(len(odom_X1_values)>0):
			ax3.plot(odom_X1_values, odom_Y1_values,color='green',label='Odometry')
		if(len(pose_array_x_1)>0):
			ax3.plot(pose_array_x_1, pose_array_y_1,color='black',label='Updated Pose')
		if(len(rtk_x_1)>0):
			ax3.plot(rtk_x_1, rtk_y_1,color='red',label='RTK')

	# if(len(odom_X2_values)>0 or len(rtk_x_2)>0): #Change it for or condition on odom data/RTK data
	# 	ax4.clear()
	# 	if(len(odom_X2_values)>0):
	# 		ax4.plot(odom_X2_values, odom_Y2_values,color='green',label='Odometry')
	# 	if(len(pose_array_x_2)>0):
	# 		ax4.plot(pose_array_x_2, pose_array_y_2,color='black',label='Updated Pose')
	# 	if(len(rtk_x_2)>0):
	# 		ax4.plot(rtk_x_2, rtk_y_2,color='red',label='RTK')



def set_axis_labels():
	global ax1
	global ax2
	global ax3
	#global ax4
	ax1.set_ylabel('Range')
	ax1.set_xlabel('Time')
	ax1.grid(linestyle='-', linewidth='0.3', color='red')
	ax2.set_ylabel('Bearing')
	ax2.set_xlabel('Time')
	ax2.grid(linestyle='-', linewidth='0.3', color='red')
	ax3.set_ylabel('Rover 1 Position in Y axis')
	ax3.set_xlabel('Rover 1 Position in X axis')
	#ax4.set_ylabel('Rover 2 Position in Y axis')
	#ax4.set_xlabel('Rover 2 Position in X axis')
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

	# Range
	rospy.Subscriber("/sampled_range", Range, range_callback)
	rospy.Subscriber("/true_range", Range, ground_truth_range_callback)

	# RTK GPS
	rospy.Subscriber("/ak1/piksi_multi/enu_pose_best_fix",PoseWithCovariance, rtk1_callback)
	# rospy.Subscriber("/ak2/piksi_multi/enu_pose_best_fix",PoseWithCovariance, rtk2_callback)

	# ODOMETRY
	rospy.Subscriber("/ak1/odometry/filtered", Odometry, odom1_callback)
	# rospy.Subscriber("/odom1", Odometry, odom_callback)

	# BEARING
	rospy.Subscriber("/bearing", bearing_msg, bearing_callback)
	rospy.Subscriber("/rtk_bearing", bearing_msg, ground_truth_bearing_callback)

	# Colocalized poses
	rospy.Subscriber("/ak1/pose1", PoseArray, pose1_callback)

	rover1_start_angle = rospy.get_param("/Real_time_Plotting/transform_frames/rover1_start_angle")
	rover2_start_angle = rospy.get_param("/Real_time_Plotting/transform_frames/rover2_start_angle")

	rate = rospy.Rate(10)
	rospy.loginfo("In Main \n")
	set_axis_labels()
	ani = animation.FuncAnimation(fig,animate,frames = None,interval = 50)
	while not rospy.is_shutdown():
		# plt.xlabel('time(s)')
		# plt.ylabel('Range(m)')
		#plt.tight_layout()
		plt.show()
		rate.sleep()




