#!/usr/bin/env python
import rospy
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import style
from nav_msgs.msg import Odometry
from statistics import mean
import pdb
'''
This node subscribes to the topic '/ak1/odometry/filtered', gets it's values from it. This data is then plotted real time 
'''

fig,ax1 = plt.subplots(1)
ax1.set_title(r'Y VS X')
#ani = 1
Y_values = []
X_values = []


def odom_callback(odom):
	global Y_values
	global X_values
	Y_values.append(odom.pose.pose.position.y)
	X_values.append(odom.pose.pose.position.x)
	rospy.loginfo(X_values)

def animate(frames):
	global ax1
	global Y_values
	global X_values
	#rospy.loginfo("In Animate \n")	
	if(len(X_values)>0):
		#ax1.clear()
		rospy.loginfo("In Animate \n")
		ax1.plot(X_values, Y_values,color='blue')
		#max value can also be plotted. max_value is being maintained

if __name__ == '__main__':
	rospy.init_node('odom_plot', anonymous=True)
	rospy.Subscriber("/ak1/odometry/filtered", Odometry, odom_callback)
	rate = rospy.Rate(10)
	rospy.loginfo("In Main \n")
	ani = animation.FuncAnimation(fig,animate,frames = None,interval = 50)
	while not rospy.is_shutdown():
		plt.xlabel('x(m)')
		plt.ylabel('y(m)')
		plt.show()
		rate.sleep()




