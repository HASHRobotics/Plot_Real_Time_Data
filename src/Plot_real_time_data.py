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
ani = 1

class plot_real_time:
	def __init__(self):
		self.range_values = []
		self.time_values = []	
		rospy.Subscriber("range", Range, self.Range_callback)
		ani = animation.FuncAnimation(fig,self.animate,frames = 1000,fargs=(self.range_values,self.time_values),interval = 50)
	
	# def call_sub(self):

	# 	def call_sub_sub():
	# 		rospy.Subscriber("range", Range, Range_callback)

	# 	# def Range_callback(distance):
	# 	# 	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	# 	# 	self.range_values.append(distance.range)
	# 	# 	self.time_values.append(distance.header.stamp)
	# 	# 	anim = animation.FuncAnimation(fig,animate,fargs=(self.range_values,self.time_values),interval = 500)

	# 	# def animate(frames,range_values,time_values):
	# 	# 	ax1.clear()	
	# 	# 	ax1.plot(time_values,range_values)

	def Range_callback(self,distance):
		global fig
		global ani
		#a = random.randint(1,101)
		self.range_values.append(distance.range)
		self.time_values.append(distance.header.stamp)
		# plt.plot(self.range_values, self.range_values, '*')
        # plt.axis("equal")
        # plt.draw()
        # plt.pause(0.00000000001)
		# plt.show()
		rospy.loginfo(self.range_values)
		#ani = animation.FuncAnimation(fig,animate,frames = len(self.range_values),fargs=(self.range_values,self.time_values),interval = 50)
		#ani = self.create_animation(self.range_values)
	
	# def create_animation(self,range_values):
	# 	ani = animation.FuncAnimation(fig,animate,frames = len(range_values),fargs=(range_values,),interval = 50)
	# 	return ani
	def animate(self, frames,range_values,time_values):
		global ax1
		if(frames == 0):
			ax1.clear()
		rospy.loginfo("In Animate \n")	
		rospy.loginfo(frames)	
		# ax1.plot(time_values[frames],range_values[frames])
		ax1.plot(time_values, range_values)

if __name__ == '__main__':
	rospy.init_node('range_plot', anonymous=True)
	prt = plot_real_time()
	rate = rospy.Rate(10)
	rospy.loginfo("In Main \n")
	while not rospy.is_shutdown():
		plt.show()
		rate.sleep()




