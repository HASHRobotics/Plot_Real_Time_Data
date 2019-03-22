#!/usr/bin/env python
import rospy
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import style
from sensor_msgs.msg import Range

'''
This node subscribes to the topic 'range', gets it's values from . This data is then plotted real time 
'''

fig = plt.figure()
fig,ax1 = plt.subplots(1)

class plot_real_time:
	def __init__(self):
		self.range_values = []
		self.time_values = []	
	
	def call_sub(self):

		def call_sub_sub():
			rospy.Subscriber("range", Range, Range_callback)

		# def Range_callback(distance):
		# 	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
		# 	self.range_values.append(distance.range)
		# 	self.time_values.append(distance.header.stamp)
		# 	anim = animation.FuncAnimation(fig,animate,fargs=(self.range_values,self.time_values),interval = 500)

		# def animate(frames,range_values,time_values):
		# 	ax1.clear()
		# 	ax1.plot(time_values,range_values)

		def Range_callback(distance):
			a = random.randint(1,101)
			self.range_values.append(a)
			anim = animation.FuncAnimation(fig,animate,fargs=(self.range_values),interval = 500)

		def animate(frames,range_values):
			ax1.clear()
			ax1.plot(frames,range_values)

		call_sub_sub()
		plt.show()

if __name__ == '__main__':
	rospy.init_node('range_plot', anonymous=True)
	prt = plot_real_time()
	while not rospy.is_shutdown():
		prt.call_sub()
		rospy.spin()



