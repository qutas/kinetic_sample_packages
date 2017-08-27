import os
import math
import time

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

class Spinner():
	def __init__(self):
		# Setup for the timer
		self.timer = rospy.Timer(rospy.Duration(1.0), self.callback_ping)
		self.pub_ping = rospy.Publisher('/ping', Empty, queue_size=10)

		# Setup for the data converter
		self.sub_transform = rospy.Subscriber("/transform", TransformStamped, self.callback_transform)
		self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)

	def shutdown(self):
		# Unregister timers and subscribers if shutting down
		self.sub_ping.unregister()
		self.timer.shutdown()

	def callback_ping(self, timer_ev):
		# Create a new message and publish it
		msg_out = Empty()
		self.pub_ping.publish(msg_out)

	def callback_transform(self, msg_in):
		# Create a new message
		msg_out = PoseStamped()

		# Copy in the corresponding data
		msg_out.header = msg_in.header
		msg_out.pose.position.x = msg_in.transform.translation.x
		msg_out.pose.position.y = msg_in.transform.translation.y
		msg_out.pose.position.z = msg_in.transform.translation.z
		msg_out.pose.orientation.x = msg_in.transform.rotation.x
		msg_out.pose.orientation.y = msg_in.transform.rotation.y
		msg_out.pose.orientation.z = msg_in.transform.rotation.z
		msg_out.pose.orientation.w = msg_in.transform.rotation.w

		# Publish the converted message
		self.pub_pose.publish(msg_out)



