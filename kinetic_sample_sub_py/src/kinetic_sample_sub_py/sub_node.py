import os
import math
import time

import rospy
from std_msgs.msg import Empty

class Spinner():
	def __init__(self):
		self.counter = 0

		# Set up the subscriber
		self.sub_ping = rospy.Subscriber("/sample_ping", Empty, self.callback_ping)

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_ping.unregister()

	def callback_ping(self, msg_in):
		self.counter += 1
		rospy.loginfo("Got ping #%i!" % self.counter)



