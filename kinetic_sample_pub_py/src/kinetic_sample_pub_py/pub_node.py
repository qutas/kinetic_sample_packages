import os
import math
import time

import rospy
from std_msgs.msg import Empty

class Pulser():
	def __init__(self):
		# Set up the publisher
		self.pub_ping = rospy.Publisher('/sample_ping', Empty, queue_size=10)

		# Another way to a constant-rate publisher
		# This method would not be needed to be called externally
		#   i.e. no need for pl.step() in the main
		#self.rate = 10.0 #Hz
		#self.timer_pulse = rospy.Timer(rospy.Duration(1.0 / self.rate), self.step)

	def shutdown_plugin(self):
		# Unregister anything that needs it here

		# If used, the timer should be shutdown here
		#self.timer_pulse.shutdown()

		pass

	def step(self):
		msg_out = Empty()

		self.pub_ping.publish(msg_out)



