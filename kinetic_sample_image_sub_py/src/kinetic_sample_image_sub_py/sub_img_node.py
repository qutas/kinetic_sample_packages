#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SubImg():
	def __init__(self):
		#Set up the CV Bridge
		self.bridge = CvBridge()

		# Set up the subscriber
		self.sub_img = rospy.Subscriber('/cv_camera/image_raw', Image, self.callback_img)
		self.pub_img = rospy.Publisher('/processed_img', Image, queue_size=10)

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_img.unregister()

	def callback_img(self, msg_in):
		#Convert ROS image to CV image
		try:
			cv_image = self.bridge.imgmsg_to_cv2( msg_in, "bgr8" )
		except CvBridgeError as e:
			print(e)

		# ===================
		# Do processing here!
		# ===================
		(rows,cols,channels) = cv_image.shape
		if cols > 70 and rows > 70 :
			# Draw circle at position (50,50), with diameter (10), bgr value (0,0,255), and thickness (2)
			cv2.circle(cv_image, (50,50), 10, (0, 0, 255), 2)
		# ===================

		#Convert CV image to ROS image and publish
		try:
			self.pub_img.publish( self.bridge.cv2_to_imgmsg( cv_image, "bgr8" ) )
		except CvBridgeError as e:
			print(e)
