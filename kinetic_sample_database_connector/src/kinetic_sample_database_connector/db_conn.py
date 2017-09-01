import os
import math
import time
import mysql.connector

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

class DBConn():
	def __init__(self):
		# Setup the subscriber
		self.sub = rospy.Subscriber("/emulator/sensors/gas/b", Int32, self.callback)

		# Setup the database connector
		config = {
					'user': 'user',
					'password': 'mysql',
					'host': 'localhost',
					'database': 'ros_db',
					'raise_on_warnings': True,
					}

		self.cnx = mysql.connector.connect(
											host="localhost",    # your host, usually localhost
											user="user",         # your username
											passwd="mysql",		# your password
											db="ros_db"        # name of the data base
											)

		self.cursor = self.cnx.cursor()

		# Prepare the query to use when adding new data
		self.add_reading = (
							"INSERT INTO gas_data "
							"(timestamp, reading) "
							"VALUES (%s, %s)"
							)

		rospy.loginfo("Database connected!")

	def shutdown(self):
		# Unregister timers and subscribers if shutting down
		self.sub.unregister()
		self.cnx.close()

	def callback(self, msg_in):
		rospy.loginfo("Inserting new data...");

		timestamp = rospy.get_time()
		reading = msg_in.data

		# Run the query and commit the data to the database
		self.cursor.execute(self.add_reading, (timestamp, reading))
		self.cnx.commit()
