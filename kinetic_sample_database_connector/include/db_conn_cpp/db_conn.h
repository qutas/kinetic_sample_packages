#pragma once

#include <ros/ros.h>

#include <std_msgs/Int32.h>

//Need to make some defines to ensure they don't conflict with OpenCV
#define int64 mysql_int64
#define uint64 mysql_uint64
#include <mysql.h>
#include <my_global.h>
#undef int64
#undef uint64
//http://www.sigverse.org/wiki/en/index.php?mysql%20connection%20using%20ROS

class DBConn {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_;

		MYSQL *conn_ptr_;

	public:
		DBConn( void );

		~DBConn( void );

		bool init_connection();

		void callback(const std_msgs::Int32 msg_in);
};
