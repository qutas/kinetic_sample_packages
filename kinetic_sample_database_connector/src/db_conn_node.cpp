#include <ros/ros.h>
#include <db_conn_cpp/db_conn.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "db_conn_cpp");
	DBConn dbc;

	if( dbc.init_connection() ) {
		ROS_INFO("Connection to database successful!");

		ros::spin();
	} else {
		ROS_INFO("Connection to database failed, shutting down.");
	}

	return 0;
}
