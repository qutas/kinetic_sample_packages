#include <ros/ros.h>

#include <db_conn_cpp/db_conn.h>

//Need to make some defines to ensure they don't conflict with OpenCV
#define int64 mysql_int64
#define uint64 mysql_uint64
#include <mysql.h>
#include <my_global.h>
#undef int64
#undef uint64
//http://www.sigverse.org/wiki/en/index.php?mysql%20connection%20using%20ROS

#include <std_msgs/Int32.h>

DBConn::DBConn() :
	nh_("~"),
	conn_ptr_(mysql_init(NULL)) {

	sub_ = nh_.subscribe<std_msgs::Int32>( "/emulator/sensors/gas/b", 10, &DBConn::callback, this );
}

DBConn::~DBConn() {
	if( conn_ptr_ ) {
		mysql_close( conn_ptr_ );
	}
}

bool DBConn::init_connection() {
	bool success = false;

	if ( conn_ptr_ ) {
		conn_ptr_ = mysql_real_connect( conn_ptr_,
										"localhost",
										"user",
										"mysql",
										"ros_db",
										0, NULL, 0);

		if (conn_ptr_) {
			success = true;
		}
	} else {
		ROS_INFO("MySQL Init Failed");
	}

	return success;
}

void DBConn::callback(const std_msgs::Int32 msg_in) {
	ROS_INFO("Inserting new data...");

	//Get a timestamp to use in the database (in seconds from Unix Epoch)
	double timestamp = ros::Time::now().toSec();
	uint32_t reading = msg_in.data;

	std::string query = "INSERT INTO gas_data (timestamp, reading) VALUES ("
						+ std::to_string(timestamp)
						+ ", "
						+ std::to_string(reading)
						+ ")";

	mysql_query( conn_ptr_, query.c_str() );

}
