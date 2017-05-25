#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <string>

class Pulser {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_ping_;

		double rate_ping_;
		std::string topic_output_ping_;

	public:
		Pulser() :
			nh_( ros::this_node::getName() ),
			rate_ping_( 10.0f ),
			topic_output_ping_( "ping_sample" ) {

			//Get parameters, or if not defined, use the defaults
			nh_.param( "rate_ping", rate_ping_, rate_ping_ );
			nh_.param( "topic_output_ping", topic_output_ping_, topic_output_ping_ );

			pub_ping_ = nh_.advertise<std_msgs::Empty>( topic_output_ping_, 10 );

			ROS_INFO("Pulsing...");
		}

		~Pulser() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}

		double get_rate( void ) {
			return rate_ping_;
		}

		void step( void ) {
			std_msgs::Empty msg_out;

			pub_ping_.publish( msg_out );
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "pulser_cpp");
	Pulser pl;

	ros::Rate loopRate( pl.get_rate() );

	while( ros::ok() ) {
		pl.step();

		ros::spinOnce();	//Not needed here, but will update subscribers if they are added
		loopRate.sleep();
	}

	return 0;
}
