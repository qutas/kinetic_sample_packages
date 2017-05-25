#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <string>

class Spinner {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_ping_;

		uint64_t counter_;
		std::string topic_input_ping_;

	public:
		Spinner() :
			nh_( ros::this_node::getName() ),
			counter_( 0 ),
			topic_input_ping_( "ping_sample" ) {

			//Get parameters, or if not defined, use the defaults
			nh_.param( "topic_input_ping", topic_input_ping_, topic_input_ping_ );

			sub_ping_ = nh_.subscribe<std_msgs::Empty>( topic_input_ping_, 100, &Spinner::ping_cb, this );

			ROS_INFO("Spinning...");
		}

		~Spinner() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}

		void ping_cb( const std_msgs::Empty::ConstPtr& msg_in ) {
			//We subscribe to the const pointer here to avoid doing a copy of the data
			// but we must be careful, as messages may be pushed out of the buffer if we take too long

			//To access to the message data, for example we can use the syntax:
			//	ros::Time msg_rec_time = msg_in->header.stamp;

			counter_ += 1;
			ROS_INFO("Got ping #%lu!", counter_);
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "spinner_cpp");
	Spinner sp;

	ros::spin();

	return 0;
}
