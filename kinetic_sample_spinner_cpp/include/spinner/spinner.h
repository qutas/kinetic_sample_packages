#pragma once

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

class Spinner {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;
		ros::Publisher pub_ping_;
		ros::Subscriber sub_transform_;
		ros::Publisher pub_pose_;

	public:
		Spinner( void );

		~Spinner( void );

		void callback_ping(const ros::TimerEvent& e);
		void callback_transform(const geometry_msgs::TransformStamped::ConstPtr& msg_in);
};
