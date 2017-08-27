#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>

class MavrosGuider {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_pose_;
		ros::Subscriber sub_state_;
		ros::ServiceClient client_arming_;
		ros::ServiceClient client_set_mode_;
		ros::Timer timer_pose_out_;

		geometry_msgs::PoseStamped msg_pose_out_;
		mavros_msgs::State msg_current_state_;
		mavros_msgs::SetMode msg_set_mode_;
		mavros_msgs::CommandBool msg_arm_cmd_;

		std::string topic_output_pose_;
		double rate_timer_;

	public:
		MavrosGuider();

		~MavrosGuider();

		void state_cb( const mavros_msgs::State::ConstPtr& msg_in );
		void timer_cb( const ros::TimerEvent& t_e );
};
