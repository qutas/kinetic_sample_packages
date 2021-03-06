#include <ros/ros.h>
#include <guider/guider.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <string>

MavrosGuider::MavrosGuider() :
	nh_( "~" ),
	rate_timer_( 20.0 ),
	topic_output_pose_( "/sepoint_pose" ) {

	//Get parameters, or if not defined, use the defaults
	nh_.param( "topic_output_pose", topic_output_pose_, topic_output_pose_ );
	nh_.param( "rate_command_output", rate_timer_, rate_timer_ );

	sub_state_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MavrosGuider::state_cb, this);
	pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_output_pose_, 10 );
	client_arming_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	client_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

	msg_pose_out_.header.frame_id = "world";
	msg_pose_out_.pose.position.x = 0.0;
	msg_pose_out_.pose.position.y = 0.0;
	msg_pose_out_.pose.position.z = 1.5;
	msg_pose_out_.pose.orientation.x = 0.0;
	msg_pose_out_.pose.orientation.y = 0.0;
	msg_pose_out_.pose.orientation.z = 0.0;
	msg_pose_out_.pose.orientation.w = 1.0;

	timer_pose_out_ = nh_.createTimer(ros::Duration( 1 / rate_timer_ ), &MavrosGuider::timer_cb, this);

	msg_set_mode_.request.custom_mode = "OFFBOARD";
	msg_arm_cmd_.request.value = true;

	ROS_INFO("Waiting for FCU connection...");

	//Wait for FCU connection
	while( ros::ok() && !msg_current_state_.connected ){
		ros::spinOnce();
		ros::Rate(rate_timer_).sleep();
	}

	ROS_INFO("FCU detected!");
	ROS_INFO("Attempting to take control of UAV...");

	//Set up a stamp to keep track of requests, so we don't flood the FCU
    ros::Time last_request = ros::Time(0);

	//Wait for Armed and in OFFBOARD mode
	while( ros::ok() && ( ( msg_current_state_.mode != "OFFBOARD" ) || ( !msg_current_state_.armed ) ) ) {

		if( ( ros::Time::now() - last_request ) > ros::Duration(5.0) ) {
			if( msg_current_state_.mode != "OFFBOARD" ) {
				if( client_set_mode_.call(msg_set_mode_) && msg_set_mode_.response.mode_sent ) {
					ROS_INFO("Offboard mode enabled!");
				}
			} else if( !msg_current_state_.armed ) {
				if( client_arming_.call(msg_arm_cmd_) && msg_arm_cmd_.response.success) {
					ROS_INFO("UAV armed!");
				}
			}

			last_request = ros::Time::now();
		}

		ros::spinOnce();
		ros::Rate(rate_timer_).sleep();
	}

	ROS_INFO("Now controlling UAV!");
}

MavrosGuider::~MavrosGuider() {
	//This message won't actually send here, as the node will have already shut down
	ROS_INFO("Shutting down...");
}

void MavrosGuider::state_cb( const mavros_msgs::State::ConstPtr& msg_in ) {
	msg_current_state_ = *msg_in;
}

void MavrosGuider::timer_cb( const ros::TimerEvent& t_e ) {
	msg_pose_out_.header.stamp = ros::Time::now();
	pub_pose_.publish( msg_pose_out_ );
}
