#include <ros/ros.h>

#include <spinner/spinner.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

Spinner::Spinner() :
	nh_("~") {

	timer_ = nh_.createTimer(ros::Duration(1.0), &Spinner::callback_ping, this );
	pub_ping_ = nh_.advertise<std_msgs::Empty>("/ping", 10);

	sub_transform_ = nh_.subscribe<geometry_msgs::TransformStamped>( "/transform", 10, &Spinner::callback_transform, this );
	pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose", 10);
}

Spinner::~Spinner() {
}

void Spinner::callback_ping(const ros::TimerEvent& e) {
	std_msgs::Empty msg_out;
	pub_ping_.publish(msg_out);
}

void Spinner::callback_transform(const geometry_msgs::TransformStamped::ConstPtr& msg_in) {
	geometry_msgs::PoseStamped msg_out;

	msg_out.header = msg_in->header;
	msg_out.pose.position.x = msg_in->transform.translation.x;
	msg_out.pose.position.y = msg_in->transform.translation.y;
	msg_out.pose.position.z = msg_in->transform.translation.z;
	msg_out.pose.orientation.x = msg_in->transform.rotation.x;
	msg_out.pose.orientation.y = msg_in->transform.rotation.y;
	msg_out.pose.orientation.z = msg_in->transform.rotation.z;
	msg_out.pose.orientation.w = msg_in->transform.rotation.w;

	pub_pose_.publish(msg_out);
}
