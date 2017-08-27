#include <ros/ros.h>
#include <guider/guider.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavros_guider_cpp");
	MavrosGuider guider;

	ros::spin();

	return 0;
}
