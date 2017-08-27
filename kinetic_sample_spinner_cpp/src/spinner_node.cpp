#include <ros/ros.h>
#include <spinner/spinner.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "spinner_cpp");
	Spinner sp;

	ros::spin();

	return 0;
}
