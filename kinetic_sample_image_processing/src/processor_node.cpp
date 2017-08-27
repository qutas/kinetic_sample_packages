#include <ros/ros.h>
#include <processor/processor.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_processor");
	Processor ip;

	ros::spin();

	return 0;
}
