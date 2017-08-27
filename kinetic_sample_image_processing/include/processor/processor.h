#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <string>

class Processor {
	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

		ros::Subscriber sub_camera_info_;
		image_transport::Subscriber sub_image_;
		image_transport::Publisher pub_debug_image_;
		image_transport::Publisher pub_overlay_image_;
		ros::Publisher pub_detect_;

		bool got_camera_info_;
		sensor_msgs::CameraInfo camera_info_;

		std::string topic_input_camera_info_;
		std::string topic_input_image_;
		std::string topic_output_debug_image_;
		std::string topic_output_overlay_image_;
		std::string topic_output_detect_;

	public:
		Processor();

		~Processor();

		void camera_info_cb(const sensor_msgs::CameraInfo::ConstPtr& msg_in);
		void image_cb( const sensor_msgs::Image::ConstPtr& msg_in );
};
