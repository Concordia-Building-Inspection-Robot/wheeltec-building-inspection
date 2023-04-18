#include <ros/ros.h>

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "cpp_monitor");
	ros::NodeHandle nh;

	ROS_INFO("HELLO WORLD");

	ros::spin();

	return 0;
}
