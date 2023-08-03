#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <string>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "follower");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    geometry_msgs::Twist msg;
    ros::Rate loop_rate(1);
    int counter = 0;
    while(ros::ok() && counter != 2)
    {
        msg.linear.x = std::stod(argv[1]);
        msg.linear.y = std::stod(argv[2]);
        msg.linear.z = std::stod(argv[3]);
        msg.angular.x = std::stod(argv[4]);
        msg.angular.y = std::stod(argv[5]);
        msg.angular.z = std::stod(argv[6]);

        pub.publish(msg);
        loop_rate.sleep();
        counter++;
    }
    return 0;
}
