#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#define MAXJERK 4.0

using namespace std;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "cpp_monitor");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<const std_msgs::String>("robot_handler_cmd", 10);

    double prevAcc{0};

    auto callback = [&] (const sensor_msgs::Imu::ConstPtr& data) {
        double x = data->linear_acceleration.x;
        double y = data->linear_acceleration.y;
        double acc = sqrt(pow(x, 2) + pow(y, 2));
        double jerk = acc - prevAcc;

        if (jerk > MAXJERK) {
            std_msgs::String msg;
            msg.data = "halt 1";
            pub.publish(msg);
        }
    };

    ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("imu", 1000, callback);

    ros::spin();

    return 0;
}
