#include "ros/ros.h"
#include "std_msgs/String.h"

#include "car_controller/car_update.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_controller");
  ros::NodeHandle n;
  ros::Publisher car_raw_pub = n.advertise<car_controller::car_update>("car_raw", 1000);
  ros::Rate loop_rate(100);
  int count = 0;
  car_controller::car_update msg;
  while (ros::ok())
  {
    msg.header.stamp = ros::Time::now();
    msg.us = count;

    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.mode = ss.str();
    car_raw_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
