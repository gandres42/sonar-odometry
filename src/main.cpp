#include <ros/ros.h>
#include "sonar_odom.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  SonarOdom odom;
  ros::spin();
  return 0;
}

