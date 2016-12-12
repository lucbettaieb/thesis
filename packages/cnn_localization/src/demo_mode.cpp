/*
 * demo_mode
 * demo mode for CNN localization
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>
#include <cnn_localization/cnn_localizer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_mode");
  ros::NodeHandle nh;

  CNNLocalizer l(nh);

  // std::shared_ptr<CNNLocalizer> localizer;
  // localizer.reset(new CNNLocalizer(nh));

  ROS_INFO("This does nothing.");

  return 1;
}
