/*
 * demo_mode
 * demo mode for CNN localization
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <tuple>

#include <ros/ros.h>
#include <cnn_localization/tensorflow_ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_mode");
  ros::NodeHandle nh;

  CNNLocalizer l(nh);

  // std::shared_ptr<CNNLocalizer> localizer;
  // localizer.reset(new CNNLocalizer(nh));

  ros::Rate naptime(10);
  std::tuple<std::string, double> result;
  while (ros::ok())
  {
    // ROS_INFO("dm run image");
  	result = l.runImage();

  	std::cout << "Result: " << std::get<0>(result) << ", " << std::get<1>(result) << std::endl;
  	
  	naptime.sleep();
  	ros::spinOnce();
  }

  return 1;
}
