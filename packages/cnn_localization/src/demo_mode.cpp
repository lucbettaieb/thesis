/*
 * demo_mode
 * demo mode for CNN localization
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <tuple>

#include <ros/ros.h>
#include <cnn_localization/cnn_localizer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_mode");
  ros::NodeHandle nh;

  CNNLocalizer l(nh);

  // std::shared_ptr<CNNLocalizer> localizer;
  // localizer.reset(new CNNLocalizer(nh));

  ros::Rate naptime(10);
  while (ros::ok())
  {
    std::tuple<std::string, double> result;

  	result = l.runImage();

  	std::cout << "Result: " << std::get<0>(result) << ", " << std::get<1>(result) << std::endl;
  	
  	naptime.sleep();
  	ros::spinOnce();
  }

  return 1;
}
