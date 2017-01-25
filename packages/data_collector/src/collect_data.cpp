/*
 * collect_data
 *
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>

#include <data_collector/data_collector.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collect_data");
  ros::NodeHandle nh;

  std::shared_ptr<DataCollector> collector;

  // Load in sampling frequency from the parameter server
  int sampling_frequency;
  nh.param<int>("data_collector/sampling_frequency", sampling_frequency, 5);
  ros::Rate naptime(sampling_frequency);

  while (ros::ok() && collector->collectData())
  {
    ros::spinOnce();
    naptime.sleep();
  }
}
