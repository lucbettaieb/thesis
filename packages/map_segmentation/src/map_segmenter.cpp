/*
 * map_segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>
#include <string>
#include <pluginlib/class_loader.h>

#include <map_segmentation/segmenter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_segmenter");
  ros::NodeHandle nh;

  // Make this a parameter
  std::string plugin_type = "simple_segmenter";

  pluginlib::ClassLoader<Segmenter> segmenter_loader("map_segmenter", "Segmenter");

  std::shared_ptr<Segmenter> segmenter;
  try
  {
    segmenter = segmenter_loader.createInstance(plugin_type);
  }
  catch(pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("Failed to load the segmenter for some reason. Error: %s", ex.what());
  }

  ROS_INFO("Loaded segmenter, going into spin.");

  // Make this a parameter
  ros::Rate naptime(15.0);
  while (ros::ok())
  {
    segmenter->segment();

    naptime.sleep();
    ros::spinOnce();
  }
}
