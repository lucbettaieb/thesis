/*
 * map_segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>
#include <string>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>
#include <map_segmentation/segmenter.h>

/*
 * Service callback to give the appropriate region (string) based on a pose
 */
// bool getRegion(map_segmentation::GetRegion::Request &req,
//                map_segmentation::GetRegion::Response &res)
// {
  
// }

// This should advertise a ROS service that, given a position, returns a region for a label
int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_segmenter");
  ros::NodeHandle nh;

  // Make this a parameter
  std::string plugin_type = "segmenter_plugins::NaiveSegmenter";

  pluginlib::ClassLoader<Segmenter> segmenter_loader("map_segmentation", "Segmenter");

  try
  {
    boost::shared_ptr<Segmenter> segmenter;

    segmenter = segmenter_loader.createInstance(plugin_type);
    segmenter->initialize(nh);

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
  catch(pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("Failed to load the segmenter for some reason. Error: %s", ex.what());
  }
}
