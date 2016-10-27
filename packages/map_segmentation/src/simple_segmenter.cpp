/*
 * simple_segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */
#include <map_segmentation/simple_segmenter.h>
#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(segmenter_plugins::SimpleSegmenter, Segmenter)

namespace segmenter_plugins
{

SimpleSegmenter::SimpleSegmenter()
{
}

SimpleSegmenter::~SimpleSegmenter()
{
}

void SimpleSegmenter::initialize(ros::NodeHandle nh)
{
  Segmenter::initialize(nh);
}

void SimpleSegmenter::segment()
{
  ROS_INFO("SIMPLE SEGMENT");
  // Make sure there is a map...

  // First copy over the segmented map...
  nav_msgs::OccupancyGrid segmented_map_ = Segmenter::map_;

  // Now, let's try to alter the segmented map's state
  for (size_t i = 0; i < segmented_map_.info.width; i++)
  {}

  // Publish the segmented map
}

}  // namespace segmenter_plugins
