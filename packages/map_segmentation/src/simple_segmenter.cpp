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
  nav_msgs::OccupancyGrid segmented_map = Segmenter::map_;

  // Now, let's try to alter the segmented map's state
  size_t iter = 0;
  for (size_t i = 0; i < segmented_map.info.height; i++)
  {
    for (size_t j = 0; i < segmented_map.info.width; j++)
    {
      segmented_map.data[iter] = 100;
      iter++;
    }
  }
  segmented_map_pub_.publish(segmented_map);

  // Publish the segmented map
}

}  // namespace segmenter_plugins
