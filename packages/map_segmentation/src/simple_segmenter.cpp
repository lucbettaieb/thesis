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
  if (Segmenter::map_.data.size() == 0)
  {
    ROS_ERROR("Map not yet populated");
    return;
  }
  // First copy over the segmented map...

  nav_msgs::OccupancyGrid segmented_map = Segmenter::map_;
  // segmented_map.data.resize(Segmenter::map_.info.height*Segmenter::map_.info.width);

  // Now, let's try to alter the segmented map's state

  for (uint i = 0; i < segmented_map.data.size(); i++)
  {
    segmented_map.data[i] = 100;
  }

  segmented_map_pub_.publish(segmented_map);

  // Publish the segmented map
}

}  // namespace segmenter_plugins
