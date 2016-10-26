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
  // Make sure there is a map
  // if (Segmenter.map_ == NULL)
  // {
  //   ROS_ERROR("Map was not populated for some reason");
  //   return;
  // }

  // if (Segmenter.segmented_map_ != NULL)
  // {
  //   ROS_ERROR("Segmented map already populated.");
  //   return;
  // }
  // Here, populated the segmented map in a handy dandy way.
}

}  // namespace segmenter_plugins
