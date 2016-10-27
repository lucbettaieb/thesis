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
  ROS_INFO("SimpleSegmenter initialized.");

  Segmenter::initialize(nh);
}

void SimpleSegmenter::segment()
{
  // Make sure there is a map...
  if (Segmenter::map_.data.size() == 0)
  {
    ROS_ERROR("Map not yet populated");
    return;
  }

  nav_msgs::OccupancyGrid segmented_map = Segmenter::map_;

  // Resize
  std::vector< std::vector<int> > map_vec;
  
  // Resize map tuple vector
  map_vec.resize(Segmenter::map_.info.height);
  for (uint i = 0; i < Segmenter::map_.info.height; i++)
  {
    map_vec.at(i).resize(Segmenter::map_.info.width);
  }

  uint iter = 0;
  for (uint i = 0; i < segmented_map.info.height; i++)
  {
    for (uint j = 0; j < segmented_map.info.width; j++)
    {
      map_vec.at(i).at(j) = Segmenter::map_.data[iter];

      segmented_map.data[iter] = 0;
      iter++;
    }
  }

  iter = 0;
  for (uint i = 0; i < segmented_map.info.height; i++)
  {
    for (uint j = 0; j < segmented_map.info.width; j++)
    {
      if ( (map_vec.at(i).at(j) >= 90) 
          && 
              ( 
                ((map_vec.at(i+1).at(j) >= 90) && (map_vec.at(i-1).at(j) == 0)  && (map_vec.at(i).at(j+1) == 0) && (map_vec.at(i).at(j-1) == 0)) 
                // ||
                // ((map_vec.at(i+1).at(j) == 0) && (map_vec.at(i-1).at(j) == 100)  && (map_vec.at(i).at(j+1) == 0) && (map_vec.at(i).at(j-1) == 0)) ||
                // ((map_vec.at(i+1).at(j) == 0) && (map_vec.at(i-1).at(j) == 0)  && (map_vec.at(i).at(j+1) == 100) && (map_vec.at(i).at(j-1) == 0)) ||
                // ((map_vec.at(i+1).at(j) == 0) && (map_vec.at(i-1).at(j) == 0)  && (map_vec.at(i).at(j+1) == 0) && (map_vec.at(i).at(j-1) == 100))
              )
          )
      {
        ROS_INFO("Found one");
        segmented_map.data[iter] = 100;
      }

      iter++;
    }
  }

  segmented_map_pub_.publish(segmented_map);
}

}  // namespace segmenter_plugins
