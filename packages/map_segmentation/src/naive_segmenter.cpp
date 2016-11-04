/*
 * naive_segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */
#include <map_segmentation/naive_segmenter.h>
#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(segmenter_plugins::NaiveSegmenter, Segmenter)

namespace segmenter_plugins
{

NaiveSegmenter::NaiveSegmenter()
{
}

NaiveSegmenter::~NaiveSegmenter()
{
}

void NaiveSegmenter::initialize(ros::NodeHandle nh)
{
  ROS_INFO("NaiveSegmenter initialized.");

  Segmenter::initialize(nh);
}

bool NaiveSegmenter::segment()
{
  // Make sure there is a map...
  if (Segmenter::map_.data.size() == 0)
  {
    ROS_ERROR("Map not yet populated");
    return false;
  }

  uint region_resolution = 30;  // TODO(lucbettaieb): parameterize this constant

  nav_msgs::OccupancyGrid segmented_map = Segmenter::map_;

  // Resize
  std::vector< std::vector<int> > map_vec;

  // Create region vector
  std::vector<Region> region_vec;

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
      map_vec.at(i).at(j) = static_cast<int>(Segmenter::map_.data[iter]);

      iter++;
    }
  }

  iter = 0;
  for (uint i = 0; i < segmented_map.info.height; i++)
  {
    for (uint j = 0; j < segmented_map.info.width; j++)
    {
      //  Every 20 pixels...
      if (j % region_resolution == 0 || i % region_resolution == 0)
      {
        // Get corner points of regions
        if (j % region_resolution == 0 && i % region_resolution == 0)
        {
          if (j + region_resolution <= segmented_map.info.width &&
              i + region_resolution <= segmented_map.info.height)
          {
            if ((map_vec.at(i).at(j) != -1 && map_vec.at(i).at(j) != 99) ||
                (map_vec.at(i + region_resolution).at(j) != -1 && map_vec.at(i + region_resolution).at(j) != 99) ||
                (map_vec.at(i).at(j + region_resolution) != -1 && map_vec.at(i).at(j + region_resolution) != 99) ||
                (map_vec.at(i + region_resolution).at(j + region_resolution) != -1 && map_vec.at(i + region_resolution).at(j + region_resolution) != 99))
            {
              Region r;
              r.id = std::to_string(i) + std::to_string(j);

              r.top_left.x = j;
              r.top_left.y = i;

              r.top_right.x = j + region_resolution;
              r.top_right.y = i;

              r.bottom_left.x = j;
              r.bottom_left.y = i + region_resolution;

              r.bottom_right.x = j + region_resolution;
              r.bottom_right.y = i + region_resolution;

              region_vec.push_back(r);

              map_vec.at(i).at(j) = 99;
              map_vec.at(i + region_resolution).at(j) = 99;
              map_vec.at(i).at(j + region_resolution) = 99;
              map_vec.at(i + region_resolution).at(j + region_resolution) = 99;
            }
          }
        }
        // Region border points
        else
        {
         // map_vec.at(i).at(j) = 50;
        }
      }
      // Copy the map_vec over
      segmented_map.data[iter] = map_vec.at(i).at(j);
      iter++;
    }
  }

  region_vector_ = region_vec;
  segmented_map_pub_.publish(segmented_map);
  return true;
}

void NaiveSegmenter::getRegions(std::vector<Region> &vec)
{
  vec = region_vector_;
}

}  // namespace segmenter_plugins
