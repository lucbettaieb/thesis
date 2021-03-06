/*
 * segmenter
 * 
 * Segmenter base class.  Takes in a map and returns a set of points 
 *
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <map_segmentation/segmenter.h>
#include <boost/bind.hpp>
#include <vector>

Segmenter::Segmenter()
{
}

Segmenter::~Segmenter()
{
  // Free any pointers that were being used if there are any
}

void Segmenter::initialize(ros::NodeHandle nh)
{
  nh_ = nh;

  map_sub_ = nh_.subscribe("/map", 1, &Segmenter::mapCB, this);
  segmented_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/segmented_map", 1);
}

bool Segmenter::segment()
{
  ROS_WARN("This is the base class, no real functionality exists.");
  return false;
}

void Segmenter::getRegions(std::vector<Region> &vec)
{
  ROS_WARN("You should not be calling this!  Returning an empty vector.");
  std::vector<Region> empty_vec;

  vec = empty_vec;
}

// Consider changing this to a ConstPtr
void Segmenter::mapCB(const nav_msgs::OccupancyGrid &msg)
{
  map_ = msg;
}
