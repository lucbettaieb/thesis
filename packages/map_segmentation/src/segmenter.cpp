/*
 * segmenter
 * 
 * Segmenter base class.  Takes in a map and returns a set of points 
 *
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <map_segmentation/segmenter.h>

Segmenter::Segementer(ros::NodeHandle nh) : nh_(nh)
{
  map_sub_ = nh_.subscribe("/map", 1, Segmenter::mapCB);
}

Segmenter::~Segmenter()
{
}

void Segmenter::segment()
{
  ROS_WARN("This is the base class, no real functionality exists.");
}

// Consider changing this to a ConstPtr
void Segmenter::mapCB(const nav_msgs::OccupancyGrid &msg)
{
  map_ = msg;
}
