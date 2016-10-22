/*
 * region
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#ifndef MAP_SEGMENTATION_REGION_H
#define MAP_SEGMENTATION_REGION_H

// ROS Messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

struct Point2d
{
  double x;
  double y;
};

class Region
{
public:
  Region();

  ~Region();

private:
  Point2d top_left;
  Point2d top_right;
  Point2d bottom_left;
  Point2d bottom_right;

  bool inRegion(geometry_msgs::PoseWithCovarianceStamped pose);
  bool inRegion(geometry_msgs::PoseWithCovariance pose);
  bool inRegion(geometry_msgs::PoseStamped pose);
  bool inRegion(geometry_msgs::Pose pose);
};

#endif  // MAP_SEGMENTATION_REGION_H
