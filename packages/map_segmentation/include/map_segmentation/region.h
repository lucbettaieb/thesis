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

#include <string>

struct Point2d
{
  int x;
  int y;
};

class Region
{
public:
  Region(int tl_x, int tl_y,
         int tr_x, int tr_y,
         int bl_x, int bl_y,
         int br_x, int br_y);
  Region();

  ~Region();

  bool inRegion(geometry_msgs::PoseWithCovarianceStamped pose);
  bool inRegion(geometry_msgs::PoseWithCovariance pose);
  bool inRegion(geometry_msgs::PoseStamped pose);
  bool inRegion(geometry_msgs::Pose pose);

  Point2d top_left;
  Point2d top_right;
  Point2d bottom_left;
  Point2d bottom_right;

  std::string id;

private:
  


};

#endif  // MAP_SEGMENTATION_REGION_H
