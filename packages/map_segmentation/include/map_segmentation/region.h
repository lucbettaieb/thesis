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
  double x;
  double y;
};

class Region
{
public:
  Region(double tl_x, double tl_y,
         double tr_x, double tr_y,
         double bl_x, double bl_y,
         double br_x, double br_y);
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

  Point2d getCenter();
};

#endif  // MAP_SEGMENTATION_REGION_H
