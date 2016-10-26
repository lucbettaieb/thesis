/*
 * segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#ifndef MAP_SEGMENTATION_SEGMENTER_H
#define MAP_SEGMENTATION_SEGMENTER_H

// ROS
#include <ros/ros.h>

// ROS Messages
#include <nav_msgs/OccupancyGrid.h>

#include <map_segmentation/region.h>

class Segmenter
{
public:
  /*
   * @brief The constructor for the Segmenter class
   * 
   * This class will provide the skeleton for a map segmenter
   */
  Segmenter();

  /*
   * @brief Destructor for the Segmenter base class
   */
  ~Segmenter();

  void initialize(ros::NodeHandle nh);

  void segment();

private:
  ros::NodeHandle nh_;

  ros::Subscriber map_sub_;
  ros::Publisher segmented_map_pub_;

  nav_msgs::OccupancyGrid map_;

  nav_msgs::OccupancyGrid segmented_map_;

  // Specify the map topic on the parameter server
  void mapCB(const nav_msgs::OccupancyGrid &msg);
};

#endif  // MAP_SEGMENTATION_SEGMENTER_H
