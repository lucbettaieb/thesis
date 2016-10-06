/*
 * segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

// C++ Standard Library

// ROS
#include <ros/ros.h>

// ROS Messages
#include <nav_msgs/OccupancyGrid.h>

class Segmenter
{
public:
  /*
   * @brief The constructor for the Segmenter class
   * 
   * This class will provide the skeleton for a map
   */
  Segmenter();

  /*
   * @brief Destructor for the Segmenter base class
   */
  ~Segmenter();

private:
  ros::NodeHandle nh_;

  nav_msgs::OccupancyGrid map_;

  void mapCB(const nav_msgs::OccupancyGridConstPtr &msg);
};
