/*
 * data_collector
 *
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#ifndef DATA_COLLECTOR_DATA_COLLECTOR_H
#define DATA_COLLECTOR_DATA_COLLECTOR_H

// C++ Standard Library
#include <string>
#include <vector>
#include <algorithm>
#include <tuple>

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// ROS Messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry>

// OpenCV
#include <opencv2/opencv.hpp>

// Booost
#include <boost/filesystem.hpp>

// Map Segmentation
#include <map_segmentation/GetRegion.h>

typedef std::tuple<std::string, int> StringIntTuple;

const std::string R_ERROR = "ERROR";

class DataCollector
{
public:
  DataCollector(ros::NodeHandle &nh);
  ~DataCollector();

  bool collectData();

private:
  ros::NodeHandle g_nh_;

  std::vector<StringIntTuple> g_visited_regions_;

  bool g_new_image_, g_in_saturated_region_;

  int g_current_region_index_, g_n_images_required_;

  std::string g_current_region_;

  cv::Mat g_current_image_;

  std::string g_path_;

  ros::ServiceClient g_region_client_;

  bool isRegionVisited(std::string region);

  bool isRegionVisitedAndSaturated(std::string region);

  bool incrementCurrentRegionSnapshotIndex();

  int getCurrentRegionNSnaphots();

  void imgCB(const sensor_msgs::Image &msg);

  void amclCB(const geometry_msgs::PoseWithCovarianceStamped &msg);

}


#endif  // DATA_COLLECTOR_DATA_COLLECTOR_H
