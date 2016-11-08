/*
 * cnn_localizer
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

// Make sure tensorflow-cmake setup steps have been followed
// https://github.com/cjweeks/tensorflow-cmake

#ifndef CNN_LOCALIZATION_CNN_LOCALIZER_H
#define CNN_LOCALIZATION_CNN_LOCALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <tensorflow/core/public/session.h>
#include <tensorflow/core/platform/env.h>

class CNNLocalizer
{
public:
  CNNLocalizer();
  ~CNNLocalizer();

  void run_image();

private:
  ros::Publisher marker_publisher;
  ros::Subscriber image_subscriber;

  sensor_msgs::Image g_most_recent_image_;

  bool g_graph_loaded_;

  void loadGraph(std::string pb_path);

};

#endif  // CNN_LOCALIZATION_CNN_LOCALIZER_H
