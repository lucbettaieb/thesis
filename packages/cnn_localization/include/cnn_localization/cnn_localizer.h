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
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <tuple>

#include <tensorflow/core/public/session.h>
#include <tensorflow/core/platform/env.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

class CNNLocalizer
{
public:
  CNNLocalizer(ros::NodeHandle &nh);
  ~CNNLocalizer();

  std::tuple<std::string, double> runImage();

private:
  ros::NodeHandle g_nh_;

  ros::Subscriber g_image_subscriber_;

  cv_bridge::CvImagePtr g_most_recent_image_;

  std::string g_graph_path_;

  bool g_got_image_ = false;
  bool g_img_height_ = 64;  // TODO(lucbettaieb) Make better
  bool g_img_width_ = 64;
  bool checkStatus(const tensorflow::Status &status);

  void imageCB(const sensor_msgs::ImageConstPtr &msg);

  // Tensorflow stuff
  tensorflow::Session *g_tf_session_ptr_;

  tensorflow::Status g_tf_status_;
  tensorflow::GraphDef g_tf_graph_def_;
};

#endif  // CNN_LOCALIZATION_CNN_LOCALIZER_H
