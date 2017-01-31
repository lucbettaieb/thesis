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

// C++ Standard Library
#include <string>
#include <tuple>
#include <mutex>

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// ROS Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// TensorFlow
#include <tensorflow/core/framework/graph.pb.h>

#include <tensorflow/core/public/session.h>
#include <tensorflow/core/platform/env.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

class CNNLocalizer
{
public:
  /*
   * @brief Default constructor for the CNNLocalizer
   *
   * @param nh the ROS NodeHandle to use for node communication
   */
  CNNLocalizer(ros::NodeHandle &nh);

  /*
   * @brief Secondary constructor for the CNNLocalizer with options for graph and label paths
   *
   * @param nh the ROS NodeHandle to use for node communication
   * @param graph_path the absolute path to the .pb tensorflow graph
   * @param label_path the absolute path to the .txt tensorflow labels
   */
  // CNNLocalizer(ros::NodeHandle &nh, std::string graph_path, std::string label_path);

  /*
   * @brief Destructor for the CNNLocalizer
   */
  ~CNNLocalizer();

  /*
   * @brief Runs the most current image through the loaded TensorFlow graph
   * @returns a tuple of the most likely label along with its confidence
   */
  std::tuple<std::string, double> runImage();

private:
  /*
   * @brief The global NodeHandle for use by the CNNLocalizer
   */
  ros::NodeHandle g_nh_;

  /*
   * @brief The global subscriber to get the most recent image
   */
  image_transport::Subscriber g_image_subscriber_;

  /*
   * @brief The most recent image acquired by the subscriber
   */
  cv::Mat g_most_recent_image_;

  /*
   * @brief The path to the TensorFlow graph that is to be loaded
   */
  std::string g_graph_path_;

  /*
   * @brief The path to the TensorFlow label file to be loaded
   */
  std::string g_label_path_;

  /*
   * @brief The topic for camera subscription
   */
  std::string g_image_topic_;

  /*
   * @brief Boolean to indicate whether or not an image has been received
   */
  bool g_got_image_;

  /*
   * @brief Height and width of the incoming images
   */
  int g_upsampled_image_height_, g_upsampled_image_width_;

  /*
   * @brief The tensorflow session ptr that is going to be loaded
   */
  std::unique_ptr<tensorflow::Session> g_tf_session_ptr_;

  /*
   * @brief The status continually checked while loading the graph initially
   */
  tensorflow::Status g_tf_status_;

  /*
   * @brief The graph definition used by tensorflow when loading the graph
   */
  tensorflow::GraphDef g_tf_graph_def_;

  /*
   * @brief Checks the status of a tensorflow::status, errors if something bad happens
   * @returns a boolean indicating the status.  false = bad
   */
  bool checkStatus(const tensorflow::Status &status);

  /*
   * @brief The image callback for the image subscriber
   */
  void imageCB(const sensor_msgs::ImageConstPtr &msg);

  boost::mutex g_mutex_;
};

#endif  // CNN_LOCALIZATION_CNN_LOCALIZER_H
