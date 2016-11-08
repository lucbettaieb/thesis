/*
 * cnn_localizer
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <cnn_localization/cnn_localizer.h>

#include <string>

CNNLocalizer::CNNLocalizer(ros::NodeHandle &nh)
{
  // Gimme dat node handle
  g_nh_ = nh;

  std::string graph_path;

  g_tf_status_ = tensorflow::NewSession(tensorflow::SessionOptions(), &g_tf_session_ptr_);
  checkStatus(g_tf_status_);

  // Check to see if parameter is set for path to peanutbutter
  if (!g_nh_.getParam("graph_path", graph_path))
  {
    graph_path = "";
    // optionally search for a pb file in some common locations
  }

  // load the graph
  g_tf_status_ = ReadBinaryProto(tensorflow::Env::Default(), graph_path, &g_tf_graph_def_);
  checkStatus(g_tf_status_);

  g_tf_status_ = g_tf_session_ptr_->Create(g_tf_graph_def_);
  checkStatus(g_tf_status_);

  g_tf_status_ = g_tf_session_ptr_->Create(g_tf_graph_def_);
  checkStatus(g_tf_status_);
}

CNNLocalizer::~CNNLocalizer()
{
  // free(g_tf_session_ptr_);
  delete g_tf_session_ptr_;
}

bool CNNLocalizer::checkStatus(const tensorflow::Status &status)
{
  if (!status.ok())
  {
    ROS_ERROR("STATUS IS NOT OK!");
    std::cerr << status.ToString() << std::endl;;
    return false;
  }
  else
  {
    return true;
  }
}

void CNNLocalizer::runImage()
{
  if (g_got_image_)
  {
    // create a tensorflow::Tensor with the image information
    // tensorflow::Tensor input_image(tf)
    // Do tensorflow stuff here
  }
}
