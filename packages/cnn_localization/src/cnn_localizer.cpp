/*
 * cnn_localizer
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <cnn_localization/cnn_localizer.h>

#include <string>
#include <tuple>
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

std::tuple<std::string, double> CNNLocalizer::runImage()
{
  std::tuple<std::string, double> result;
  result.get<0> = "none";
  result.get<1> = 0.0;

  if (g_got_image_)
  {
    std::string label;

    // create a tensorflow::Tensor with the image information
    tensorflow::TensorShape image_shape;
    image_shape.AddDim(g_img_height_);
    image_shape.AddDim(g_img_width_);

    tensorflow::Tensor input_image(tensorflow::DT_INT8, image_shape);
    // I have no idea how to make this work right now.  Copying data is very confusing..
    for (uint i = 0; i < g_img_height_; i++)
    {
      for (uint j = 0; j < g_img_width_; j++)
      {
        // ??  Populate a matrix or something?
      }
    }
    // Copy the matrix into the tensor?
    // input_image.matrix<float>()() = z;
  }

  return result;
}

void CNNLocalizer::imageCB(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    g_most_recent_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    g_img_height_ = msg->height;
    g_img_width_ = msg->width;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Failed converting the received message: %s", e.what());
  }
}
