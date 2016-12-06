/*
 * cnn_localizer
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <cnn_localization/cnn_localizer.h>

#include <string>
#include <tuple>
#include <vector>
#include <utility>
#include <algorithm>

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
  g_tf_session_ptr_->Close();
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

  std::get<0>(result) = "none";
  std::get<1>(result) = 0.0;

  if (g_got_image_)
  {
    std::string label;

    // create a tensorflow::Tensor with the image information
    tensorflow::TensorShape image_shape;
    image_shape.AddDim(g_img_height_);
    image_shape.AddDim(g_img_width_);

    tensorflow::Tensor input_image(tensorflow::DT_INT8, image_shape);
    auto input_image_mapped = input_image.tensor<int, 3>();

    const int* source_data = (int*)(g_most_recent_image_->image.data);
    // Potentially do some normalization operations?  Maybe do this in img_downsize..

    // https://gist.github.com/lucbettaieb/66c06f23de7a30b0ca0cccffb2bc732b
    // Populate the input image tensor
    for (int y = 0; y < g_img_height_; ++y)
    {
      const int* source_row = source_data + (y * g_img_width_);
      for (int x = 0; x < g_img_width_; ++x)
      {
        const int* source_pixel = source_row + x;
        input_image_mapped(0, y, x) = *source_pixel;
      }
    }

    std::vector<tensorflow::Tensor> finalOutput;
    std::string InputName = "input";
    std::string OutputName = "output";

    tensorflow::Status run_status = g_tf_session_ptr_->Run({InputName, input_image}, {OutputName}, {}, &finalOutput);

    std::cerr << "final output size = " << finalOutput.size() << std::endl;

    tensorflow::Tensor output = std::move(finalOutput.at(0));

    auto scores = output.flat<float>();
    std::cerr << "scores size: " << scores.size() << std::endl;
    std::vector<std::pair<float, std::string>> sorted;

    for (uint i = 0; i <= 1000; ++i)
    {
      std::getline(label, line);
      sorted.emplace_back(scores(i), line);
      // std::cout << scores(i) << " / line=" << line << std::endl;
    }

    std::sort(sorted.begin(), sorted.end());
    std::reverse(sorted.begin(), sorted.end());

    std::cout << "size of the sorted file is " << sorted.size() << std::endl;

    for (uint i = 0; i < 5; ++i)
    {
      std::cout << "The output of the current graph has category  " << sorted[i].second
                << " with probability " << sorted[i].first << std::endl;
    }
    std::get<0>(result) = sorted[0].second;
    std::get<1>(result) = sorted[0].first;
  }

  return result;
}

void CNNLocalizer::imageCB(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    g_most_recent_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    g_img_height_ = msg->height;
    g_img_width_ = msg->width;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Failed converting the received message: %s", e.what());
  }
}
