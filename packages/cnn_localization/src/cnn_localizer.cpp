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
#include <fstream>

CNNLocalizer::CNNLocalizer(ros::NodeHandle &nh)
{
  // Gimme dat node handle
  g_nh_ = nh;

  g_tf_status_ = tensorflow::NewSession(tensorflow::SessionOptions(), &g_tf_session_ptr_);
  checkStatus(g_tf_status_);

  // Check to see if parameter is set for path to peanutbutter
  if (!g_nh_.getParam("graph_path", g_graph_path_))
  {
    g_graph_path_ = "/home/luc/Desktop/output_graph.pb";
    // optionally search for a pb file in some common locations?
  }

  if (!g_nh_.getParam("label_path", g_label_path_))
  {
    g_graph_path_ = "/home/luc/Desktop/output_labels.txt";
  }

  // Load the graph
  g_tf_status_ = ReadBinaryProto(tensorflow::Env::Default(), g_graph_path_, &g_tf_graph_def_);
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
  // First, create an empty result tuple
  std::tuple<std::string, double> result;

  // Initialize it with dummy variables (if an image has not been recieved)
  std::get<0>(result) = "none";
  std::get<1>(result) = 0.0;

  // If there is an image..
  if (g_got_image_)
  {
    // create a tensorflow::Tensor with the image information
    tensorflow::TensorShape image_shape;
    image_shape.AddDim(g_img_height_);
    image_shape.AddDim(g_img_width_);

    // Create the Tensor of integer type in the same shape as the image
    tensorflow::Tensor input_image(tensorflow::DT_INT8, image_shape);

    // Pull the Tensor out of the object to put data inside it
    auto input_image_mapped = input_image.tensor<int, 3>();

    // Grab a constant pointer to an integer stream of the image data
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

    // Create a vector of tensors to be populated by running the graph
    std::vector<tensorflow::Tensor> finalOutput;
    std::string InputName = "unknown_image";
    std::string OutputName = "output_vector";

    // Run the input_image tensor through the graph and store the output in the output vector
    tensorflow::Status run_status = g_tf_session_ptr_->Run({{InputName, input_image}}, {OutputName}, {}, &finalOutput);

    checkStatus(run_status);

    std::cerr << "final output size = " << finalOutput.size() << std::endl;

    // Move the first Tensor from the output to its own piece of real estate
    tensorflow::Tensor output = std::move(finalOutput.at(0));

    // Getting the scores from the first tensor
    auto scores = output.flat<float>();
    std::cerr << "scores size: " << scores.size() << std::endl;
    std::vector<std::pair<float, std::string>> sorted;

    // Label File Name
    std::string labelfile = "../../model/imagenet_comp_graph_label_strings.txt";  // PARAM THIS
    std::ifstream label(labelfile);
    std::string line;
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

    // Set result to the "best" outcome!
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
