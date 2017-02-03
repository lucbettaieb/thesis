/*
 * cnn_localizer
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

// C++ Standard Library
#include <algorithm>
#include <fstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// CNN Localizer
#include <cnn_localization/cnn_localizer.h>

#include <opencv2/highgui/highgui.hpp>

// TODO(lucbettaieb): Get rid of this crap
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"

CNNLocalizer::CNNLocalizer(ros::NodeHandle &nh)
{
  g_nh_ = nh;

  // Load parameters
  if (!g_nh_.getParam("demo_mode/graph_path", g_graph_path_))
  {
    ROS_ERROR("USING DEFAULT GRAPH");
    g_graph_path_ = "/home/luc/Desktop/32x32_POC_retrain/output_graph.pb";
  }

  if (!g_nh_.getParam("demo_mode/label_path", g_label_path_))
  {
    g_label_path_ = "/home/luc/Desktop/32x32_POC_retrain/output_labels.txt";
  }

  if (!g_nh_.getParam("demo_mode/image_topic", g_image_topic_))
  {
    g_image_topic_ = "camera/rgb/image_raw/downsized";
  }

  if (!g_nh_.getParam("demo_mode/upsampled_height", g_upsampled_image_height_))
  {
    g_upsampled_image_height_ = 299;
  }

  if (!g_nh_.getParam("demo_mode/upsampled_width", g_upsampled_image_width_))
  {
    g_upsampled_image_width_ = 299;
  }

  // Set up image subscriber
  image_transport::ImageTransport it(g_nh_);
  g_image_subscriber_ = it.subscribe(g_image_topic_, 10, &CNNLocalizer::imageCB, this);

  // TENSORFLOW STUFF
  tensorflow::SessionOptions options;
  g_tf_session_ptr_.reset(tensorflow::NewSession(options));

  // Load the graph
  g_tf_status_ = ReadBinaryProto(tensorflow::Env::Default(), g_graph_path_, &g_tf_graph_def_);
  checkStatus(g_tf_status_);

  // Creating the session with the graph
  g_tf_status_ = g_tf_session_ptr_->Create(g_tf_graph_def_);
  checkStatus(g_tf_status_);

  g_got_image_ = false;
}

CNNLocalizer::~CNNLocalizer()
{
  g_tf_session_ptr_->Close();
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

// TODO(enhancement): Make this return a list for better results
std::tuple<std::string, double> CNNLocalizer::runImage()
{
  // First, create an empty result tuple
  std::tuple<std::string, double> result;

  // Initialize it with dummy variables (if an image has not been recieved)
  if (!g_got_image_)
  {
    std::get<0>(result) = "none";
    std::get<1>(result) = 0.0;
  }
  else
  {
    // create a tensorflow::Tensor with the image information
    int img_depth = g_most_recent_image_.channels();
    int img_width = g_most_recent_image_.cols;
    int img_height = g_most_recent_image_.rows;

    tensorflow::TensorShape image_shape;
    image_shape.AddDim(1);
    image_shape.AddDim(img_height);
    image_shape.AddDim(img_width);
    image_shape.AddDim(img_depth);

    // Create the Tensor of integer type in the same shape as the image
    tensorflow::Tensor input_image(tensorflow::DT_FLOAT, image_shape);

    // Pull the Tensor out of the object to put data inside it
    auto input_image_mapped = input_image.tensor<float, 4>();

    // Grab a constant pointer to an integer stream of the image data
    const float* source_data = (float*)(g_most_recent_image_.data);

    // Potentially do some normalization operations?  Maybe do this in img_downsize..

    // https://gist.github.com/lucbettaieb/66c06f23de7a30b0ca0cccffb2bc732b
    // Populate the input image tensor
    for (int y = 0; y < img_height; ++y)
    {
      const float* source_row = source_data + (y * img_width * img_depth);
      for (int x = 0; x < img_width; ++x)
      {
        const float* source_pixel = source_row + (x * img_depth);
        for (int c = 0; c < img_depth; ++c)
        {
          const float* source_value = source_pixel + c;
          input_image_mapped(0, y, x, c) = *source_value;
        }
      }
    }

    // Create a vector of tensors to be populated by running the graph
    std::vector<tensorflow::Tensor> finalOutput;
    std::string InputName = "Mul";
    std::string OutputName = "final_result";

    // Run the input_image tensor through the graph and store the output in the output vector
    auto start = ros::Time::now();
    tensorflow::Status run_status = g_tf_session_ptr_->Run({{InputName, input_image}},
                                                           {OutputName},
                                                           {},
                                                           &finalOutput);
    auto end = ros::Time::now();

    std::cerr << "Run finished in: " << end.toSec() - start.toSec() << std::endl;
    checkStatus(run_status);

    // Move the first Tensor from the output to its own piece of real estate
    tensorflow::Tensor output = std::move(finalOutput.at(0));

    // Getting the scores from the first tensor
    auto scores = output.flat<float>();

    std::vector<std::pair<float, std::string>> sorted;

    // Label File Name
    std::ifstream label(g_label_path_);
    std::string line;

    for (uint i = 0; i <= 3; ++i)
    {
      std::getline(label, line);
      sorted.emplace_back(scores(i), line);
    }

    std::sort(sorted.begin(), sorted.end());
    std::reverse(sorted.begin(), sorted.end());

    // Print the results
    for (uint i = 0; i < 4; ++i)
    {
      std::cout << "OUTPUT: " << sorted[i].second
                << "| SCORE: " << sorted[i].first << std::endl;
    }

    // Set result to the "best" outcome!
    std::get<0>(result) = sorted[0].second;
    std::get<1>(result) = sorted[0].first;
  }

  return result;
}

void CNNLocalizer::imageCB(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat image_in = cv_ptr->image.clone();

    cv::Mat image_out;
    cv::Size size(g_upsampled_image_height_, g_upsampled_image_width_);

    cv::resize(image_in, image_out, size, 0, 0, CV_INTER_NN);

    image_out.convertTo(g_most_recent_image_, CV_32FC3);

    // Consider subtracting the mean and dividing by the scale?
    // This is what is done in label_image.cc

    // cv::Mat image_mean, image_std;
    // cv::meanStdDev(image_out, image_mean, image_std);
    // cv::subtract(image_out, image_mean, image_out);
    // cv::divide(image_out, image_std, image_out);

    cv::imshow("upsampled", image_out);
    cv::waitKey(1);

    g_got_image_ = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Failed converting the received message: %s", e.what());
    return;
  }
}
