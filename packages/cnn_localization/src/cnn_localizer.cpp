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

#include "tensorflow/cc/ops/const_op.h"
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

tensorflow::Status ConvertFromOpenCVToRunnableTensor()
{

}

CNNLocalizer::CNNLocalizer(ros::NodeHandle &nh)
{
  // Gimme dat node handle
  g_nh_ = nh;

  ROS_INFO("Grabbing parameters from the parameter server.");

  if (!g_nh_.getParam("graph_path", g_graph_path_))
  {
    // TODO(enhancement): Consider searching some common locations for PB files
    g_graph_path_ = "/home/luc/Desktop/32x32_POC_retrain/output_graph.pb";
  }
  if (!g_nh_.getParam("label_path", g_label_path_))
  {
    g_label_path_ = "/home/luc/Desktop/32x32_POC_retrain/output_labels.txt";
  }
  if (!g_nh_.getParam("image_topic", g_image_topic_))
  {
    g_image_topic_ = "camera/rgb/image_raw/downsized";
  }

  // Set up image subscriber
  image_transport::ImageTransport it(g_nh_);
  g_image_subscriber_ = it.subscribe(g_image_topic_, 10, &CNNLocalizer::imageCB, this);
 
  // TENSORFLOW STUFF
  ROS_INFO("About to create a new tensorflow session.");

  tensorflow::SessionOptions options;
  g_tf_session_ptr_.reset(tensorflow::NewSession(options));

  // (g_tf_status_);

  // Load the graph
  ROS_INFO("Loading the graph.");
  g_tf_status_ = ReadBinaryProto(tensorflow::Env::Default(), g_graph_path_, &g_tf_graph_def_);
  checkStatus(g_tf_status_);

  // Creating the session with the graph
  ROS_INFO("Creating the session with the loaded graph.");
  g_tf_status_ = g_tf_session_ptr_->Create(g_tf_graph_def_);
  checkStatus(g_tf_status_);

  // Default image height and width.
  // This is updated with each new image.
  g_img_width_ = 42;
  g_img_height_ = 32;

  g_got_image_ = false;
}

// CNNLocalizer::CNNLocalizer(ros::NodeHandle &nh, std::string graph_path, std::string label_path)
// {
//   CNNLocalizer(nh);
//   g_graph_path_ = graph_path;
//   g_label_path_ = label_path;
// }

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
  std::get<0>(result) = "none";
  std::get<1>(result) = 0.0;

  // If there is an image..
  if (g_got_image_)
  {

    // create a tensorflow::Tensor with the image information

    int img_depth = 3;// = g_most_recent_image_.channels();
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
    auto start = ros::Time::now();
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
          // std::cout << "WIDTH: " << img_width << ", HEIGHT: " << img_height << ", DEPTH: " << img_depth << "| y: " << y << ", x: " << x << ", c: " << c << ", src val: " << *source_value << std::endl;
          input_image_mapped(0, y, x, c) = *source_value;
        }
      }
    }
    auto end = ros::Time::now();
    std::cerr << "Copy finished in: " << end.toSec() - start.toSec() << ", H/W/D: " << img_height << " / " << img_width << " / " << img_depth<< std::endl;
    std::cout << "tensor shape before resizing: " << input_image.shape().num_elements() << std::endl;
    // Create a vector of tensors to be populated by running the graph
    std::vector<tensorflow::Tensor> finalOutput;
    std::string InputName = "Mul";  // TODO(lucbetaieb): These seem to be correct but I should make sure
    std::string OutputName = "final_result";

    // Run the input_image tensor through the graph and store the output in the output vector

    ////////////////////////////////////////

    // std::vector<tensorflow::Tensor> out_tensors;
    // NEW NEW NEW NEW NEW HERE
    // auto root = tensorflow::Scope::NewRootScope();

    // auto resized = tensorflow::ops::ResizeBilinear(root, input_image, tensorflow::ops::Const(root.WithOpName("size"), {299, 299}));
    // tensorflow::Status chop_status;

    // tensorflow::ops::Div(root.WithOpName("normalized"), tensorflow::ops::Sub(root, resized, {128}), {128}); 

    // tensorflow::GraphDef graph;
    // chop_status = root.ToGraphDef(&graph);
    // checkStatus(chop_status);

    // std::unique_ptr<tensorflow::Session> session(tensorflow::NewSession(tensorflow::SessionOptions()));
    // chop_status = (session->Create(graph));
    // checkStatus(chop_status);
    // chop_status = (session->Run({}, {"normalized"}, {}, &out_tensors));
    // checkStatus(chop_status);

    ////////////////////////////////////////

    std::cout << "tensor shape after resizing: " << input_image.shape().num_elements() << std::endl;

    start = ros::Time::now();
    tensorflow::Status run_status = g_tf_session_ptr_->Run({{InputName, input_image}}, {OutputName}, {}, &finalOutput);
    //tensorflow::Status run_status = g_tf_session_ptr_->Run
    end = ros::Time::now();
    std::cerr << "Run finished in: " << end.toSec() - start.toSec() << std::endl;

    checkStatus(run_status);

    // Move the first Tensor from the output to its own piece of real estate
    tensorflow::Tensor output = std::move(finalOutput.at(0));

    // Getting the scores from the first tensor
    auto scores = output.flat<float>();
    //std::cerr << "scores size: " << scores.size() << std::endl;
    std::vector<std::pair<float, std::string>> sorted;

    // Label File Name
    std::ifstream label(g_label_path_);
    std::string line;

    for (uint i = 0; i <= 4; ++i)
    {
      std::getline(label, line);
      sorted.emplace_back(scores(i), line);
      // std::cout << scores(i) << " / line=" << line << std::endl;
    }

    std::sort(sorted.begin(), sorted.end());
    std::reverse(sorted.begin(), sorted.end());

    // std::cout << "size of the sorted file is " << sorted.size() << std::endl;

    // for (uint i = 0; i < 5; ++i)
    // {
    //   std::cout << "The output of the current graph has category  " << sorted[i].second
    //             << " with probability " << sorted[i].first << std::endl;
    // }

    // Set result to the "best" outcome!
    std::get<0>(result) = sorted[0].second;
    std::get<1>(result) = sorted[0].first;
  }

  return result;
}

void CNNLocalizer::imageCB(const sensor_msgs::ImageConstPtr &msg)
{
  g_mutex_.lock();

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    // SEGFAULTINESS HERE!!!
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

    cv::Mat image_in = cv_ptr->image.clone();

    cv::Mat image_out;
    cv::Size size(299, 299);
    cv::resize(image_in, image_out, size, 0, 0, CV_INTER_LINEAR);

    image_out.convertTo(g_most_recent_image_, CV_32FC3);

    cv::imshow("upsampled", image_out);
    cv::waitKey(1);
    // cv::destroyWindow("upsampled");
    g_img_height_ = 299;  // msg->height;
    g_img_width_ = 299;  // msg->width;

    g_got_image_ = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Failed converting the received message: %s", e.what());
    return;
  }

  g_mutex_.unlock();
}
