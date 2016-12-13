/*
 * cnn_localizer
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <cnn_localization/cnn_localizer.h>

// C++ Standard Library
#include <algorithm>
#include <fstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

CNNLocalizer::CNNLocalizer(ros::NodeHandle &nh)
{
  // Gimme dat node handle
  g_nh_ = nh;

  ROS_INFO("Grabbing parameters from the parameter server.");

  if (!g_nh_.getParam("graph_path", g_graph_path_))
  {
    // TODO(enhancement): Consider searching some common locations for PB files
    g_graph_path_ = "/home/luc/Desktop/output_graph.pb";
  }
  if (!g_nh_.getParam("label_path", g_label_path_))
  {
    g_label_path_ = "/home/luc/Desktop/output_labels.txt";
  }
  if (!g_nh_.getParam("image_topic", g_image_topic_))
  {
    g_image_topic_ = "camera/rgb/image_raw";
  }

  // Set up image subscriber
  image_transport::ImageTransport it(g_nh_);
  g_image_subscriber_ = it.subscribe(g_image_topic_, 10, &CNNLocalizer::imageCB, this);
 
  // TENSORFLOW STUFF
  ROS_INFO("About to create a new tensorflow session.");
  g_tf_status_ = tensorflow::NewSession(tensorflow::SessionOptions(), &g_tf_session_ptr_);
  checkStatus(g_tf_status_);

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
  g_img_width_ = 64;
  g_img_height_ = 64;

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

// TODO(enhancement): Make this return a list for better results
std::tuple<std::string, double> CNNLocalizer::runImage()
{
  // First, create an empty result tuple
  std::tuple<std::string, double> result;

  // Initialize it with dummy variables (if an image has not been recieved)
  std::get<0>(result) = "none";
  std::get<1>(result) = 0.0;

  ROS_INFO("ABOUT TO RUN");
  // If there is an image..
  if (g_got_image_)
  {
    ROS_INFO("GOT IMG RUN");

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
    ROS_INFO("about to populate tensor");
    for (int y = 0; y < img_height; ++y)
    {
      const float* source_row = source_data + (y * img_width * img_depth);
      for (int x = 0; x < img_width; ++x)
      {
        const float* source_pixel = source_row + (x * img_depth);
        for (int c = 0; c < img_depth; ++c)
        {
          const float* source_value = source_pixel + c;
          //std::cout << "WIDTH: " << img_width << ", HEIGHT: " << img_height << ", DEPTH: " << img_depth << "| y: " << y << ", x: " << x << ", c: " << c << ", src val: " << *source_value << std::endl;
          input_image_mapped(0, y, x, c) = *source_value;
        }
      }
    }

    ROS_INFO("about to create var");
    // Create a vector of tensors to be populated by running the graph
    std::vector<tensorflow::Tensor> finalOutput;
    std::string InputName = "unknown_image";
    std::string OutputName = "output_vector";

    ROS_INFO("about to run");
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
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    // SEGFAULTINESS HERE!!!
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

    ROS_INFO("Attempt!");
    cv::Mat image = cv_ptr->image.clone();
    ROS_INFO("Success!");

    image.convertTo(g_most_recent_image_, CV_32FC3);

    g_img_height_ = msg->height;
    g_img_width_ = msg->width;

    g_got_image_ = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Failed converting the received message: %s", e.what());
    return;
  }
}
