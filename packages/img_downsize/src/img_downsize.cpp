/*
 * img_downsize.cpp
 * Luc Bettaieb
 * bettaieb@case.edu
 */

// C++ Standard Library
#include <string>

// ROS
#include <ros/ros.h>

// ROS Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

// Globals
ros::Publisher g_img_pub_;
ros::Subscriber g_img_sub_;

std::string g_image_topic_;

bool g_use_scale_;
int g_scale_;
int g_downsized_height_, g_downsized_width_;
int g_sampling_frequency_;

void imageCB(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img_out;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8 );
    cv::Mat img_in = cv_ptr->image;

    if (g_use_scale_)
    {
      cv::Size size(msg->width / g_scale_, msg->height / g_scale_);
      cv::resize(img_in, img_out, size);
    }
    else
    {
      cv::Size size(g_downsized_width_, g_downsized_height_);
      cv::resize(img_in, img_out, size);
    }
  }
  catch(cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception; couldn't convery msg callback to CvImagePtr: %s", e.what());
  }

  // Helper objects
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;

  header.stamp = msg->header.stamp;
  header.frame_id = msg->header.frame_id;

  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_out);
  img_bridge.toImageMsg(img_msg);

  g_img_pub_.publish(img_msg);
  ros::Rate(g_sampling_frequency_).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_downsize");
  ros::NodeHandle nh;

  nh.param<std::string>("image_topic", g_image_topic_, "/camera/rgb/image_raw");

  nh.param<int>("scale", g_scale_, 2);
  nh.param<bool>("use_scale", g_use_scale_, false);
  nh.param<int>("downsized_width", g_downsized_width_, 400);
  nh.param<int>("downsized_height", g_downsized_height_, 400);
  nh.param<int>("sampling_frequency", g_sampling_frequency_, 3);

  g_img_pub_ = nh.advertise<sensor_msgs::Image>(g_image_topic_ + "/downsized", 1);
  g_img_sub_ = nh.subscribe(g_image_topic_, 10, imageCB);

  ros::spin();
}
