/*
 * img_downsize.cpp
 * Luc Bettaieb
 * bettaieb@case.edu
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>


ros::Publisher g_img_pub_;
ros::Subscriber g_img_sub_;

void imageCB(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception; couldn't convery msg callback to CvImagePtr: %s", e.what());
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_downsize");
  ros::NodeHandle nh;

  // TODO Make this the subscribed topic/downsized
  g_img_pub_ = nh.advertise<sensor_msgs::Image>("/downsized_img", 1);
  g_img_sub_ = nh.subscribe("/rgb", 10, imageCB);

  ros::spin();
}