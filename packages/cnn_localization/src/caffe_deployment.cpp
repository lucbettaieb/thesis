/*
 * caffe_deployment
 * 
 * Copyright (c) 2017, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros_caffe/Classifier.h>

const std::string IMG_TOPIC = "camera/rgb/image_raw/downsized";
const std::string PUB_TOPIC = "/caffe_ret";

std::shared_ptr<Classifier> classifier;
std::string g_model_path_, g_weights_path_, g_mean_file_, g_label_file_, g_image_topic_, g_pub_topic;

ros::Publisher g_pub_;

void publishRet(const std::vector<Prediction>& predictions);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try 
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    // cv::imwrite("rgb.png", cv_ptr->image);
    cv::Mat img = cv_ptr->image;
    std::vector<Prediction> predictions = classifier->Classify(img);
    publishRet(predictions);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void publishRet(const std::vector<Prediction>& predictions)
{
  std_msgs::String msg;
  std::stringstream ss;
  for (size_t i = 0; i < predictions.size(); ++i)
  {
    Prediction p = predictions[i];
    ss << "[" << p.second << " - " << p.first << "]" << std::endl;
  }
  msg.data = ss.str();
  g_pub_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caffe_deployment");
  ros::NodeHandle nh;

  if (!g_nh_.getParam("caffe_deployment/image_topic", g_image_topic_))
  {
    g_image_topic_ = "/camera/rgb/image_raw/downsized";
  }
  if (!g_nh_.getParam("caffe_deployment/pub_topic", g_pub_topic_))
  {
    g_image_topic_ = "/caffe_ret";
  }
  if (!g_nh_.getParam("caffe_deployment/model_path", g_pub_topic_))
  {
    g_model_path_ = "/home/luc/Desktop/deploy.prototxt";
  }
  if (!g_nh_.getParam("caffe_deployment/weights_path", g_pub_topic_))
  {
    g_weights_path_ = "/home/luc/Desktop/model.caffemodel";
  }
  if (!g_nh_.getParam("caffe_deployment/mean_path", g_pub_topic_))
  {
    g_mean_file_ = "/home/luc/Desktop/mean.binaryproto";
  }
  if (!g_nh_.getParam("caffe_deployment/label_path", g_pub_topic_))
  {
    g_label_file_ = "/home/luc/Desktop/labels.txt";
  }

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub = it.subscribe(g_image_topic_, 1, imageCallback);
  g_pub_ = nh.advertise<std_msgs::String>(g_image_topic_, 100);

  classifier.reset(Classifier(g_model_path_, g_weights_path_, g_mean_file_, g_label_file_));

  ros::spin();

  ros::shutdown();
  return 0;
}
