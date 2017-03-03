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
#include <vector>
#include <algorithm>

#include <ros_caffe/Classifier.h>

std::shared_ptr<Classifier> g_classifier_;
std::string g_model_path_, g_weights_path_, g_mean_file_, g_label_file_, g_image_topic_, g_pub_topic_;
uint g_n_multi_hypothesis_;

ros::Publisher g_pub_;

std::vector<Prediction> g_sum_predictions_;
uint g_i_prediction_;

bool sortByScore(const Prediction &a, const Prediction &b)
{
  return a.second < b.second;
}

void publishRet(const std::vector<Prediction>& predictions);
void publishMultiHypothesis(std::vector<Prediction> predictions);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    // cv::imwrite("rgb.png", cv_ptr->image);
    cv::Mat img = cv_ptr->image;
    std::vector<Prediction> predictions = g_classifier_->Classify(img);
    // publishRet(predictions);

    std::sort(predictions.begin(), predictions.end());

    // If the counter is zero, clear the previous predicitons and initialize
    if (g_i_prediction_ == 0)
    {
      g_sum_predictions_.clear();
      g_sum_predictions_ = predictions;
    }
    // Otherwise, combine the predictions
    else
    {
      for (uint i = 0; i < predictions.size(); i++)
      {
        g_sum_predictions_[i].second() += predictions[i].second();
      }
    }

    if (g_i_prediction_ >= g_n_multi_hypothesis_)
    {
     publishMultiHypothesis(g_sum_predictions_);
     g_i_prediction_ = 0;
    }

    g_i_prediction_++;
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

void publishMultiHypothesis(std::vector<Prediction> predictions)
{
  std::sort(predictions.begin(), predictions.end(), sortByScore);
  std_msgs::String msg;
  msg.data = "CONSENSUS" + predictions.[i].first;
  g_pub_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caffe_deployment");
  ros::NodeHandle nh;

  g_i_prediction_ = 0;

  // Get parameters from the parameter server
  if (!g_nh_.getParam("caffe_deployment/image_topic", g_image_topic_))
  {
    g_image_topic_ = "/camera/rgb/image_raw/downsized";
  }
  if (!g_nh_.getParam("caffe_deployment/pub_topic", g_pub_topic_))
  {
    g_pub_topic_ = "/caffe_ret";
  }
  if (!g_nh_.getParam("caffe_deployment/model_path", g_model_path_))
  {
    g_model_path_ = "/home/luc/Desktop/deploy.prototxt";
  }
  if (!g_nh_.getParam("caffe_deployment/weights_path", g_weights_path_))
  {
    g_weights_path_ = "/home/luc/Desktop/model.caffemodel";
  }
  if (!g_nh_.getParam("caffe_deployment/mean_path", g_mean_file_))
  {
    g_mean_file_ = "/home/luc/Desktop/mean.binaryproto";
  }
  if (!g_nh_.getParam("caffe_deployment/label_path", g_label_file_))
  {
    g_label_file_ = "/home/luc/Desktop/labels.txt";
  }
  if (!g_nh_.getParam("caffe_deployment/g_n_multi_hypothesis_"), g_n_multi_hypothesis_)
  {
    g_n_multi_hypothesis_ = 4;
  }

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub = it.subscribe(g_image_topic_, 1, imageCallback);
  g_pub_ = nh.advertise<std_msgs::String>(g_image_topic_, 100);

  g_classifier_.reset(Classifier(g_model_path_, g_weights_path_, g_mean_file_, g_label_file_));

  ros::spin();

  ros::shutdown();
  return 0;
}
