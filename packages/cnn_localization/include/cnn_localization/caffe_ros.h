/*
 * caffe_ros
 * 
 * An easier to use caffe/ROS bridge.  Inspired by "ros_caffe" by Tzutalin
 * (c) 2015 Tzutalin
 * (c) 2017 Luc Bettaieb
 * BSD
 */

#ifndef CNN_LOCALIZATION_CAFFE_ROS_H
#define CNN_LOCALIZATION_CAFFE_ROS_H

#include <vector>
#include <sstream>
#include <iostream>

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef std::pair<std::string, float> Prediction;

class CaffeROS
{
public:
  CaffeROS();

  CaffeROS(const std::string& model_file,
           const std::string& trained_file,
           const std::string& mean_file,
           const std::string& label_file);

  ~CaffeROS();

  std::vector<Prediction> classify(const cv::Mat &img, int N = 5);

private:
  std::vector<float> Predict(const cv::Mat &img);
  
  void SetMean(const std::string &mean_file);
  void WrapInputLayer(std::vector<cv::Mat>* input_channels);
  void Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels);

  std::shared_ptr<caffe::Net<float>> net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
  std::vector<std::string> labels_;
};

#endif  // CNN_LOCALIZATION_CAFFE_ROS_H
