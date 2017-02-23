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

Classifier* classifier;
std::string model_path;
std::string weights_path;
std::string mean_file;
std::string label_file;
std::string image_path;

ros::Publisher g_pub_;

void publishRet(const std::vector<Prediction>& predictions);

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
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


// TODO: Define a msg or create a service
// Try to receive : $rostopic echo /caffe_ret
void publishRet(const std::vector<Prediction>& predictions)
{
  std_msgs::String msg;
  std::stringstream ss;
  for (size_t i = 0; i < predictions.size(); ++i) {
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
  image_transport::ImageTransport it(nh);


  image_transport::Subscriber sub = it.subscribe(IMG_TOPIC, 1, imageCallback);

  g_pub_ = nh.advertise<std_msgs::String>(PUB_TOPIC, 100);

  const std::string ROOT_SAMPLE = ros::package::getPath("ros_caffe");
  model_path = ROOT_SAMPLE + "/data/deploy.prototxt";
  weights_path = ROOT_SAMPLE + "/data/bvlc_reference_caffenet.caffemodel";
  mean_file = ROOT_SAMPLE + "/data/imagenet_mean.binaryproto";
  label_file = ROOT_SAMPLE + "/data/synset_words.txt";
  image_path = ROOT_SAMPLE + "/data/cat.jpg";

  classifier = new Classifier(model_path, weights_path, mean_file, label_file);

  ros::spin();

  delete classifier;
  ros::shutdown();
  return 0;
}
