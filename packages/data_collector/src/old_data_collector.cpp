/* 
 * data_collector
 * subscribes to img downsize and saves image to a directory
 *
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

// C++ Standard Library
#include <string>
#include <vector>
#include <algorithm>
#include <tuple>

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// ROS Messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Booost
#include <boost/filesystem.hpp>

// Map Segmentation
#include <map_segmentation/GetRegion.h>

typedef std::tuple<std::string, int> StringIntTuple;

const std::string R_ERROR = "ERROR";

std::vector<StringIntTuple> g_visited_regions_;

bool g_new_image_, g_in_saturated_region_;
int g_current_region_index_, g_n_images_required_;

std::string g_current_region_;

cv::Mat g_current_image_;

std::string g_path_;

ros::ServiceClient region_client;

bool isRegionVisited(std::string region)
{
  // TODO(enhancement): Make this more runtime efficient
  for (uint i = 0; i < g_visited_regions_.size(); i++)
  {
    if (std::get<0>(g_visited_regions_.at(i)) == region)
    {
      // Region has been visited
      return true;
    }
  }
  // Region has not been visited
  return false;
}
bool isRegionVisitedAndSaturated(std::string region)
{
  // TODO(enhancement): Make this more runtime efficient
  for (uint i = 0; i < g_visited_regions_.size(); i++)
  {
    if (std::get<0>(g_visited_regions_.at(i)) == region)
    {
      // Region has been visited
      if (std::get<1>(g_visited_regions_.at(i)) >= g_n_images_required_)
      {
        // Region is saturated
        return true;
      }
      else
      {
        // Region has been visited and is not saturated
        return false;
      }
    }
  }
  // Region has not been visited
  return false;
}

bool incrementCurrentRegionSnapshotIndex()
{
  for (uint i = 0; i < g_visited_regions_.size(); i++)
  {
    if (std::get<0>(g_visited_regions_.at(i)) == g_current_region_)
    {
      std::cout << std::get<1>(g_visited_regions_.at(i)) << std::endl;
      if (std::get<1>(g_visited_regions_.at(i)) < g_n_images_required_)
      {
        std::get<1>(g_visited_regions_.at(i)) = std::get<1>(g_visited_regions_.at(i)) + 1;
        return true;
      }
      else
      {
        return false;
      }
    }
  }
  return false;
}

int getCurrentRegionNSnaphots()
{
  for (uint i = 0; i < g_visited_regions_.size(); i++)
  {
    if (std::get<0>(g_visited_regions_.at(i)) == g_current_region_)
    {
      return std::get<1>(g_visited_regions_.at(i));
    }
  }
  return -1;
}

void imgCB(const sensor_msgs::Image &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    g_current_image_ = cv_ptr->image;
    g_new_image_ = true;
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception; couldn't convert msg callback to CvImagePtr: %s", e.what());
  }
}


void amclCB(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  // float covariance [36] = msg.pose.covariance;

  // TODO(lucbettaieb): check for covariance things!
  map_segmentation::GetRegion srv;
  srv.request.pose = msg.pose.pose;

  if (region_client.call(srv))
  {
    if (isRegionVisitedAndSaturated(srv.response.region))
    {
      // Region has been visited and is saturated
      ROS_ERROR("Region saturated.  Find new region!");

      g_in_saturated_region_ = true;
    }
    else if (!isRegionVisited(srv.response.region))
    {
      // Region has not been visited and should be added to the vector.
      ROS_INFO("Region has not been visited.");
      StringIntTuple visited_region_tuple;
      std::get<0>(visited_region_tuple) = srv.response.region;
      std::get<1>(visited_region_tuple) = 0;
      ROS_INFO("New region found, adding.");
      g_visited_regions_.push_back(visited_region_tuple);

      boost::filesystem::path dir(g_path_ + "/" + srv.response.region);

      if (boost::filesystem::create_directories(dir))
      {
        ROS_INFO("Created directory for new region!");
      }
      else
      {
        ROS_ERROR("Error creating directory!  THIS IS BAD!");
      }

      g_in_saturated_region_ = false;
    }
    else
    { 
      // Region has been visited and does not need to be added to the vector
      g_in_saturated_region_ = false;
    }

    g_current_region_ = srv.response.region;
  }
  else
  {
    ROS_WARN("Failed to call service... Something is wrong.");
    g_current_region_ = R_ERROR;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_collector");
  ros::NodeHandle nh;

  g_current_region_ = R_ERROR;
  g_current_region_index_ = 0;
  g_in_saturated_region_ = false;
  g_new_image_ = false;

  // Load up the server client to get the current region
  region_client = nh.serviceClient<map_segmentation::GetRegion>("get_region");

  // Load the number of samples required per region
  nh.param<int>("data_collector/required_samples", g_n_images_required_, 30);

  // Load in the image topic from the parameter server
  std::string image_topic;
  nh.param<std::string>("data_collector/image_topic", image_topic, "/camera/rgb/image_raw/downsized");
  ros::Subscriber img_sub = nh.subscribe(image_topic, 1, imgCB);

  // AMCL Pose is published to a set topic
  ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1, amclCB);
  
  // Load in sampling frequency from the parameter server
  int sampling_frequency;
  nh.param<int>("data_collector/sampling_frequency", sampling_frequency, 5);
  ros::Rate naptime(sampling_frequency);

  // Load the map name from the parameter server
  std::string map_name;
  nh.param<std::string>("data_collector/map_name", map_name, "default_map");

  // TODO(lucbettaieb): Do string checking to make sure the map name is OK
  g_path_ = "/tmp/" + map_name;
  boost::filesystem::path dir(g_path_);

  if (boost::filesystem::create_directories(dir))
  {
    ROS_INFO("Created directory!");
  }
  else
  {
    ROS_ERROR("Error creating directory!  Terminating program.");
    return 0;
  }

  // TODO(lucbettaieb): Put a zip file on desktop after completion?
  // TODO(lucbettaieb): Define better heuristic for region and also get most coverage possible for map or interrupt
  while (ros::ok())
  {
    if (g_new_image_ && (g_current_region_ != R_ERROR))
    {
      // Begin image saving
      try
      {
        std::vector<int> compression_params;
        compression_params.push_back(1);  // JPG quality
        compression_params.push_back(80);  // TODO(lucbettaieb): make param???

        if (!g_in_saturated_region_)
        {
          if (incrementCurrentRegionSnapshotIndex())
          {
            cv::imwrite(g_path_ + "/" + g_current_region_ + "/" + std::to_string(getCurrentRegionNSnaphots()) + ".jpg", g_current_image_, compression_params);
            std::cout << "Wrote new snapshot for region: " << g_current_region_ << std::endl;
            g_new_image_ = false;
          }
          else
          {
            ROS_ERROR("Region saturated.  Find new region!");
          }
        }
      }
      catch (std::runtime_error &e)
      {
        ROS_ERROR("OpenCV encountered an error doing imwrite: %s", e.what());
        return 1;
      }

      naptime.sleep();
    }

    ros::spinOnce();
  }

  return 0;
}
