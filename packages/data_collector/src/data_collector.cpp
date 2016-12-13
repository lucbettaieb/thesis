/* 
 * data_collector
 * subscribes to img downsize and saves image to a directory
 *
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <vector>
#include <algorithm>
#include <tuple>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>

#include <map_segmentation/GetRegion.h>

typedef std::tuple<std::string, int> StringIntTuple;

const std::string R_ERROR = "ERROR";

bool g_new_image_;
uint g_n_images_required_;

std::vector<StringIntTuple> g_visited_regions_;
bool g_in_saturated_region_;

int g_current_region_index_;
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
      ROS_INFO("Region is saturated, find a new region");

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
    ROS_ERROR("Failed to call service... Something is wrong.");
    g_current_region_ = R_ERROR;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_collector");
  ros::NodeHandle nh;

  g_new_image_ = false;


  g_current_region_ = R_ERROR;
  g_current_region_index_ = 0;
  g_n_images_required_ = 40;
  g_in_saturated_region_ = false;

  region_client = nh.serviceClient<map_segmentation::GetRegion>("get_region");

  // TODO(lucbettaieb): Make these parameters
  ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1, amclCB);
  ros::Subscriber img_sub = nh.subscribe("/camera/rgb/image_raw/downsized", 1, imgCB);
  ros::Rate naptime(5);  // 5 hz

  // TODO(lucbettaieb): Make this string comparison for better goodness (?)
  std::string map_name;

  // Maybe get rid of this?
  std::cout << "ENTER MAP NAME: ";
  std::cin >> map_name;
  std::cout << std::endl;

  // TODO(lucbettaieb): Do string checking for nonfucky strings

  // TODO(lucbettaieb): Make a parameter and/or make this in /tmp and then put a zip file on desktop
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

  // TODO(lucbettaieb): Define heuristic for region and also get most coverage possible for map or interrupt
  while (ros::ok())
  {
    // Start saving training data to disk
    // Format
    // cnn_localization_<map_name>_<date>
    // /region_name1/img_n.jpg
    // /region_name2/img_n+1.jpg
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
            ROS_ERROR("FIND NEW REGION");
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
