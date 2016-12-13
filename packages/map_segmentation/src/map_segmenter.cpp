/*
 * map_segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>
#include <map_segmentation/segmenter.h>

#include <map_segmentation/GetRegion.h>
#include <map_segmentation/GetRegionCenter.h>
#include <vector>
#include <limits>

std::vector<Region> region_vector;

double euclidian_distance(geometry_msgs::Pose pose, Point2d center)
{
  return std::sqrt(((pose.position.x * pose.position.x) - (center.x * center.x)) +
                   ((pose.position.y * pose.position.y) - (center.y * center.y)));
}

/*
 * Service callback to give the appropriate region (string) based on a pose
 */
bool getRegion(map_segmentation::GetRegion::Request &req,
               map_segmentation::GetRegion::Response &res)
{
  std::cout << "Got pose: " << req.pose.position.x << ", " << req.pose.position.y << std::endl;
  std::string min_id;
  double min_dist = std::numeric_limits<double>::max();
  Point2d min_center;
  for (uint i = 0; i < region_vector.size(); i++)
  {
    double euc_dist = euclidian_distance(req.pose, region_vector.at(i).getCenter());

    if (euc_dist < min_dist)
    {
      min_dist = euc_dist;
      min_id = region_vector.at(i).id;
      min_center = region_vector.at(i).getCenter();
    }
  }
  std::cout << "Center of region: " << min_center.x << ", " << min_center.y << std::endl;
  res.region = min_id;
  return true;
}

bool getRegionCenter(map_segmentation::GetRegionCenter::Request &req,
                     map_segmentation::GetRegionCenter::Response &res)
{
  for (uint i = 0; i < region_vector.size(); i++)
  {
    if (req.region_id.compare(region_vector.at(i).id) == 0)
    {
      res.frame_id = "map";  // TODO(lucbettaieb): Make this better..
      res.x = region_vector.at(i).getCenter().x;
      res.y = region_vector.at(i).getCenter().y;
      return true;
    }
  }
  return false;
}

// This should advertise a ROS service that, given a position, returns a region for a label
int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_segmenter");
  ros::NodeHandle nh;


  ros::Publisher region_center_pub = nh.advertise<visualization_msgs::MarkerArray>("/region_centers", 0);

  visualization_msgs::MarkerArray region_centers;

  // Load the plugin type from the parameter server
  std::string plugin_type;
  nh.param<std::string>("map_segmenter/segmenter", plugin_type, "segmenter_plugins::NaiveSegmenter");

  pluginlib::ClassLoader<Segmenter> segmenter_loader("map_segmentation", "Segmenter");

  try
  {
    boost::shared_ptr<Segmenter> segmenter;

    segmenter = segmenter_loader.createInstance(plugin_type);
    segmenter->initialize(nh);

    ROS_INFO("Loaded segmenter, going into spin.");

    // Load segmentation rate from the parameter server
    double segmentation_rate;
    nh.param<double>("map_segmenter/segmentation_rate", segmentation_rate, 0.5);
    ros::Rate naptime(segmentation_rate);

    while (ros::ok() && !segmenter->segment())
    {
      naptime.sleep();
      ros::spinOnce();
    }

    // Wait until the region vector is populated and then get them
    while (region_vector.size() == 0)
    {
      segmenter->getRegions(region_vector);
    }

    // Publish all region centers for visualization
    // TODO(lucbettaieb): Make these interactive so that AMCL can be
    // deemed untrustworthy
    for (uint i = 0; i < region_vector.size(); i++)
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "region_centers";

      marker.id = std::stoi(region_vector.at(i).id);
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = region_vector.at(i).getCenter().x;
      marker.pose.position.y = region_vector.at(i).getCenter().y;
      marker.pose.position.z = 0.01;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;

      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      region_centers.markers.push_back(marker);
    }

    region_center_pub.publish(region_centers);

    // Advertise services!
    ros::ServiceServer get_region_service = nh.advertiseService("get_region", getRegion);
    ros::ServiceServer get_region_center_service = nh.advertiseService("get_region_center", getRegionCenter);

    ROS_INFO("Standing by.");
    ros::spin();
  }

  catch(pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("Failed to load the segmenter for some reason. Error: %s", ex.what());
    return 0;
  }

  return 1;
}
