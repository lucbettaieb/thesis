/*
 * map_segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>
#include <string>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>
#include <map_segmentation/segmenter.h>

#include <map_segmentation/GetRegion.h>
#include <vector>
#include <limits>

std::vector<Region> region_vector;

double euclidian_distance(geometry_msgs::Pose pose, Point2d center)
{
  std::cout << "PS" << pose.position.x << " " << center.x << std::endl;
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

  for (uint i = 0; i < region_vector.size(); i++)
  {
    double euc_dist = euclidian_distance(req.pose, region_vector.at(i).getCenter());
    std::cout << "ed: " << euc_dist << std::endl;
    if (euc_dist < min_dist)
    {
      min_dist = euc_dist;
      min_id = region_vector.at(i).id;
    }
  }
  res.region = min_id;
  return true;
}

// This should advertise a ROS service that, given a position, returns a region for a label
int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_segmenter");
  ros::NodeHandle nh;

  // Advertise service!
  // Consider doing this after whatever lookup table is published...
  ros::ServiceServer service = nh.advertiseService("get_region", getRegion);

  // Make this a parameter
  std::string plugin_type = "segmenter_plugins::NaiveSegmenter";

  pluginlib::ClassLoader<Segmenter> segmenter_loader("map_segmentation", "Segmenter");

  try
  {
    boost::shared_ptr<Segmenter> segmenter;

    segmenter = segmenter_loader.createInstance(plugin_type);
    segmenter->initialize(nh);

    ROS_INFO("Loaded segmenter, going into spin.");

    // Make this a parameter
    ros::Rate naptime(0.1);

    while (ros::ok())
    {
      segmenter->segment();
      segmenter->getRegions(region_vector);
      naptime.sleep();
      ros::spinOnce();
    }

  }
  catch(pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("Failed to load the segmenter for some reason. Error: %s", ex.what());
  }
}
