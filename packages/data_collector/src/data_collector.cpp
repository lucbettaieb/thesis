/* 
 * data_collector
 *
 *
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image>

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <boost/filesystem.hpp>

const std::string R_ERROR = "ERROR";

bool new_image;

std::string current_region;
cv::Mat current_image;

ros::ServiceClient region_client;

void imgCB(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
		current_image = cv_ptr->image;
  	new_image = true;
	}
	catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception; couldn't convery msg callback to CvImagePtr: %s", e.what());
  }
}

void amclCB(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
	float[36] covariance = msg.pose.covariance;

	// TODO(lucbettaieb): check for covariance things!
	map_segmentation::GetRegion srv;
	srv.request.pose = msg.pose.pose;

	if (region_client.call(srv))
	{
		ROS_INFO("Region client called with current AMCL pose.");
		current_regioin = srv.response.region;
	}
	else
	{
		ROS_ERROR("Failed to call service... Something is wrong.");
		current_region = ERROR;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_collector");
	ros::NodeHandle nh;

	new_image = false;
	current_region = R_ERROR;

	region_client = nh.serviceClient<map_segmentation::GetRegion>("get_region");

	// TODO(lucbettaieb): Make these parameters
	ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1, amclCB);
	ros::Subscriber img_sub = nh.subscribe("/camera/rgb/image_raw/downsized", 1, imgCB);

	// TODO(lucbettaieb): Make this string comparison for better goodness
	std::string map_name;

	// Maybe get rid of this?
	std::cout << "ENTER MAP NAME: ";
	std::cin << map_name;
	std::cout << std::endl;

	// TODO(lucbettaieb): Do string checking for nonfucky strings

	// TODO(lucbettaieb): Make a parameter and/or make this in /tmp and then put a zip file on desktop
	std::string path = "/Users/luc/Desktop/" + map_name;
	boost::filesystem::path dir(path);

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
	while (ros::ok() && current_region != R_ERROR)
	{
		// Start saving training data to disk
		// Format
		// cnn_localization_<map_name>_<date>
		// /region_name1/img_n.jpg
		// /region_name2/img_n+1.jpg
		if (new_image)
		{
				

			new_image = false;
		}
	}

	return 1;
}
