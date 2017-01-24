/*
 * data_collector
 *
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

DataCollector::DataCollector(ros::NodeHandle &nh)
{
  g_nh_ = nh;
}

DataCollector::~DataCollector()
{
  g_current_region_ = R_ERROR;
  g_current_region_index_ = 0;
  g_in_saturated_region_ = false;
  g_new_image_ = false;

  // Load up the server client to get the current region
  g_region_client_ = nh.serviceClient<map_segmentation::GetRegion>("get_region");

  // Load the number of samples required per region
  nh.param<int>("data_collector/required_samples", g_n_images_required_, 30);

  // Load in the image topic from the parameter server
  std::string image_topic;
  nh.param<std::string>("data_collector/image_topic", image_topic, "/camera/rgb/image_raw/downsized");
  ros::Subscriber img_sub = nh.subscribe(image_topic, 1, &DataCollector::imgCB, this);

  // AMCL Pose is published to a set topic
  ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1, &DataCollector::amclCB, this);
  

}

bool DataCollector::collectData()
{
  
  return true;
}

bool DataCollector::isRegionVisited(std::string region)
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
bool DataCollector::isRegionVisitedAndSaturated(std::string region)
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

bool DataCollector::incrementCurrentRegionSnapshotIndex()
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

int DataCollector::getCurrentRegionNSnaphots()
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

void DataCollector::imgCB(const sensor_msgs::Image &msg)
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


void DataCollector::amclCB(const geometry_msgs::PoseWithCovarianceStamped &msg)
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
