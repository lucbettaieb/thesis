/*
 * 4class_service_caller
 * waits for a service call and then calls other services
 *
 * (c) 2017 Luc Betaieb
 */

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

std::string g_class_;

std_srvs::Empty srv_call;

ros::ServiceClient g_office_client_;
ros::ServiceClient g_machineshop_client_;
ros::ServiceClient g_lab_client_;
ros::ServiceClient g_serverroom_client_;

void classificationCB(const std_msgs::String &msg)
{
  g_class_ = msg.data;
}

bool initAMCL(std_srvs::Empty::Request &req,
              std_srvs::Empty::Response &res)
{
  std::cout << "init AMCL w/ " << g_class_ << std::endl;
  if (g_class_ == "office")
  {
    g_office_client_.call(srv_call);
    return 1;
  }
  else if (g_class_ == "lab")
  {
    g_lab_client_.call(srv_call);
    return 1;
  }
  else if (g_class_ == "machineshop")
  {
    g_machineshop_client_.call(srv_call);
    return 1;
  }
  else if (g_class_ == "serverroom")
  {
    g_serverroom_client_.call(srv_call);
    return 1;
  }
  else
  {
    ROS_ERROR("Class is mismatched.  Maybe you're using the wrong classifier?");
    return false;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "four_class_service_caller");
  ros::NodeHandle nh;

  ros::Subscriber caffe_sub = nh.subscribe("/caffe_ret", 100, classificationCB);
  ros::ServiceServer init_service = nh.advertiseService("init_amcl_from_classifier", initAMCL);

  g_office_client_ = nh.serviceClient<std_srvs::Empty>("office_initialization");
  g_machineshop_client_ = nh.serviceClient<std_srvs::Empty>("machineshop_initialization");
  g_lab_client_ = nh.serviceClient<std_srvs::Empty>("lab_initialization");
  g_serverroom_client_ = nh.serviceClient<std_srvs::Empty>("serverroom_initialization");

  ros::spin();

}
