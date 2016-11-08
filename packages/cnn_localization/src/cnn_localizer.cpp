/*
 * cnn_localizer
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <cnn_localization/cnn_localizer.h>

CNNLocalizer::CNNLocalizer(ros::NodeHandle &nh)
{
  // Gimme dat node handle
  g_nh_ = nh;

  // Check to see if parameter is set for path to peanutbutter
  g_nh_.param<std::string>("")

}

CNNLocalizer::