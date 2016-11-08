/*
 * cnn_localizer
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

// Make sure tensorflow-cmake setup steps have been followed
// https://github.com/cjweeks/tensorflow-cmake

#ifndef CNN_LOCALIZATION_CNN_LOCALIZER_H
#define CNN_LOCALIZATION_CNN_LOCALIZER_H

class CNNLocalizer
{
public:
  CNNLocalizer();
  ~CNNLocalizer();

private:
  ros::Publisher marker_publisher;
  ros::Subscriber image_subscriber;

};

#endif  // CNN_LOCALIZATION_CNN_LOCALIZER_H
