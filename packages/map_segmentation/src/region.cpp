/*
 * region
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <map_segmentation/region.h>

Region::Region(double tl_x, double tl_y,
               double tr_x, double tr_y,
               double bl_x, double bl_y,
               double br_x, double br_y)
{
  top_left.x = tl_x;
  top_left.y = tl_y;

  top_right.x = tr_x;
  top_right.y = tr_y;

  bottom_left.x = bl_x;
  bottom_left.y = bl_y;

  bottom_right.x = br_x;
  bottom_right.y = br_y;
}

Region::Region()
{
  top_left.x = 0.0;
  top_left.y = 0.0;

  top_right.x = 0.0;
  top_right.y = 0.0;

  bottom_left.x = 0.0;
  bottom_left.y = 0.0;

  bottom_right.x = 0.0;
  bottom_right.y = 0.0;
}

Region::~Region()
{
}

Point2d Region::getCenter()
{
  Point2d center;

  center.x = (top_left.x + top_right.x) / 2;
  center.y = (top_left.y + bottom_left.y) / 2;

  center.x *= 0.05;
  center.y *= 0.05;

  return center;
}
