/*
 * region
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#include <map_segmentation/region.h>

Region::Region(int tl_x, int tl_y,
               int tr_x, int tr_y,
               int bl_x, int bl_y,
               int br_x, int br_y)
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
  top_left.x = 0;
  top_left.y = 0;

  top_right.x = 0;
  top_right.y = 0;

  bottom_left.x = 0;
  bottom_left.y = 0;

  bottom_right.x = 0;
  bottom_right.y = 0;
}

Region::~Region()
{
}

