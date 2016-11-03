/*
 * naive_segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#ifndef MAP_SEGMENTATION_NAIVE_SEGMENTER_H
#define MAP_SEGMENTATION_NAIVE_SEGMENTER_H

#include <map_segmentation/segmenter.h>
#include <vector>

namespace segmenter_plugins
{

class NaiveSegmenter : public Segmenter
{
public:
  NaiveSegmenter();
  virtual ~NaiveSegmenter();

  void initialize(ros::NodeHandle nh);
  void segment();

  void getRegions(std::vector<Region> &vec);

private:
  std::vector<Region> region_vector_;
};

}  // namespace segmenter_plugins

#endif  // MAP_SEGMENTATION_NAIVE_SEGMENTER_H
