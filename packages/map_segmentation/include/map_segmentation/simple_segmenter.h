/*
 * simple_segmenter
 * 
 * Copyright (c) 2016, Luc Bettaieb
 * BSD Licensed
 */

#ifndef MAP_SEGMENTATION_SIMPLE_SEGMENTER_H
#define MAP_SEGMENTATION_SIMPLE_SEGMENTER_H

#include <map_segmentation/segmenter.h>

namespace segmenter_plugins
{

class SimpleSegmenter : public Segmenter
{
public:
  SimpleSegmenter();

  ~SimpleSegmenter();

private:
  void segment();
}

}  // namespace segmenter_plugins

#endif  // MAP_SEGMENTATION_SIMPLE_SEGMENTER_H
