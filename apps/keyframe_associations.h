#ifndef RGBDTOOLS_KEYFRAME_ASSOCIATIONS_H
#define RGBDTOOLS_KEYFRAME_ASSOCIATIONS_H

#include <time.h>
#include <boost/filesystem.hpp>
#include "rgbdtools/rgbdtools.h"

void printUsage(char** argv);

void bruteForceAssociations(
  rgbdtools::KeyframeGraphDetector& graph_detector,
  rgbdtools::KeyframeVector& keyframes,
  const std::string& bf_output_path);

void treeAssociations(
  rgbdtools::KeyframeGraphDetector& graph_detector,
  rgbdtools::KeyframeVector& keyframes,
  const std::string& tree_output_path,
  const cv::Mat& bf_assoc,
  int k, int n);

#endif // RGBDTOOLS_KEYFRAME_ASSOCIATIONS_H