#ifndef RGBDTOOLS_KEYFRAME_ASSOCIATIONS_H
#define RGBDTOOLS_KEYFRAME_ASSOCIATIONS_H

#include <time.h>
#include "rgbdtools/rgbdtools.h"

void printUsage(char** argv);

void buildAndSaveBFMatrix(
  rgbdtools::KeyframeVector keyframes,
  const std::string output_path);

#endif // RGBDTOOLS_KEYFRAME_ASSOCIATIONS_H