#ifndef RGBDTOOLS_GLOBAL_CLOUD_ALIGN_H
#define RGBDTOOLS_GLOBAL_CLOUD_ALIGN_H

#include "rgbdtools/rgbdtools.h"

void printUsage(char** argv);

void alignGlobalCloud(const rgbdtools::PointCloudT::Ptr& cloud);

#endif // RGBDTOOLS_GLOBAL_CLOUD_ALIGN_H