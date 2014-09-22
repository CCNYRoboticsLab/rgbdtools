/**
 *  @file gft_detector.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rgbdtools/features/gft_detector.h"

namespace rgbdtools {

GftDetector::GftDetector():
  FeatureDetector()
{
  gft_detector_.reset(
    new cv::GoodFeaturesToTrackDetector(n_features_, 0.01, min_distance_));
}

GftDetector::~GftDetector()
{

}

void GftDetector::findFeatures(RGBDFrame& frame, const cv::Mat& input_img)
{
  cv::Mat mask(frame.depth_img.size(), CV_8UC1);
  frame.depth_img.convertTo(mask, CV_8U);

  gft_detector_->detect(input_img, frame.keypoints, mask);
  
  if(compute_descriptors_)
    orb_descriptor_.compute(
      input_img, frame.keypoints, frame.descriptors);
}

void GftDetector::setNFeatures(int n_features)
{
  n_features_ = n_features;
    
  gft_detector_.reset(
    new cv::GoodFeaturesToTrackDetector(n_features_, 0.01, min_distance_));
}
    
void GftDetector::setMinDistance(double min_distance)
{
  min_distance_ = min_distance;
    
  gft_detector_.reset(
    new cv::GoodFeaturesToTrackDetector(n_features_, 0.01, min_distance_));
}

} // namespace rgbdtools
