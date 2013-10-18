/**
 *  @file motion_estimation_icp_prob_model.cpp
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

#include "rgbdtools/registration/motion_estimation_klt.h"

namespace rgbdtools {

MotionEstimationKLT::MotionEstimationKLT():
  MotionEstimation()
{
  first_time_ = true;
}

MotionEstimationKLT::~MotionEstimationKLT()
{

}

bool MotionEstimationKLT::getMotionEstimationImpl(
  RGBDFrame& frame,
  const AffineTransform& prediction,
  AffineTransform& motion)
{
  if (first_time_) {
    first_time_ = false;

    for (size_t i = 0; i < frame.keypoints.size(); ++i) {
      prev_pts_.push_back(frame.keypoints[i].pt);
    }
    printf("prev_points initialized to %lu\n", prev_pts_.size());
  } else {
    std::vector<unsigned char> status;
    std::vector<float> errors;
    std::vector<cv::Point2f> next_pts;

    printf("PyrLK on %lu points...\n", prev_pts_.size());
    cv::calcOpticalFlowPyrLK(prev_rgb_img_, 
                             frame.rgb_img, 
                             prev_pts_, 
                             next_pts, 
                             status,
                             errors);
    printf("prev_pts_: %lu, next_pts: %lu\n", prev_pts_.size(), 
        next_pts.size());  

    // The tracked features:
    std::vector<cv::Point2f> tracked_pts;
    std::vector<cv::KeyPoint> tracked_kpts;

    int c = 0;
    for (size_t i = 0; i < prev_pts_.size(); ++i) {
      //if (status[i] != 0) {
        tracked_pts.push_back(next_pts[c]);
        cv::KeyPoint kpt;
        kpt.pt = next_pts[c];
        tracked_kpts.push_back(kpt);
        ++c;
      //}
    }
    printf("tracked_pts: %lu\n", tracked_pts.size());  

    // Show the keypoint image.
    cv::Mat kpt_img = frame.rgb_img.clone();
    cv::drawKeypoints(frame.rgb_img, tracked_kpts, kpt_img, 255);
    cv::imshow("tracked", kpt_img);
    cv::waitKey(1); 

    prev_pts_ = tracked_pts;
  }

  prev_rgb_img_ = frame.rgb_img.clone();

  return false;
}

} // namespace rgbdtools
