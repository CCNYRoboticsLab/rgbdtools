/**
 *  @file motion_estimation_pairwise_ransac.h
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

#ifndef RGBDTOOLS_MOTION_ESTIMATION_PAIRWISE_RANSAC_H
#define RGBDTOOLS_MOTION_ESTIMATION_PAIRWISE_RANSAC_H

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "rgbdtools/types.h"
#include "rgbdtools/registration/motion_estimation.h"
#include "rgbdtools/features/orb_detector.h"

namespace rgbdtools {

/** @brief Motion estimation based on aligning sparse features
 * against a persistent, dynamic model.
 * 
 * The model is build from incoming features through a Kalman Filter
 * update step.
 */  
class MotionEstimationPairwiseRANSAC: public MotionEstimation
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief Constructor from ROS noehandles
     * @param nh the public nodehandle
     * @param nh_private the private notehandle
     */
    MotionEstimationPairwiseRANSAC();
    
    /** @brief Default destructor
     */    
    virtual ~MotionEstimationPairwiseRANSAC();

    /** @brief Main method for estimating the motion given an RGBD frame
     * @param frame the current RGBD frame
     * @param prediction the predicted motion (currently ignored)
     * @param motion the (output) incremental motion, wrt the fixed frame
     * @retval true the motion estimation was successful
     * @retval false the motion estimation failed
     */
    bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const AffineTransform& prediction,
      AffineTransform& motion);

    void setFeatureDetector(FeatureDetectorPtr detector) {detector_ = detector;}
    
  private:

    bool initialized_;

    RGBDFrame prev_frame_;

    FeatureDetectorPtr detector_;
       
    AffineTransform f2b_; ///< Transform from fixed to moving frame

    // parameters
    
    int sac_min_inliers_;
    double sac_max_eucl_dist_sq_;
    bool sac_reestimate_tf_;
    int ransac_max_iterations_;
    double ransac_confidence_;
    double log_one_minus_ransac_confidence_;
    bool matcher_use_desc_ratio_test_;
    double matcher_max_desc_ratio_;
    double matcher_max_desc_dist_; 
    
    int pairwiseMatchingRANSAC(
      const RGBDFrame& frame_q, const RGBDFrame& frame_t,
      DMatchVector& best_inlier_matches,
      Eigen::Matrix4f& best_transformation);
     
    void getCandidateMatches(
      const RGBDFrame& frame_q, const RGBDFrame& frame_t, 
      cv::DescriptorMatcher& matcher,
      DMatchVector& candidate_matches);
      
    void getRandomIndices(
      int k, int n, IntVector& output)
    {
      while ((int)output.size() < k)
      {
        int random_number = rand() % n;
        bool duplicate = false;    

        for (unsigned int i = 0; i < output.size(); ++i)
        {
          if (output[i] == random_number)
          {
            duplicate = true;
            break;
          }
        }

        if (!duplicate)
          output.push_back(random_number);
      }
    }

    void get3RandomIndices(
      int n, std::set<int>& mask, IntVector& output)
    {
      int key;
      
      while(true)
      {
        output.clear();
        getRandomIndices(3, n, output);
        
        // calculate a key based on the 3 random indices
        key = output[0] * n * n + output[1] * n + output[2];
               
        //printf("%d %d %d\n", output[0], output[1], output[2]);
        
        // value not present in mask
        if (mask.find(key) == mask.end())
          break;
      }

      mask.insert(key);
    }
    
    double distEuclideanSq(const PointFeature& a, const PointFeature& b)
    {
      float dx = a.x - b.x;
      float dy = a.y - b.y;
      float dz = a.z - b.z;
      return dx*dx + dy*dy + dz*dz;
    }
};

} // namespace rgbdtools

#endif // RGBDTOOLS_MOTION_ESTIMATION_PAIRWISE_RANSAC_H

