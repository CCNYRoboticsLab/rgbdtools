/**
 *  @file motion_estimation_pairwise_ransac.cpp
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

#include "rgbdtools/registration/motion_estimation_pairwise_ransac.h"

namespace rgbdtools {

MotionEstimationPairwiseRANSAC::MotionEstimationPairwiseRANSAC():
  MotionEstimation(),
  initialized_(false)
{
  sac_min_inliers_ = 30;
  sac_max_eucl_dist_sq_ = pow(0.03, 2.0);
  sac_reestimate_tf_ = false;
  ransac_max_iterations_ = 200;
  ransac_confidence_ = 0.999;
  matcher_use_desc_ratio_test_ = true;
  matcher_max_desc_ratio_ = 0.8;
  matcher_max_desc_dist_ = 50.0;
    
  // derived parameters
  log_one_minus_ransac_confidence_ = log(1.0 - ransac_confidence_);
   
  f2b_.setIdentity();

  //cv::namedWindow("sacmatches", 0);
}

MotionEstimationPairwiseRANSAC::~MotionEstimationPairwiseRANSAC()
{

}

bool MotionEstimationPairwiseRANSAC::getMotionEstimationImpl(
  RGBDFrame& frame,
  const AffineTransform& prediction,
  AffineTransform& motion)
{  
  bool result;

  if (!initialized_)
  {
    initialized_ = true;
    result = false;
  }
  else
  {
    DMatchVector sac_matches;
    Eigen::Matrix4f sac_transformation;
    int ransac_iterations = pairwiseMatchingRANSAC(
      frame, prev_frame_, sac_matches, sac_transformation);
         
    //printf("ransac_iterations: %d\n", ransac_iterations);
    if (sac_matches.size() > sac_min_inliers_)
    {
      // express the motion in the fixed frame
      AffineTransform motion_cam(sac_transformation);
      AffineTransform f2c = f2b_ * b2c_;
      AffineTransform f2c_new = f2c * motion_cam;      
      motion = f2c_new * f2c.inverse();
   
      // update the pose of the base frame
      f2b_ = f2c_new * b2c_.inverse();
   
      //delete prev_frame_;
      
      result = true;
      
      /*
      // visualize
      cv::Mat img_matches;
      cv::drawMatches(frame.rgb_img, frame.keypoints, 
        prev_frame_.rgb_img, prev_frame_.keypoints, 
        sac_matches, img_matches);
      cv::imshow("sacmatches", img_matches);
      cv::waitKey(1);*/
    }
    else
    {
      /*
      // visualize
      cv::Mat img_matches;
      cv::drawMatches(frame.rgb_img, frame.keypoints, 
        prev_frame_.rgb_img, prev_frame_.keypoints, 
        DMatchVector(), img_matches);
      cv::imshow("sacmatches", img_matches);
      cv::waitKey(1);*/
     
      std::cerr << "RANSAC alignment failed. Using identity transform." << std::endl;
      result = false;
    }
  }
  
  prev_frame_ = RGBDFrame(frame);

  return result;
}
       
int MotionEstimationPairwiseRANSAC::pairwiseMatchingRANSAC(
  const RGBDFrame& frame_q, const RGBDFrame& frame_t,
  DMatchVector& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // constants
  int min_sample_size = 3; 
   
  // **** establish matches ***************************************
   
  // build matcher  
  cv::BFMatcher matcher(cv::NORM_HAMMING, false);

  // train
  //printf("training\n");
  std::vector<cv::Mat> descriptors_vector;
  descriptors_vector.push_back(frame_t.descriptors);
  matcher.add(descriptors_vector);
  matcher.train();    

  // get matchers 
  DMatchVector candidate_matches;
  getCandidateMatches(frame_q, frame_t, matcher, candidate_matches);
  
  //printf("candidate_matches.size(): %d\n", (int)candidate_matches.size());
  
  // check if enough matches are present
  if (candidate_matches.size() < min_sample_size)  return 0;
  if (candidate_matches.size() < sac_min_inliers_) return 0;
  
  // **** build 3D features for SVD ********************************

  PointCloudFeature features_t, features_q;

  features_t.resize(candidate_matches.size());
  features_q.resize(candidate_matches.size());

  for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_q = match.queryIdx;
    int idx_t = match.trainIdx; 

    PointFeature& p_t = features_t[m_idx];
    p_t.x = frame_t.kp_means[idx_t](0,0);
    p_t.y = frame_t.kp_means[idx_t](1,0);
    p_t.z = frame_t.kp_means[idx_t](2,0);

    PointFeature& p_q = features_q[m_idx];
    p_q.x = frame_q.kp_means[idx_q](0,0);
    p_q.y = frame_q.kp_means[idx_q](1,0);
    p_q.z = frame_q.kp_means[idx_q](2,0);
  }

  // **** main RANSAC loop ****************************************
  
  TransformationEstimationSVD svd;
  Eigen::Matrix4f transformation; // transformation used inside loop
  best_inlier_matches.clear(); 
  std::set<int> mask;
  
  int iteration = 0;  
  for (iteration = 0; iteration < ransac_max_iterations_; ++iteration)
  {   
    // generate random indices
    IntVector sample_idx;
    get3RandomIndices(candidate_matches.size(), mask, sample_idx);
    
    // build initial inliers from random indices
    IntVector init_inlier_idx;
    std::vector<cv::DMatch> init_inlier_matches;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      init_inlier_idx.push_back(m_idx);
      init_inlier_matches.push_back(candidate_matches[m_idx]);
    } 
    
    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      features_q, init_inlier_idx,
      features_t, init_inlier_idx,
      transformation);

    // transformation rejection
    
    /*
    double angular, linear;
    AffineTransform temp(transformation);
    getTfDifference(temp, linear, angular);
    if (linear > 0.10 || angular > 5.0 * M_PI / 180.0)
      continue;
    */
    
    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_q_tf;
    pcl::transformPointCloud(features_q, features_q_tf, transformation);

    IntVector inlier_idx;
    std::vector<cv::DMatch> inlier_matches;
    
    for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
    {
      // euclidedan distance test
      const PointFeature& p_t = features_t[m_idx];
      const PointFeature& p_q = features_q_tf[m_idx];
      float eucl_dist_sq = distEuclideanSq(p_t, p_q);
      
      if (eucl_dist_sq < sac_max_eucl_dist_sq_)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);

        // reestimate transformation from all inliers
        if (sac_reestimate_tf_)
        {
          svd.estimateRigidTransformation(
            features_q, inlier_idx,
            features_t, inlier_idx,
            transformation);
          pcl::transformPointCloud(features_q, features_q_tf, transformation);
        }
      }
    }
    
    // check if inliers are better than the best model so far
    if (inlier_matches.size() > best_inlier_matches.size())
    {
      svd.estimateRigidTransformation(
        features_q, inlier_idx,
        features_t, inlier_idx,
        transformation);

      best_transformation = transformation;
      best_inlier_matches = inlier_matches;
    }

    double best_inlier_ratio = (double) best_inlier_matches.size() / 
                               (double) candidate_matches.size();
    
    // **** early termination: iterations + inlier ratio
    if(best_inlier_matches.size() >= sac_min_inliers_)
    {
      double h = log_one_minus_ransac_confidence_ / 
                log(1.0 - pow(best_inlier_ratio, min_sample_size));
                
      if (iteration > (int)(h+1)) break;
    }
  }
  
  //printf("best_inlier_matches.size(): %d\n", (int)best_inlier_matches.size());
  
  return iteration;
}

// frame_a = train, frame_b = query
void MotionEstimationPairwiseRANSAC::getCandidateMatches(
  const RGBDFrame& frame_q, const RGBDFrame& frame_t, 
  cv::DescriptorMatcher& matcher,
  DMatchVector& candidate_matches)
{
  // **** build candidate matches ***********************************
  // assumes detectors and distributions are computed
  // establish all matches from b to a

  if (matcher_use_desc_ratio_test_)
  {
    std::vector<DMatchVector> all_matches2;
    
    matcher.knnMatch(
      frame_q.descriptors, all_matches2, 2);

    for (unsigned int m_idx = 0; m_idx < all_matches2.size(); ++m_idx)
    {
      const cv::DMatch& match1 = all_matches2[m_idx][0];
      const cv::DMatch& match2 = all_matches2[m_idx][1];
      
      double ratio = match1.distance / match2.distance;
      
      // remove bad matches - ratio test, valid keypoints
      if (ratio < matcher_max_desc_ratio_)
      {
        int idx_q = match1.queryIdx;
        int idx_t = match1.trainIdx; 

        if (frame_t.kp_valid[idx_t] && frame_q.kp_valid[idx_q])
          candidate_matches.push_back(match1);
      }
    }
  }
  else
  {
    DMatchVector all_matches;
    
    matcher.match(
      frame_q.descriptors, all_matches);

    for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
    {
      const cv::DMatch& match = all_matches[m_idx];

      // remove bad matches - descriptor distance, valid keypoints
      if (match.distance < matcher_max_desc_dist_)
      {      
        int idx_q = match.queryIdx;
        int idx_t = match.trainIdx; 
        
        if (frame_t.kp_valid[idx_t] && frame_q.kp_valid[idx_q])
          candidate_matches.push_back(match);
      }
    }
  }
}

} // namespace rgbdtools
