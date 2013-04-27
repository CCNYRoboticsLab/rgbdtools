/**
 *  @file keyframe_graph_detector.cpp
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

#include "rgbdtools/graph/keyframe_graph_detector.h"

namespace rgbdtools {

KeyframeGraphDetector::KeyframeGraphDetector()
{
  srand(time(NULL));
  
  // USRF params
  n_keypoints_ = 500;
  
  // Pairwise matching params
  pairwise_matching_method_ = PAIRWISE_MATCHING_RANSAC;            
  
  // tree algorithm params            
  candidate_method_ = CANDIDATE_GENERATION_SURF_TREE;
  n_candidates_ = 10;
  k_nearest_neighbors_ = 2;
  
  // matcher params
  pairwise_matcher_index_ = PAIRWISE_MATCHER_KDTREE;
  matcher_use_desc_ratio_test_ = true;
  matcher_max_desc_ratio_ = 0.75;  // when ratio_test = true
  matcher_max_desc_dist_ = 0.5;    // when ratio_test = false
  
  // common SAC params
  sac_max_eucl_dist_sq_ = 0.03 * 0.03;
  sac_min_inliers_ = 30;  // this or more are needed
  sac_reestimate_tf_ = false;
  
  // RANSAC params
  ransac_confidence_ = 0.99;
  ransac_max_iterations_ = 1000;
  ransac_sufficient_inlier_ratio_ = 0.75;

  // output params
  verbose_ = false;     // console output
  sac_save_results_ = false;
  sac_results_path_ = std::getenv("HOME");
  
  // derived parameters
  log_one_minus_ransac_confidence_ = log(1.0 - ransac_confidence_);
}

KeyframeGraphDetector::~KeyframeGraphDetector()
{

}

void KeyframeGraphDetector::setNCandidates(int n_candidates)
{
  n_candidates_ = n_candidates;
}

void KeyframeGraphDetector::setKNearestNeighbors(int k_nearest_neighbors)
{
  k_nearest_neighbors_ = k_nearest_neighbors;
}

void KeyframeGraphDetector::setNKeypoints(int n_keypoints)
{
  n_keypoints_ = n_keypoints;
}

void KeyframeGraphDetector::setSACResultsPath(const std::string& sac_results_path)
{
  sac_results_path_ = sac_results_path;
}

void KeyframeGraphDetector::setCandidateGenerationMethod(
  CandidateGenerationMethod candidate_method)
{
  assert(candidate_method == CANDIDATE_GENERATION_BRUTE_FORCE ||
         candidate_method == CANDIDATE_GENERATION_SURF_TREE);
         
  candidate_method_ = candidate_method;
}

void KeyframeGraphDetector::setPairwiseMatchingMethod(
  PairwiseMatchingMethod pairwise_matching_method)
{
  assert(pairwise_matching_method == PAIRWISE_MATCHING_BFSAC ||
         pairwise_matching_method == PAIRWISE_MATCHING_RANSAC);
         
  pairwise_matching_method_ = pairwise_matching_method;
}

void KeyframeGraphDetector::setPairwiseMatcherIndex(
  PairwiseMatcherIndex pairwise_matcher_index)
{
  assert(pairwise_matcher_index == PAIRWISE_MATCHER_LINEAR ||
         pairwise_matcher_index == PAIRWISE_MATCHER_KDTREE);
         
  pairwise_matcher_index_ = pairwise_matcher_index;
}

void KeyframeGraphDetector::generateKeyframeAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  buildAssociationMatrix(keyframes);  
}

void KeyframeGraphDetector::prepareMatchers(
  const KeyframeVector& keyframes)
{
  if(verbose_) printf("training individual keyframe matchers ...\n");   
    
  matchers_.clear();
  matchers_.resize(keyframes.size());
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
  { 
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    cv::FlannBasedMatcher& matcher = matchers_[kf_idx];
    
    // build matcher  
    cv::Ptr<cv::flann::IndexParams> indexParams;
       
    if (pairwise_matcher_index_ == PAIRWISE_MATCHER_LINEAR)
      indexParams = new cv::flann::LinearIndexParams();
    else if (pairwise_matcher_index_ == PAIRWISE_MATCHER_KDTREE)
      indexParams = new cv::flann::KDTreeIndexParams();  

    cv::Ptr<cv::flann::SearchParams> searchParams = new cv::flann::SearchParams(32);
    
    matcher = cv::FlannBasedMatcher(indexParams, searchParams);

    // train
    std::vector<cv::Mat> descriptors_vector;
    descriptors_vector.push_back(keyframe.descriptors);
    matcher.add(descriptors_vector);
    matcher.train();    
  }
}

void KeyframeGraphDetector::prepareFeaturesForRANSAC(
  KeyframeVector& keyframes)
{
  double init_surf_threshold = 800.0;
  double min_surf_threshold = 25;

  if(verbose_) printf("preparing SURF features for matching...\n");  

  cv::SurfDescriptorExtractor extractor;
 
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
  { 
    RGBDKeyframe& keyframe = keyframes[kf_idx];

    double surf_threshold = init_surf_threshold;

    while (surf_threshold >= min_surf_threshold)
    {
      cv::SurfFeatureDetector detector(surf_threshold);
      keyframe.keypoints.clear();
      detector.detect(keyframe.rgb_img, keyframe.keypoints);

      if(verbose_)
        printf("[KF %d of %d] %d SURF keypoints detected (threshold: %.1f)\n", 
          (int)kf_idx, (int)keyframes.size(), 
          (int)keyframe.keypoints.size(), surf_threshold); 

      if ((int)keyframe.keypoints.size() < n_keypoints_)
        surf_threshold /= 2.0;
      else break;
    }

    if (sac_save_results_)
    {
      cv::Mat kp_img;
      cv::drawKeypoints(keyframe.rgb_img, keyframe.keypoints, kp_img);
      std::stringstream ss1;
      ss1 << "kp_" << kf_idx;
      cv::imwrite(sac_results_path_ + "/" + ss1.str() + ".png", kp_img);
    }

    extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
    keyframe.computeDistributions();
  }
}


void KeyframeGraphDetector::buildAssociationMatrix(
  const KeyframeVector& keyframes)
{
  prepareMatchers(keyframes);
  
  // 1. Create the candidate matrix
  buildCandidateMatrix(keyframes);

  // 2. Perfrom pairwise matching for all candidates
  buildCorrespondenceMatrix(keyframes); 
  
  // 3. Threshold the correspondence matrix to find the associations
  thresholdMatrix(correspondence_matrix_, association_matrix_, sac_min_inliers_);
}

void KeyframeGraphDetector::buildCandidateMatrix(
  const KeyframeVector& keyframes)
{
  if (candidate_method_ == CANDIDATE_GENERATION_BRUTE_FORCE) // brute-force
  {
    // create a candidate matrix which considers all posiible combinations
    int size = keyframes.size();
    candidate_matrix_ = cv::Mat::ones(size, size, CV_8UC1);
  }
  else if (candidate_method_ == CANDIDATE_GENERATION_SURF_TREE) // tree-based
  {
    // build a math knn matrix using a kdtree
    buildMatchMatrixSurfTree(keyframes);
     
    // keep only the top n candidates
    buildCandidateMatrixSurfTree();
  }
}

/** @brief Builds a matrix of nearest neighbor matches between keyframes 
* using a kdtree
* 
* match_matrix[query, train] = X correspondences
*/  
void KeyframeGraphDetector::buildMatchMatrixSurfTree(
  const KeyframeVector& keyframes)
{
  unsigned int kf_size = keyframes.size(); 
  
  // train matcher from all the features
  cv::FlannBasedMatcher matcher;
  trainSURFMatcher(keyframes, matcher);

  // lookup per frame
  if(verbose_) printf("Keyframe lookups...\n");

  match_matrix_ = cv::Mat::zeros(kf_size, kf_size, CV_32FC1);
  for (unsigned int kf_idx = 0; kf_idx < kf_size; ++kf_idx)
  {
    if(verbose_)
      printf("[KF %d of %d]: Computing SURF neighbors\n", (int)kf_idx, (int)kf_size);
    const RGBDFrame& keyframe = keyframes[kf_idx];

    // find k nearest matches for each feature in the keyframe
    std::vector<std::vector<cv::DMatch> > matches_vector;
    matcher.knnMatch(keyframe.descriptors, matches_vector, k_nearest_neighbors_);

    // create empty bins vector of Pairs <count, image_index>
    std::vector<std::pair<int, int> > bins;
    bins.resize(kf_size);
    for (unsigned int b = 0; b < bins.size(); ++b) 
      bins[b] = std::pair<int, int>(0, b);

    // fill out bins with match indices
    for (unsigned int j = 0; j < matches_vector.size(); ++j)
    {
      std::vector<cv::DMatch>& matches = matches_vector[j];
      for (unsigned int k = 0; k < matches.size(); ++k)
      {
        bins[matches[k].imgIdx].first++;
      }
    }
    
    for (unsigned int b = 0; b < kf_size; ++b)
    {
      unsigned int index_a = kf_idx;
      unsigned int index_b = bins[b].second;
      int corresp_count = bins[b].first;
      
      if (index_a != index_b)
        match_matrix_.at<float>(index_a, index_b) = corresp_count;
    }
  }
}

/** @brief Takes in a matrix of matches from a SURF tree (match_matrix_)
 * and marks the top n_candidates in each row into (candidate_matrix)
 */  
void KeyframeGraphDetector::buildCandidateMatrixSurfTree()
{
  // check for square matrix
  assert(match_matrix_.rows == match_matrix_.cols);
  
  // check for validity of n_candidates argument
  int size = match_matrix_.rows;
  assert(n_candidates_ <= size);
  
  // initialize candidate matrix as all 0
  candidate_matrix_ = cv::Mat::eye(match_matrix_.size(), CV_8UC1);
  
  for (int v = 0; v < match_matrix_.rows; ++v)
  {
    // create a vector from the current row
    std::vector<std::pair<int, int> > values(match_matrix_.cols);
    for (int u = 0; u < match_matrix_.cols; ++u)
    {
      int value = match_matrix_.at<float>(v,u);
      values[u] =  std::pair<int, int>(value, u);
    }
    
    // sort the vector based on values, highest first
    std::sort(values.begin(), values.end(), std::greater<std::pair<int, int> >());

    // mark 1 for the top n_candidates, if > 0
    for (int u = 0; u < n_candidates_; ++u)
    {
      if (values[u].first == 0) continue;
      unsigned int uc = values[u].second;     
      candidate_matrix_.at<uint8_t>(v,uc) = 1;
    }
  }
}

void KeyframeGraphDetector::buildCorrespondenceMatrix(
  const KeyframeVector& keyframes)
{
  // check for square matrix
  assert(candidate_matrix_.rows == candidate_matrix_.cols);
  int size = candidate_matrix_.rows;    
  
  // initialize correspondence matrix    
  correspondence_matrix_ = cv::Mat::zeros(size, size, CV_32FC1);
  
  for (int kf_idx_a = 0; kf_idx_a < size; ++kf_idx_a)
  for (int kf_idx_b = 0; kf_idx_b < size; ++kf_idx_b)
  {
    const RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    const RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];
    
    if (kf_idx_a == kf_idx_b)
    {
      // self-association
      // @todo actually this should only account for the "valid" keypoints
      correspondence_matrix_.at<float>(kf_idx_a, kf_idx_a) = keyframe_a.keypoints.size();
    }
    else
    {
      // skip non-candidates
      if (candidate_matrix_.at<uint8_t>(kf_idx_a, kf_idx_b) != 0)
      {
        if(verbose_) printf("[RANSAC %d to %d]: ", kf_idx_a, kf_idx_b);

        std::vector<cv::DMatch> inlier_matches;

        // perform ransac matching, b onto a
        Eigen::Matrix4f transformation;

        // train, query
        int iterations = pairwiseMatching(
          kf_idx_b, kf_idx_a, keyframes, inlier_matches, transformation);
        
        if (inlier_matches.size() >= sac_min_inliers_)
        {
          if(verbose_) 
            printf("pass [%d][%d]\n", iterations, (int)inlier_matches.size());
          
          if (sac_save_results_)
          {
            cv::Mat img_matches;
            cv::drawMatches(keyframe_a.rgb_img, keyframe_a.keypoints, 
                            keyframe_b.rgb_img, keyframe_b.keypoints, 
                            inlier_matches, img_matches);

            std::stringstream ss1;
            ss1 << kf_idx_a << "_to_" << kf_idx_b;
            cv::imwrite(sac_results_path_ + "/" + ss1.str() + ".png", img_matches);
          }
        }
        else
        {
          if(verbose_)
            printf("fail [%d][%d]\n", iterations, (int)inlier_matches.size());     
        }
           
        correspondence_matrix_.at<float>(kf_idx_a, kf_idx_b) = inlier_matches.size();
      }
    }
  }
}  

// frame_a = train, frame_b = query
void KeyframeGraphDetector::getCandidateMatches(
  const RGBDFrame& frame_a, const RGBDFrame& frame_b,
  cv::FlannBasedMatcher& matcher,
  DMatchVector& candidate_matches)
{
  // **** build candidate matches ***********************************
  // assumes detectors and distributions are computed
  // establish all matches from b to a

  if (matcher_use_desc_ratio_test_)
  {
    std::vector<DMatchVector> all_matches2;
    
    matcher.knnMatch(
      frame_b.descriptors, all_matches2, 2);

    for (unsigned int m_idx = 0; m_idx < all_matches2.size(); ++m_idx)
    {
      const cv::DMatch& match1 = all_matches2[m_idx][0];
      const cv::DMatch& match2 = all_matches2[m_idx][1];
      
      double ratio =  match1.distance / match2.distance;
      
      // remove bad matches - ratio test, valid keypoints
      if (ratio < matcher_max_desc_ratio_)
      {
        int idx_b = match1.queryIdx;
        int idx_a = match1.trainIdx; 

        if (frame_a.kp_valid[idx_a] && frame_b.kp_valid[idx_b])
          candidate_matches.push_back(match1);
      }
    }
  }
  else
  {
    DMatchVector all_matches;
    
    // query, train
    matcher.match(
      frame_b.descriptors, all_matches);

    for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
    {
      const cv::DMatch& match = all_matches[m_idx];

      // remove bad matches - descriptor distance, valid keypoints
      if (match.distance < matcher_max_desc_dist_)
      {      
        int idx_b = match.queryIdx;
        int idx_a = match.trainIdx; 
        
        if (frame_a.kp_valid[idx_a] && frame_b.kp_valid[idx_b])
          candidate_matches.push_back(match);
      }
    }
  }
}
  
// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatching(
  int kf_idx_a, int kf_idx_b,
  const KeyframeVector& keyframes,
  DMatchVector& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  int iterations;
  
  if (pairwise_matching_method_ == PAIRWISE_MATCHING_RANSAC)
  {
    iterations = pairwiseMatchingRANSAC(
      kf_idx_a, kf_idx_b, keyframes, best_inlier_matches, best_transformation);
  }
  else if (pairwise_matching_method_ == PAIRWISE_MATCHING_BFSAC)
  {
    iterations = pairwiseMatchingBFSAC(
      kf_idx_a, kf_idx_b, keyframes, best_inlier_matches, best_transformation);
  }
  
  return iterations;
}
  
// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatchingBFSAC(
  int kf_idx_a, int kf_idx_b,
  const KeyframeVector& keyframes,
  DMatchVector& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // constants
  int min_sample_size = 3;

  const RGBDFrame& frame_a = keyframes[kf_idx_a];
  const RGBDFrame& frame_b = keyframes[kf_idx_b];
  cv::FlannBasedMatcher& matcher = matchers_[kf_idx_a];
  
  // find candidate matches
  DMatchVector candidate_matches;
  getCandidateMatches(frame_a, frame_b, matcher, candidate_matches);
  if (candidate_matches.size() < min_sample_size) return 0;
  
  // **** build 3D features for SVD ********************************

  PointCloudFeature features_a, features_b;

  features_a.resize(candidate_matches.size());
  features_b.resize(candidate_matches.size());

  for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    PointFeature& p_a = features_a[m_idx];
    p_a.x = frame_a.kp_means[idx_a](0,0);
    p_a.y = frame_a.kp_means[idx_a](1,0);
    p_a.z = frame_a.kp_means[idx_a](2,0);

    PointFeature& p_b = features_b[m_idx];
    p_b.x = frame_b.kp_means[idx_b](0,0);
    p_b.y = frame_b.kp_means[idx_b](1,0);
    p_b.z = frame_b.kp_means[idx_b](2,0);
  }
  
   // **** main BFSAC loop ****************************************
  
  TransformationEstimationSVD svd;
  Eigen::Matrix4f transformation; // transformation used inside loop
  best_inlier_matches.clear();
  int iterations = 0;
  
  for (int i = 0;   i < candidate_matches.size(); ++i)
  for (int j = i+1; j < candidate_matches.size(); ++j)
  for (int k = j+1; k < candidate_matches.size(); ++k)
  {
    // build the Minimum Sample Set of 3 matches (index vector)
    IntVector inlier_idx;
    inlier_idx.push_back(i);
    inlier_idx.push_back(j);
    inlier_idx.push_back(k);
    
    // build the Minimum Sample Set of 3 matches (actual matches)
    std::vector<cv::DMatch> inlier_matches;       
    inlier_matches.push_back(candidate_matches[i]);
    inlier_matches.push_back(candidate_matches[j]);
    inlier_matches.push_back(candidate_matches[k]);
    
    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      features_b, inlier_idx,
      features_a, inlier_idx,
      transformation);
    
    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_b_tf;
    pcl::transformPointCloud(features_b, features_b_tf, transformation);

    for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
    {
      // euclidedan distance test
      const PointFeature& p_a = features_a[m_idx];
      const PointFeature& p_b = features_b_tf[m_idx];
      float eucl_dist_sq = distEuclideanSq(p_a, p_b);
      
      if (eucl_dist_sq < sac_max_eucl_dist_sq_)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);
        
        // reestimate transformation from all inliers
        if (sac_reestimate_tf_)
        {
          svd.estimateRigidTransformation(
            features_b, inlier_idx,
            features_a, inlier_idx,
            transformation);
          pcl::transformPointCloud(features_b, features_b_tf, transformation);
        }
      }
    }
    
    // check if inliers are better than the best model so far
    if (inlier_matches.size() > best_inlier_matches.size())
    {
      svd.estimateRigidTransformation(
        features_b, inlier_idx,
        features_a, inlier_idx,
        transformation);

      best_transformation = transformation;
      best_inlier_matches = inlier_matches;
    }
    
    iterations++;
  }
  
  return iterations;
}  
  
// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatchingRANSAC(
  int kf_idx_a, int kf_idx_b,
  const KeyframeVector& keyframes,
  DMatchVector& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // constants
  int min_sample_size = 3; 

  // find candidate matches
  const RGBDFrame& frame_a = keyframes[kf_idx_a];
  const RGBDFrame& frame_b = keyframes[kf_idx_b];
  cv::FlannBasedMatcher& matcher = matchers_[kf_idx_a];
  
  DMatchVector candidate_matches;
  getCandidateMatches(frame_a, frame_b, matcher, candidate_matches);
  
  // check if enough matches are present
  if (candidate_matches.size() < min_sample_size)  return 0;
  if (candidate_matches.size() < sac_min_inliers_) return 0;
  
  // **** build 3D features for SVD ********************************

  PointCloudFeature features_a, features_b;

  features_a.resize(candidate_matches.size());
  features_b.resize(candidate_matches.size());

  for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    PointFeature& p_a = features_a[m_idx];
    p_a.x = frame_a.kp_means[idx_a](0,0);
    p_a.y = frame_a.kp_means[idx_a](1,0);
    p_a.z = frame_a.kp_means[idx_a](2,0);

    PointFeature& p_b = features_b[m_idx];
    p_b.x = frame_b.kp_means[idx_b](0,0);
    p_b.y = frame_b.kp_means[idx_b](1,0);
    p_b.z = frame_b.kp_means[idx_b](2,0);
  }

  // **** main RANSAC loop ****************************************
  
  TransformationEstimationSVD svd;
  Eigen::Matrix4f transformation; // transformation used inside loop
  best_inlier_matches.clear();
  int iteration = 0;
  
  std::set<int> mask;
  
  while(true)
  //for (iteration = 0; iteration < ransac_max_iterations_; ++iteration)
  {   
    // generate random indices
    IntVector sample_idx;
    get3RandomIndices(candidate_matches.size(), mask, sample_idx);
    
    // build initial inliers from random indices
    IntVector inlier_idx;
    std::vector<cv::DMatch> inlier_matches;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
      inlier_matches.push_back(candidate_matches[m_idx]);
    } 
    
    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      features_b, inlier_idx,
      features_a, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_b_tf;
    pcl::transformPointCloud(features_b, features_b_tf, transformation);

    for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
    {
      const PointFeature& p_a = features_a[m_idx];
      const PointFeature& p_b = features_b_tf[m_idx];

      float eucl_dist_sq = distEuclideanSq(p_a, p_b);
      
      if (eucl_dist_sq < sac_max_eucl_dist_sq_)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);

        // reestimate transformation from all inliers
        if (sac_reestimate_tf_)
        {
          svd.estimateRigidTransformation(
            features_b, inlier_idx,
            features_a, inlier_idx,
            transformation);
          pcl::transformPointCloud(features_b, features_b_tf, transformation);
        }
      }
    }
    
    // check if inliers are better than the best model so far
    if (inlier_matches.size() > best_inlier_matches.size())
    {
      svd.estimateRigidTransformation(
        features_b, inlier_idx,
        features_a, inlier_idx,
        transformation);

      best_transformation = transformation;
      best_inlier_matches = inlier_matches;
    }

    double best_inlier_ratio = (double) best_inlier_matches.size() / 
                               (double) candidate_matches.size();
    
    // **** termination: iterations + inlier ratio
    if(best_inlier_matches.size() < sac_min_inliers_)
    {
      if (iteration >= ransac_max_iterations_) break;   
    }                     
    // **** termination: confidence ratio test
    else
    {     
      double h = log_one_minus_ransac_confidence_ / 
                log(1.0 - pow(best_inlier_ratio, min_sample_size));
                
      if (iteration > (int)(h+1)) break;
    }
    
    iteration++;
  }
  
  return iteration;
}

/*
void KeyframeGraphDetector::visualOdometryAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  for (unsigned int kf_idx_a = 0; kf_idx_a < keyframes.size()-1; ++kf_idx_a)
  {
    // determine second index, NO wrap-around
    unsigned int kf_idx_b = kf_idx_a + 1;

    // set up the two keyframe references
    RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

    // create an association object
    KeyframeAssociation association;
    
    association.type = KeyframeAssociation::VO;    
    association.kf_idx_a = kf_idx_a;
    association.kf_idx_b = kf_idx_b;
    association.a2b = keyframe_a.pose.inverse() * keyframe_b.pose;

    associations.push_back(association);
  }
}
*/



/*
void KeyframeGraphDetector::pairwiseMatchingRANSAC(
  RGBDFrame& frame_a, RGBDFrame& frame_b,
  double max_eucl_dist_sq, 
  double max_desc_dist,
  double sufficient_inlier_ratio,
  std::vector<cv::DMatch>& all_matches,
  std::vector<cv::DMatch>& best_inlier_matches,
  AffineTransform& best_transformation)
{
  // constants
  int min_sample_size = 3;

  cv::FlannBasedMatcher matcher;          // for SURF
  TransformationEstimationSVD svd;

  // **** build candidate matches ***********************************
  
  // assumes detectors and distributions are computed
  // establish all matches from b to a
  matcher.match(frame_b.descriptors, frame_a.descriptors, all_matches);

  // remove bad matches - too far away in descriptor space,
  //                    - nan, too far, or cov. too big
  std::vector<cv::DMatch> candidate_matches;
  for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = all_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    if (match.distance < max_desc_dist && 
        frame_a.kp_valid[idx_a] && 
        frame_b.kp_valid[idx_b])
    {
      candidate_matches.push_back(all_matches[m_idx]);
    }
    
    //printf("%f %s %s\n", match.distance, 
    //  frame_a.kp_valid[idx_a]?"t":"f", 
    //  frame_b.kp_valid[idx_b]?"t":"f");
  }

  int size = candidate_matches.size();

  if (size < min_sample_size) return;
  
  // **** build 3D features for SVD ********************************

  PointCloudFeature features_a, features_b;

  features_a.resize(size);
  features_b.resize(size);

  for (int m_idx = 0; m_idx < size; ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    PointFeature& p_a = features_a[m_idx];
    p_a.x = frame_a.kp_means[idx_a](0,0);
    p_a.y = frame_a.kp_means[idx_a](1,0);
    p_a.z = frame_a.kp_means[idx_a](2,0);

    PointFeature& p_b = features_b[m_idx];
    p_b.x = frame_b.kp_means[idx_b](0,0);
    p_b.y = frame_b.kp_means[idx_b](1,0);
    p_b.z = frame_b.kp_means[idx_b](2,0);
  }

  // **** main RANSAC loop ****************************************
  
  int best_n_inliers = 0;
  Eigen::Matrix4f transformation; // transformation used inside loop
  
  for (int iteration = 0; iteration < max_ransac_iterations_; ++iteration)
  {   
    // generate random indices
    IntVector sample_idx;
    getRandomIndices(min_sample_size, size, sample_idx);
    
    // build initial inliers from random indices
    IntVector inlier_idx;
    std::vector<cv::DMatch> inlier_matches;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
      inlier_matches.push_back(candidate_matches[m_idx]);
    } 
    
    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      features_b, inlier_idx,
      features_a, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_b_tf;
    pcl::transformPointCloud(features_b, features_b_tf, transformation);

    for (int m_idx = 0; m_idx < size; ++m_idx)
    {
      const PointFeature& p_a = features_a[m_idx];
      const PointFeature& p_b = features_b_tf[m_idx];

      float dist_sq = distEuclideanSq(p_a, p_b);
      
      if (dist_sq < max_eucl_dist_sq)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);

        // reestimate transformation from all inliers
        svd.estimateRigidTransformation(
          features_b, inlier_idx,
          features_a, inlier_idx,
          transformation);
        pcl::transformPointCloud(features_b, features_b_tf, transformation);
      }
    }
    
    // check if inliers are better than the best model so far
    int n_inliers = inlier_idx.size();

    if (n_inliers > best_n_inliers)
    {
      svd.estimateRigidTransformation(
        features_b, inlier_idx,
        features_a, inlier_idx,
        transformation);

      best_n_inliers = n_inliers;
      best_transformation = AffineTransform(transformation);
      best_inlier_matches = inlier_matches;
    }

    // check if we reached ratio termination criteria
    double inlier_ratio = (double) n_inliers / (double) size;

    if (inlier_ratio > sufficient_inlier_ratio)
      break;
  }
}
*/

/*
void KeyframeGraphDetector::trainMatcher(
  const KeyframeVector& keyframes,
  cv::FlannBasedMatcher& matcher)
{
  printf("Building aggregate feature vector...\n"); 
  std::vector<cv::Mat> descriptors_vector;
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    descriptors_vector.push_back(keyframe.descriptors);
  }
  matcher.add(descriptors_vector);

  printf("Training feature matcher...\n");
  matcher.train();
}
*/

/*
void KeyframeGraphDetector::treeAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  // extra params
  double sufficient_ransac_inlier_ratio = 1.0;
  
  // train matcher from all teh features
  cv::FlannBasedMatcher matcher;
  trainMatcher(keyframes, matcher);

  // lookup per frame
  printf("Keyframe lookups...\n");

  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    printf("[KF %d of %d]:\n", (int)kf_idx, (int)keyframes.size());
    RGBDFrame& keyframe = keyframes[kf_idx];

    // find k nearest matches for each feature in the keyframe
    std::vector<std::vector<cv::DMatch> > matches_vector;
    matcher.knnMatch(keyframe.descriptors, matches_vector, k_nearest_neighbors_);

    // create empty bins vector of Pairs <count, image_index>
    std::vector<std::pair<int, int> > bins;
    bins.resize(keyframes.size());
    for (unsigned int b = 0; b < bins.size(); ++b) 
      bins[b] = std::pair<int, int>(0, b);

    // fill out bins with match indices
    for (unsigned int j = 0; j < matches_vector.size(); ++j)
    {
      std::vector<cv::DMatch>& matches = matches_vector[j];
      for (unsigned int k = 0; k < matches.size(); ++k)
      {
        bins[matches[k].imgIdx].first++;
      }
    }
  
    // sort - highest counts first
    std::sort(bins.begin(), bins.end(), std::greater<std::pair<int, int> >());

    // output results
    printf(" - best matches: ");
    for (int b = 0; b < n_ransac_candidates_; ++b)
      printf("[%d(%d)] ", bins[b].second, bins[b].first);
    printf("\n");

    // **** find top X candidates
    
    printf(" - candidate matches: ");
    IntVector ransac_candidates;
    int n_ransac_candidates_found = 0;
    for (unsigned int b = 0; b < bins.size(); ++b)
    {
      unsigned int index_a = kf_idx;
      unsigned int index_b = bins[b].second;
      int corresp_count = bins[b].first;

      // test for order consistence
      // and for minimum number of keypoints
      if (index_b > index_a && 
          corresp_count >= min_ransac_inliers_)
      {
        ransac_candidates.push_back(index_b);
        ++n_ransac_candidates_found;
        printf("[%d(%d)] ", index_b, corresp_count);
      }

      if (n_ransac_candidates_found >= n_ransac_candidates_) break;
    }
    printf("\n");

    // **** test top X candidates using RANSAC

    for (unsigned int rc = 0; rc < ransac_candidates.size(); ++rc)
    {
      unsigned int kf_idx_a = kf_idx;
      unsigned int kf_idx_b = ransac_candidates[rc];

      RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
      RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

      std::vector<cv::DMatch> all_matches;
      std::vector<cv::DMatch> inlier_matches;

      // perform ransac matching, b onto a
      AffineTransform transformation;

      pairwiseMatchingRANSAC(keyframe_a, keyframe_b, 
        max_corresp_dist_eucl_sq_, max_corresp_dist_desc_, 
        sufficient_ransac_inlier_ratio,
        all_matches, inlier_matches, transformation);
      
      if ((int)inlier_matches.size() >= min_ransac_inliers_)
      {
        if (save_ransac_results_)
        {
          cv::Mat img_matches;
          cv::drawMatches(keyframe_b.rgb_img, keyframe_b.keypoints, 
                          keyframe_a.rgb_img, keyframe_a.keypoints, 
                          inlier_matches, img_matches);

          std::stringstream ss1;
          ss1 << kf_idx_a << "_to_" << kf_idx_b;
          cv::imwrite(ransac_results_path_ + "/" + ss1.str() + ".png", img_matches);
        }

        printf(" - RANSAC %d -> %d: PASS\n", kf_idx_a, kf_idx_b);

        // create an association object
        KeyframeAssociation association;
        association.type = KeyframeAssociation::RANSAC;
        association.kf_idx_a = kf_idx_a;
        association.kf_idx_b = kf_idx_b;
        association.matches  = inlier_matches;
        association.a2b = transformation;
        associations.push_back(association);      
      }
      else  
        printf(" - RANSAC %d -> %d: FAIL\n", kf_idx_a, kf_idx_b);
    }
  }
}
*/

} // namespace rgbdtools
