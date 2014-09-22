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
  n_keypoints_ = 600;
  init_surf_threshold_ = 400;
  
  // Pairwise matching params
  pairwise_matching_method_ = PAIRWISE_MATCHING_RANSAC;            
  
  // tree algorithm params            
  candidate_method_ = CANDIDATE_GENERATION_SURF_TREE;
  n_candidates_ = 10;
  k_nearest_neighbors_ = 4;
  
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
  
  // derived parameters
  log_one_minus_ransac_confidence_ = log(1.0 - ransac_confidence_);
  
  setOutputPath(std::getenv("HOME"));
}

KeyframeGraphDetector::~KeyframeGraphDetector()
{

}

void KeyframeGraphDetector::setSACReestimateTf(bool sac_reestimate_tf)
{
  sac_reestimate_tf_ = sac_reestimate_tf;
}

void KeyframeGraphDetector::setMatcherUseDescRatioTest(bool matcher_use_desc_ratio_test)
{
  matcher_use_desc_ratio_test_ = matcher_use_desc_ratio_test; 
}

void KeyframeGraphDetector::setMatcherMaxDescRatio(double matcher_max_desc_ratio)
{
  matcher_max_desc_ratio_ = matcher_max_desc_ratio;
}

void KeyframeGraphDetector::setMatcherMaxDescDist(double matcher_max_desc_dist)
{
  matcher_max_desc_dist_ = matcher_max_desc_dist;
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

void KeyframeGraphDetector::setOutputPath(const std::string& output_path)
{
  output_path_ = output_path;
  boost::filesystem::create_directories(output_path_); 
}

void KeyframeGraphDetector::setSACSaveResults(bool sac_save_results)
{
  sac_save_results_ = sac_save_results;
}

void KeyframeGraphDetector::setVerbose(bool verbose)
{
  verbose_ = verbose;
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
  prepareFeaturesForRANSAC(keyframes);
  
  buildAssociationMatrix(keyframes, associations);  
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
  bool upright = true;
  double orb_threshold = 31;

  if(verbose_) printf("preparing SURF features for matching...\n");  

  cv::OrbDescriptorExtractor extractor;
 
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
  { 
    RGBDKeyframe& keyframe = keyframes[kf_idx];
  
    cv::OrbFeatureDetector detector(400, 1.2f, 8, orb_threshold, 0, 2, 0, 31);
    keyframe.keypoints.clear();
    detector.detect(keyframe.rgb_img, keyframe.keypoints);

    if(verbose_)
    {
      printf("[KF %d of %d] %d ORB keypoints detected\n", 
        (int)kf_idx, (int)keyframes.size(), (int)keyframe.keypoints.size()); 
    }

    if (sac_save_results_)
    {
      cv::Mat kp_img;
      cv::drawKeypoints(keyframe.rgb_img, keyframe.keypoints, kp_img);
      std::stringstream ss1;
      ss1 << "kp_" << kf_idx;
      cv::imwrite(output_path_ + "/" + ss1.str() + ".png", kp_img);
    }

    extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
    keyframe.computeDistributions();
  }
}


void KeyframeGraphDetector::buildAssociationMatrix(
  const KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  prepareMatchers(keyframes);
  
  // 1. Create the candidate matrix
  buildCandidateMatrix(keyframes);

  // 2. Perfrom pairwise matching for all candidates
  buildCorrespondenceMatrix(keyframes, associations); 
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

  match_matrix_ = cv::Mat::zeros(kf_size, kf_size, CV_16UC1);
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
        match_matrix_.at<uint16_t>(index_a, index_b) = corresp_count;
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
  int size = match_matrix_.rows;
  assert(size > 0);
  
  // check for validity of n_candidates argument
  if(n_candidates_ < size) n_candidates_ = size;

  // initialize candidate matrix as all 0
  candidate_matrix_ = cv::Mat::eye(match_matrix_.size(), CV_8UC1);
  
  for (int v = 0; v < match_matrix_.rows; ++v)
  {
    // create a vector from the current row
    std::vector<std::pair<int, int> > values(match_matrix_.cols);
    for (int u = 0; u < match_matrix_.cols; ++u)
    {
      int value = match_matrix_.at<uint16_t>(v,u);
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
  const KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  // check for square matrix
  assert(candidate_matrix_.rows == candidate_matrix_.cols);
  int size = candidate_matrix_.rows;    
  
  // initialize correspondence matrix    
  correspondence_matrix_ = cv::Mat::zeros(size, size, CV_16UC1);
  association_matrix_    = cv::Mat::zeros(size, size, CV_8UC1);

  for (int kf_idx_q = 0; kf_idx_q < size; ++kf_idx_q)
  for (int kf_idx_t = 0; kf_idx_t < size; ++kf_idx_t)
  {
    const RGBDKeyframe& keyframe_q = keyframes[kf_idx_q];
    const RGBDKeyframe& keyframe_t = keyframes[kf_idx_t];
  
    if (kf_idx_q == kf_idx_t)
    {
      // self-association
      
      //correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.keypoints.size();
      correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_q) = keyframe_q.n_valid_keypoints;
          
      association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_q) = 1;
    }
    else
    {
      // skip non-candidates
      if (candidate_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) != 0)
      {
        if(verbose_) printf("[RANSAC %d to %d]: ", kf_idx_q, kf_idx_t);

        std::vector<cv::DMatch> inlier_matches;

        // perform ransac matching, b onto a
        Eigen::Matrix4f transformation;

        // query, train
        int iterations = pairwiseMatching(
          kf_idx_q, kf_idx_t, keyframes, inlier_matches, transformation);
        
        correspondence_matrix_.at<uint16_t>(kf_idx_q, kf_idx_t) = inlier_matches.size();
        
        if (inlier_matches.size() >= sac_min_inliers_)
        {
          // mark the association matrix
          association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 1;
          
          // add an association
          KeyframeAssociation association;
          association.type = KeyframeAssociation::RANSAC;
          association.kf_idx_a = kf_idx_t;
          association.kf_idx_b = kf_idx_q;
          association.matches  = inlier_matches;
          association.a2b = transformation;
          associations.push_back(association);
          
          // output the results to screen         
          if(verbose_) 
            printf("pass [%d][%d]\n", iterations, (int)inlier_matches.size());         
          
          // save the results to file
          if (sac_save_results_)
          {
            cv::Mat img_matches;
            cv::drawMatches(keyframe_q.rgb_img, keyframe_q.keypoints, 
                            keyframe_t.rgb_img, keyframe_t.keypoints, 
                            inlier_matches, img_matches);

            std::stringstream ss1;
            ss1 << kf_idx_q << "_to_" << kf_idx_t;
            cv::imwrite(output_path_ + "/" + ss1.str() + ".png", img_matches);
          }
        }
        else
        {
          association_matrix_.at<uint8_t>(kf_idx_q, kf_idx_t) = 0;
          
          if(verbose_)
            printf("fail [%d][%d]\n", iterations, (int)inlier_matches.size());     
        }
      }
    }
  }
}  

// frame_a = train, frame_b = query
void KeyframeGraphDetector::getCandidateMatches(
  const RGBDFrame& frame_q, const RGBDFrame& frame_t, 
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
      frame_q.descriptors, all_matches2, 2);

    for (unsigned int m_idx = 0; m_idx < all_matches2.size(); ++m_idx)
    {
      const cv::DMatch& match1 = all_matches2[m_idx][0];
      const cv::DMatch& match2 = all_matches2[m_idx][1];
      
      double ratio =  match1.distance / match2.distance;
      
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
  
// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatching(
  int kf_idx_q, int kf_idx_t,
  const KeyframeVector& keyframes,
  DMatchVector& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  int iterations;
  
  if (pairwise_matching_method_ == PAIRWISE_MATCHING_RANSAC)
  {
    iterations = pairwiseMatchingRANSAC(
      kf_idx_q, kf_idx_t, keyframes, best_inlier_matches, best_transformation);
  }
  else if (pairwise_matching_method_ == PAIRWISE_MATCHING_BFSAC)
  {
    iterations = pairwiseMatchingBFSAC(
      kf_idx_q, kf_idx_t, keyframes, best_inlier_matches, best_transformation);
  }
  
  return iterations;
}
  
// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatchingBFSAC(
  int kf_idx_q, int kf_idx_t,
  const KeyframeVector& keyframes,
  DMatchVector& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // constants
  int min_sample_size = 3;

  const RGBDFrame& frame_t = keyframes[kf_idx_t];
  const RGBDFrame& frame_q = keyframes[kf_idx_q];
  cv::FlannBasedMatcher& matcher = matchers_[kf_idx_t];
  
  // find candidate matches
  DMatchVector candidate_matches;
  getCandidateMatches(frame_q, frame_t, matcher, candidate_matches);
  if (candidate_matches.size() < min_sample_size) return 0;
  
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
      features_q, inlier_idx,
      features_t, inlier_idx,
      transformation);
    
    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_q_tf;
    pcl::transformPointCloud(features_q, features_q_tf, transformation);

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
    
    iterations++;
  }
  
  return iterations;
}  
  
// frame_a = train, frame_b = query
int KeyframeGraphDetector::pairwiseMatchingRANSAC(
  int kf_idx_q, int kf_idx_t,
  const KeyframeVector& keyframes,
  DMatchVector& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // constants
  int min_sample_size = 3; 

  // find candidate matches
  const RGBDFrame& frame_t = keyframes[kf_idx_t];
  const RGBDFrame& frame_q = keyframes[kf_idx_q];
  cv::FlannBasedMatcher& matcher = matchers_[kf_idx_t];
  
  DMatchVector candidate_matches;
  getCandidateMatches(frame_q, frame_t, matcher, candidate_matches);
  
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
      features_q, inlier_idx,
      features_t, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_q_tf;
    pcl::transformPointCloud(features_q, features_q_tf, transformation);

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

} // namespace rgbdtools
