/**
 *  @file keyframe_graph_detector.h
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

#ifndef RGBDTOOLS_KEYFRAME_GRAPH_DETECTOR_H
#define RGBDTOOLS_KEYFRAME_GRAPH_DETECTOR_H

#include <boost/thread/mutex.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <opencv2/nonfree/features2d.hpp>

#include "rgbdtools/types.h"
#include "rgbdtools/rgbd_keyframe.h"
#include "rgbdtools/map_util.h"
#include "rgbdtools/graph/keyframe_association.h"

namespace rgbdtools {

/** @brief Detects graph correspondences based on visual feature
 * matching between keyframes.
 */  
class KeyframeGraphDetector
{
  public:

    enum CandidateGenerationMethod
    {
      CANDIDATE_GENERATION_BRUTE_FORCE,
      CANDIDATE_GENERATION_TREE
    };

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
    KeyframeGraphDetector();
    
    /** @brief Default destructor
     */
    virtual ~KeyframeGraphDetector();

    void setNCandidates(int n_candidates);
    void setKNearestNeighbors(int k_nearest_neighbors);
    void setNKeypoints(int n_keypoints);
    void setCandidateGenerationMethod(CandidateGenerationMethod candidate_method);
    
    /** Main method for generating associatuions
     * @param keyframes the input vector of RGBD keyframes
     * @param associations reference to the output vector of associations
     */
    void generateKeyframeAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    // --------------------------------------------

    void buildSURFAssociationMatrix(const KeyframeVector& keyframes);

   private:

    /** @brief Maximim iterations for the RANSAC test
     */
    int ransac_max_iterations_;
    
    /** @brief How many inliers are required to pass the RANSAC test.
     * 
     * If a candidate keyframe has fewer correspondences or more, 
     * it will not be eligible for a RANSAC test 
     */
    int ransac_min_inliers_;
    
    bool ransac_use_desc_ratio_test_;
    
    double ransac_max_desc_ratio_;
    
    /** @brief Maximum distance (in descriptor space) between
     * two features to be considered a correspondence candidate
     */
    double ransac_max_desc_dist_;
    
    /** @brief Maximum distance squared (in Euclidean space) between
     * two features to be considered a correspondence candidate
     */
    double ransac_max_eucl_dist_sq_;
    
    double ransac_sufficient_inlier_ratio_;
    
    /** @brief If true, positive RANSAC results will be saved
     * to file as images with keypoint correspondences
     */
    bool ransac_save_results_;
    
    /** @brief The path where to save images if save_ransac_results_ is true
     */
    std::string ransac_results_path_;
    
    /** @brief For kd-tree based correspondences, how many candidate
     * keyframes will be tested agains the query keyframe using a RANSAC test
     */
    double n_candidates_;
    
    /** @brief How many nearest neighbors are requested per keypoint
     */
    int k_nearest_neighbors_;    
        
    /** @brief Number of desired keypoints to detect in each image
     */
    int n_keypoints_;
    
    /** @brief TREE of BRUTE_FORCE */
    CandidateGenerationMethod candidate_method_;
              

    
    //------------------------------------------
    
    /** @brief CV_8UC1, 1 if associated, 0 otherwise */
    cv::Mat association_matrix_;
    
    /** @brief CV_8UC1, 1 if candidate, 0 otherwise */
    cv::Mat candidate_matrix_;
    
    /** @brief CV_32FC1, for tree-based matching, contains number of inlier matches */
    cv::Mat correspondence_matrix_;
    
    /** @brief CV_32FC1, for tree-based matching, contains number of total matches */
    cv::Mat match_matrix_;  
    
    // ------------
    
    void prepareFeaturesForRANSAC(KeyframeVector& keyframes);
    
    void buildSURFMatchMatrixTree(const KeyframeVector& keyframes);
      
    void buildSURFCandidateMatrixTree();
    
    void buildSURFCandidateMatrix(const KeyframeVector& keyframes);
        
    void buildRANSACCorrespondenceMatrix(const KeyframeVector& keyframes);
        
    void pairwiseMatchingRANSAC(
      const RGBDFrame& frame_a, const RGBDFrame& frame_b,
      std::vector<cv::DMatch>& all_matches,
      std::vector<cv::DMatch>& best_inlier_matches,
      Eigen::Matrix4f& best_transformation);
};

} // namespace rgbdtools

#endif // RGBDTOOLS_KEYFRAME_GRAPH_DETECTOR_H
