#include "keyframe_associations.h"

using namespace rgbdtools;

int main(int argc, char** argv)
{
  // **** handle input  **********************************************
  
  if (argc != 3)
  {
    printUsage(argv);
    return -1;
  }
  
  std::string keyframe_path = argv[1];
  std::string output_path   = argv[2];
  
  std::string bf_output_path   = output_path + "/bf/";
  std::string tree_output_path = output_path + "/tree/";
  
  // **** load keyframes and prepare features ***********************
  
  rgbdtools::KeyframeVector keyframes;
  loadKeyframes(keyframes, keyframe_path);
  
  rgbdtools::KeyframeGraphDetector graph_detector;
  graph_detector.setVerbose(true);
  graph_detector.prepareFeaturesForRANSAC(keyframes);
    
  // **** brute force ***********************************************
       
  bruteForceAssociations(graph_detector, keyframes, bf_output_path);
  
  // load BF matrix and show
  cv::Mat bf_assoc = cv::imread(bf_output_path + "/bf_assoc.png", -1);  
  cv::namedWindow("BF Associations", 0);
  cv::imshow("BF Associations", bf_assoc*255);
  cv::waitKey(1);
  
  // **** tree based ************************************************
   
  printf("--------------------------------------------------------------\n");
  printf("K\tN\tTOT\tFN(C)\tFN(A)\tR(C)\tR(A)\tDUR[s]\n");
  printf("--------------------------------------------------------------\n");
  
  cv::namedWindow("Tree Associations", 0);
  cv::namedWindow("Tree Candidates", 0);
    
  for (int n = 5; n <= 20; n+=5)
  for (int k = 2; k <= 10; k+=1)
  {
    treeAssociations(graph_detector, keyframes, tree_output_path, bf_assoc, k, n);
  }
  
  printf("--------------------------------------------------------------\n");
  
  return 0;
}

void bruteForceAssociations(
  rgbdtools::KeyframeGraphDetector& graph_detector,
  rgbdtools::KeyframeVector& keyframes,
  const std::string& bf_output_path)
{
  graph_detector.setCandidateGenerationMethod(
    rgbdtools::KeyframeGraphDetector::CANDIDATE_GENERATION_BRUTE_FORCE);
  graph_detector.setPairwiseMatchingMethod(
    rgbdtools::KeyframeGraphDetector::PAIRWISE_MATCHING_BFSAC);
  graph_detector.setPairwiseMatcherIndex(
    rgbdtools::KeyframeGraphDetector::PAIRWISE_MATCHER_LINEAR);

  graph_detector.setVerbose(true);
  graph_detector.setOutputPath(bf_output_path + "/sac_images/");
  graph_detector.setSACSaveResults(true);
  
  rgbdtools::KeyframeAssociationVector associations;
  
  const clock_t start = clock();
  graph_detector.buildAssociationMatrix(keyframes, associations);
  float dur_s = (clock() - start)/(float)CLOCKS_PER_SEC;
      
  printf("--------------------------------------------------------------\n");
  printf("BF:%.1f\n", dur_s);
  printf("--------------------------------------------------------------\n");
  
  // save BF matrices to file
  cv::imwrite(bf_output_path + "/bf_corr.png",  graph_detector.getCorrespondenceMatrix());
  cv::imwrite(bf_output_path + "/bf_assoc.png", graph_detector.getAssociationMatrix());
}

void treeAssociations(
  rgbdtools::KeyframeGraphDetector& graph_detector,
  rgbdtools::KeyframeVector& keyframes,
  const std::string& tree_output_path,
  const cv::Mat& bf_assoc,
  int k, int n)
{
  // set parameters relating to tree matchin
  graph_detector.setCandidateGenerationMethod(
    rgbdtools::KeyframeGraphDetector::CANDIDATE_GENERATION_SURF_TREE);
  graph_detector.setPairwiseMatchingMethod(
    rgbdtools::KeyframeGraphDetector::PAIRWISE_MATCHING_RANSAC); 
  graph_detector.setPairwiseMatcherIndex(
    rgbdtools::KeyframeGraphDetector::PAIRWISE_MATCHER_KDTREE);
  graph_detector.setNCandidates(n);
  graph_detector.setKNearestNeighbors(k);
  
  // set output parameters
  graph_detector.setVerbose(false);
  graph_detector.setSACSaveResults(false);
  
  std::stringstream ss_tree_output_path_kn;
  ss_tree_output_path_kn << tree_output_path << n << "_" << k << "/";
  std::string tree_output_path_kn = ss_tree_output_path_kn.str();
    
  graph_detector.setOutputPath(tree_output_path_kn + "/sac_images/");
  
  // build the associations
  rgbdtools::KeyframeAssociationVector associations;
  const clock_t start = clock();
  graph_detector.buildAssociationMatrix(keyframes, associations);
  float dur_s = (clock() - start)/(float)CLOCKS_PER_SEC;
  
  // show & save matrices
  cv::Mat tree_cand  = graph_detector.getCandidateMatrix();
  cv::Mat tree_corr  = graph_detector.getCorrespondenceMatrix();
  cv::Mat tree_assoc = graph_detector.getAssociationMatrix();   
  
  // save BF matrices to file
  cv::imwrite(tree_output_path_kn + "/tree_cand.png",  tree_cand);
  cv::imwrite(tree_output_path_kn + "/tree_corr.png",  tree_corr);
  cv::imwrite(tree_output_path_kn + "/tree_assoc.png", tree_assoc);
  
  cv::imshow("Tree Associations", tree_assoc*255);
  cv::imshow("Tree Candidates",   tree_cand*255);
  cv::waitKey(1);
  
  // print out results
  int fp, fn, tot;
  compareAssociationMatrix(bf_assoc, tree_assoc, fp, fn, tot);
  double fnr = (double)fn / double(tot);
  
  int fp_c, fn_c, tot_c;
  compareAssociationMatrix(bf_assoc, tree_cand, fp_c, fn_c, tot_c);
  double fnr_c = (double)fn_c / double(tot_c);

  printf("%d\t%d\t%d\t%d\t%d\t%.3f\t%.3f\t%.1f\n", 
    k, n, tot, fn_c, fn, 1-fnr_c, 1-fnr, dur_s);
}
 
void printUsage(char** argv)
{
  std::cerr << "error: usage is " << argv[0] 
            << " [keyframe_path] [output_path]"
            << std::endl;
}
