#include "keyframe_associations.h"

using namespace rgbdtools;

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    printUsage(argv);
    return -1;
  }
  
  std::string keyframe_path  = argv[1];
  std::string output_path = argv[2];
  
  // load keyframes
  rgbdtools::KeyframeVector keyframes;
  loadKeyframes(keyframes, keyframe_path);

  // **** brute force
    
  // build and save brute-force matrix
  //buildAndSaveBFMatrix(keyframes, keyframe_path);

  // load bf matrix
  cv::Mat bf_assoc = cv::imread(output_path + "/bf_assoc.png", -1);  
  cv::namedWindow("BF Associations", 0);
  cv::Mat bf_assoc_vis; 
  bf_assoc.convertTo(bf_assoc_vis, CV_8UC1);
  bf_assoc_vis *= 255;
  cv::imshow("BF Associations", bf_assoc_vis);
  cv::waitKey(1000);
  
  // **** tree based
  
  rgbdtools::KeyframeGraphDetector graph_detector;
  
  graph_detector.setCandidateGenerationMethod(
    rgbdtools::KeyframeGraphDetector::CANDIDATE_GENERATION_SURF_TREE);

  graph_detector.setPairwiseMatchingMethod(
    rgbdtools::KeyframeGraphDetector::PAIRWISE_MATCHING_RANSAC);
  
  graph_detector.setPairwiseMatcherIndex(
    rgbdtools::KeyframeGraphDetector::PAIRWISE_MATCHER_KDTREE);
  
  graph_detector.setSACResultsPath(output_path + "/sac_tree_images/");
  graph_detector.setSACSaveResults(false);
  
  graph_detector.prepareFeaturesForRANSAC(keyframes);

  // **** comparisons
  
  printf("--------------------------------------------------------------\n");
  printf("K\tN\tTOT\tFN(C)\tFN(A)\tR(C)\tR(A)\tDUR[s]\n");
  printf("--------------------------------------------------------------\n");
   
  for (int n = 5; n <= 20; n+=5)
  for (int k = 2; k <= 10; k+=1)
  {
    graph_detector.setNCandidates(n);
    graph_detector.setKNearestNeighbors(k);
  
    const clock_t start = clock();
    graph_detector.buildAssociationMatrix(keyframes);
    float dur_s = (clock() - start)/(float)CLOCKS_PER_SEC;
    
    cv::Mat tree_assoc = graph_detector.getAssociationMatrix(); 
    cv::Mat tree_cand  = graph_detector.getCandidateMatrix();
    
    cv::Mat tree_assoc_vis = 255 * tree_assoc;
    cv::Mat tree_cand_vis  = 255 * tree_cand;
    
    cv::namedWindow("Tree Associations", 0);
    cv::namedWindow("Tree Candidates", 0);
    
    cv::imshow("Tree Associations", tree_assoc_vis);
    cv::imshow("Tree Candidates", tree_cand_vis);
    
    cv::waitKey(1);
    
    int fp, fn, tot;
    compareAssociationMatrix(bf_assoc, tree_assoc, fp, fn, tot);
    double fnr = (double)fn / double(tot);
    
    int fp_c, fn_c, tot_c;
    compareAssociationMatrix(bf_assoc, tree_cand, fp_c, fn_c, tot_c);
    double fnr_c = (double)fn_c / double(tot_c);

    printf("%d\t%d\t%d\t%d\t%d\t%.3f\t%.3f\t%.1f\n", 
      k, n, tot, fn_c, fn, 1-fnr_c, 1-fnr, dur_s);
  }
  
  cv::waitKey(0);
  
  return 0;
}

void printUsage(char** argv)
{
  std::cerr << "error: usage is " << argv[0] 
            << " [keyframe_path] [output_path]"
            << std::endl;
}

void buildAndSaveBFMatrix(
  rgbdtools::KeyframeVector keyframes,
  const std::string output_path)
{
  rgbdtools::KeyframeGraphDetector graph_detector;
  
  graph_detector.setVerbose(true);
  
  graph_detector.setCandidateGenerationMethod(
    rgbdtools::KeyframeGraphDetector::CANDIDATE_GENERATION_BRUTE_FORCE);

  graph_detector.setPairwiseMatchingMethod(
    rgbdtools::KeyframeGraphDetector::PAIRWISE_MATCHING_BFSAC);
  
  graph_detector.setPairwiseMatcherIndex(
    rgbdtools::KeyframeGraphDetector::PAIRWISE_MATCHER_LINEAR);
  
  graph_detector.setSACResultsPath(output_path + "/sac_bf_images/");
  graph_detector.setSACSaveResults(true);
  
  graph_detector.prepareFeaturesForRANSAC(keyframes);
  
  const clock_t start = clock();
  graph_detector.buildAssociationMatrix(keyframes);
  float dur_s = (clock() - start)/(float)CLOCKS_PER_SEC;
      
  printf("--------------------------------------------------------------\n");
  printf("BF:%.1f\n", dur_s);
  printf("--------------------------------------------------------------\n");
  
  // save corerspondence matrix to file
  std::string corr_matrix_filename = output_path + "/bf_corr.png";
  cv::imwrite(corr_matrix_filename, graph_detector.getCorrespondenceMatrix());

  // save association matrix to file
  std::string assoc_matrix_filename = output_path + "/bf_assoc.png";
  cv::imwrite(assoc_matrix_filename, graph_detector.getAssociationMatrix());
}
