#include "global_cloud_align.h"

using namespace rgbdtools;

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    printUsage(argv);
    return -1;
  }
  
  std::string filename_in  = argv[1];
  std::string filename_out = argv[2];
  
  // read in
  printf("Reading cloud\n");
  PointCloudT::Ptr cloud;
  cloud.reset(new rgbdtools::PointCloudT());
  pcl::PCDReader reader;
  reader.read(filename_in, *cloud);
  
  alignGlobalCloud(cloud);
  
  return 0;
}

void printUsage(char** argv)
{
  std::cerr << "error: usage is " << argv[0] 
            << " [filename_in] [filename_out]"
            << std::endl;
}

void alignGlobalCloud(const PointCloudT::Ptr& cloud)
{
  double vgf_res = 0.01;

  // filter cloud
  printf("Filtering cloud\n");
  PointCloudT::Ptr cloud_f;
  cloud_f.reset(new PointCloudT());
  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud(cloud);
  vgf.setLeafSize(vgf_res, vgf_res, vgf_res);
  vgf.filter(*cloud_f);

  /*
  // rotate 45 deg
  tf::Transform t;
  t.setOrigin(tf::Vector3(0,0,0));
  tf::Quaternion q;
  q.setRPY(0, 0, M_PI/6.0);
  t.setRotation(q);
  pcl::transformPointCloud(*cloud_f, *cloud_f, eigenFromTf(t)); 
  */

  // show map
  printf("Showing map\n");
  cv::Mat map_r;
  create2DProjectionImage(*cloud_f, map_r);
  cv::imshow("map_r", map_r);
  cv::waitKey(0);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<PointT, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud(cloud_f);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  printf("Creating kd-tree\n");
  pcl::search::KdTree<PointT>::Ptr tree;
  tree.reset(new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals;
  cloud_normals.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  // Use all neighbors in a sphere of radius x cm
  ne.setRadiusSearch(0.05);
  
  // Compute the features
  printf("Estimating normals\n");
  ne.compute (*cloud_normals);

  /*
  for (unsigned int i = 0; i < cloud_f->points.size(); ++i)
  {
    const PointT& p_in = cloud_f->points[i]; 
    pcl::PointXYZRGBNormal& p_out = cloud_normals->points[i]; 

    p_out.x   = p_in.x;
    p_out.y   = p_in.y;
    p_out.z   = p_in.z;
    p_out.rgb = p_in.rgb;
  }

  // write out  
  printf("Writing out\n");
  pcl::PCDWriter writer;
  writer.write ("/home/idryanov/cloud_00_n.pcd", *cloud_normals);
*/
  
  // create expected histogram
  double hist_resolution = 0.25;
  cv::Mat hist_exp;
  buildExpectedPhiHistorgtam(hist_exp, hist_resolution, 5.0); 
  cv::Mat hist_exp_img;
  createImageFromHistogram(hist_exp, hist_exp_img);
  cv::imshow("hist_exp_img", hist_exp_img);

  // create histogram
  printf("creating histogram\n");
  cv::Mat hist;
  buildPhiHistogram(*cloud_normals, hist, hist_resolution);

  // show histogram
  printf("showing histogram\n");
  cv::Mat hist_img;
  createImageFromHistogram(hist, hist_img);
  cv::imshow("Histogram Phi", hist_img);
  cv::waitKey(0);

  // find alignement
  double best_angle;
  alignHistogram(hist, hist_exp, hist_resolution, best_angle);
  printf("best_angle: %f\n", best_angle);

  // show best aligned histogram
  cv::Mat hist_best;
  shiftHistogram(hist, hist_best, best_angle/hist_resolution);
  cv::Mat hist_best_img;
  createImageFromHistogram(hist_best, hist_best_img);
  cv::imshow("hist_best_img", hist_best_img);
  cv::waitKey(0);

  // derotate
  AffineTransform best_tf;
  XYZRPYToEigenAffine(0, 0, 0, 0, 0, best_angle * M_PI/180.0, best_tf);

  /*
  // derotate
  tf::Transform t1;
  t1.setOrigin(tf::Vector3(0,0,0));
  tf::Quaternion q1;
  q1.setRPY(0, 0, best_angle * M_PI/180.0);
  t1.setRotation(q1);
  */
  
  pcl::transformPointCloud(*cloud_f, *cloud_f, best_tf); 

  // show map
  cv::Mat map_f;
  create2DProjectionImage(*cloud_f, map_f);
  cv::imshow("map_f", map_f);
  cv::waitKey(0);

  printf("Done\n");
}