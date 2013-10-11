/**
 *  @file rgbd_frame.cpp
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

#include "rgbdtools/rgbd_frame.h"

namespace rgbdtools {

RGBDFrame::RGBDFrame()
{

}

RGBDFrame::RGBDFrame(
  const cv::Mat& rgb_img_in,
  const cv::Mat& depth_img_in,
  const cv::Mat& intr_in,
  const Header& header_in)
{
  // zero-copy assignment
  rgb_img = rgb_img_in;
  depth_img = depth_img_in;

  header = header_in; 
  intr = intr_in;
  
  // @FIXME: handle encodings
  /*
     const std::string& enc = depth_msg->encoding; 
  if (enc.compare("16UC1") == 0)
    depth_img = cv_bridge::toCvShare(depth_msg)->image;
  else if (enc.compare("32FC1") == 0)
    depthImageFloatTo16bit(cv_bridge::toCvShare(depth_msg)->image, depth_img);
  */
  
  // @FIXME double vs float - change all to float
}

/*
RGBDFrame::RGBDFrame(
  const ImageMsg::ConstPtr& rgb_msg,
  const ImageMsg::ConstPtr& depth_msg,
  const CameraInfoMsg::ConstPtr& info_msg)
{ 
  rgb_img = cv_bridge::toCvShare(rgb_msg)->image;
  
  // handles 16UC1 natively
  // 32FC1 need to be converted into 16UC1
  const std::string& enc = depth_msg->encoding; 
  if (enc.compare("16UC1") == 0)
    depth_img = cv_bridge::toCvShare(depth_msg)->image;
  else if (enc.compare("32FC1") == 0)
    depthImageFloatTo16bit(cv_bridge::toCvShare(depth_msg)->image, depth_img);
      
  header = rgb_msg->header;

  model.fromCameraInfo(info_msg);
}*/

double RGBDFrame::getStdDevZ(double z) const
{
  return Z_STDEV_CONSTANT * z * z;
}

double RGBDFrame::getVarZ(double z) const
{
  double std_dev_z = getStdDevZ(z);
  return std_dev_z * std_dev_z;
}

void RGBDFrame::getGaussianDistribution(
  int u, int v, double& z_mean, double& z_var) const
{
  // get raw z value (in mm)
  uint16_t z_raw = depth_img.at<uint16_t>(v, u);

  // z [meters]
  z_mean = z_raw * 0.001;

  // var_z [meters]
  z_var = getVarZ(z_mean);
}

void RGBDFrame::getGaussianMixtureDistribution(
  int u, int v, double& z_mean, double& z_var) const
{
  /// @todo Different window sizes? based on sigma_u, sigma_v?
  int w = 1;

  int u_start = std::max(u - w, 0);
  int v_start = std::max(v - w, 0);
  int u_end   = std::min(u + w, depth_img.cols - 1);
  int v_end   = std::min(v + w, depth_img.rows - 1);

  // iterate accross window - find mean
  double weight_sum = 0.0;
  double mean_sum   = 0.0;
  double alpha_sum  = 0.0;

  for (int uu = u_start; uu <= u_end; ++uu)
  for (int vv = v_start; vv <= v_end; ++vv)
  {
    uint16_t z_neighbor_raw = depth_img.at<uint16_t>(vv, uu);
 
    if (z_neighbor_raw != 0)
    {
      double z_neighbor = z_neighbor_raw * 0.001;

      // determine and aggregate weight
      double weight;
      if       (u==uu && v==vv) weight = 4.0;
      else if  (u==uu || v==vv) weight = 2.0;
      else                      weight = 1.0; 
      weight_sum += weight;

      // aggregate mean
      mean_sum += weight * z_neighbor;

      // aggregate var
      double var_z_neighbor = getVarZ(z_neighbor);
      alpha_sum += weight * (var_z_neighbor + z_neighbor * z_neighbor);
    }
  }

  z_mean = mean_sum  / weight_sum;
  z_var  = alpha_sum / weight_sum - z_mean * z_mean;
}

void RGBDFrame::computeDistributions(
  double max_z,
  double max_stdev_z)
{
  double max_var_z = max_stdev_z * max_stdev_z; // maximum allowed z variance

  /// @todo These should be arguments or const static members
  double s_u = 1.0;            // uncertainty in pixels
  double s_v = 1.0;            // uncertainty in pixels

  n_valid_keypoints = 0;

  // center point
  double cx = intr.at<double>(0, 2);
  double cy = intr.at<double>(1, 2);

  // focus length
  double fx = intr.at<double>(0, 0);
  double fy = intr.at<double>(1, 1);

  // precompute for convenience
  double var_u = s_u * s_u;
  double var_v = s_v * s_v;
  double fx2 = fx*fx;
  double fy2 = fy*fy;

  // allocate space
  kp_valid.clear();
  kp_means.clear();
  kp_covariances.clear();

  kp_valid.resize(keypoints.size());
  kp_means.resize(keypoints.size());
  kp_covariances.resize(keypoints.size());

  for (unsigned int kp_idx = 0; kp_idx < keypoints.size(); ++kp_idx)
  {
    // calculate pixel coordinates
    double u = keypoints[kp_idx].pt.x;
    double v = keypoints[kp_idx].pt.y;  

    // get raw z value
    uint16_t z_raw = depth_img.at<uint16_t>((int)v, (int)u);

    // skip bad values  
    if (z_raw == 0)
    {
      kp_valid[kp_idx] = false;
      continue;
    }
  
    // get z: mean and variance
    double z, var_z;
    //getGaussianDistribution(u, v, z, var_z);
    getGaussianMixtureDistribution(u, v, z, var_z);

    // skip bad values - too far away, or z-variance too big
    if (z > max_z || var_z > max_var_z)
    {
      kp_valid[kp_idx] = false;
      continue;
    }
    kp_valid[kp_idx] = true;
    n_valid_keypoints++;

    // precompute for convenience
    double z_2  = z * z;
    double umcx = u - cx;
    double vmcy = v - cy;

    // calculate x and y
    double x = z * umcx / fx;
    double y = z * vmcy / fy;
  
    // calculate covariances
    double s_xz = var_z * umcx / fx;
    double s_yz = var_z * vmcy / fy;

    double s_xx = (var_z * umcx * umcx + var_u * (z_2 + var_z))/fx2;
    double s_yy = (var_z * vmcy * vmcy + var_v * (z_2 + var_z))/fy2;

    double s_xy = umcx * vmcy * var_z / (fx * fy);
    double s_yx = s_xy;

    double s_zz = var_z; 

    double s_zy = s_yz;
    double s_zx = s_xz;
   
    // fill out mean matrix
    Vector3f& kp_mean = kp_means[kp_idx];

    kp_mean(0,0) = x;
    kp_mean(1,0) = y;
    kp_mean(2,0) = z;

    // fill out covariance matrix
    Matrix3f& kp_covariance = kp_covariances[kp_idx];

    kp_covariance(0,0) = s_xx; // xx
    kp_covariance(0,1) = s_xy; // xy
    kp_covariance(0,2) = s_xz; // xz

    kp_covariance(1,0) = s_yx; // xy
    kp_covariance(1,1) = s_yy; // yy
    kp_covariance(1,2) = s_yz; // yz

    kp_covariance(2,0) = s_zx; // xz-
    kp_covariance(2,1) = s_zy; // yz
    kp_covariance(2,2) = s_zz; // zz
  }
}

void RGBDFrame::constructFeaturePointCloud(
  PointCloudFeature& cloud)
{
  // filter invalid
  Vector3fVector means_f;
  removeInvalidMeans(kp_means, kp_valid, means_f);

  // create point cloud
  pointCloudFromMeans(means_f, cloud);
  
  //set the header
  cloud.header.frame_id   = header.frame_id;
  cloud.header.seq        = header.seq;

  // The point cloud stamp, in usec.
  cloud.header.stamp = header.stamp.sec * 1e6 + header.stamp.nsec * 1e-3;
}

void RGBDFrame::constructDensePointCloud(
  PointCloudT& cloud,
  double max_z,
  double max_stdev_z) const
{
  double max_var_z = max_stdev_z * max_stdev_z; // maximum allowed z variance

  // center point
  double cx = intr.at<double>(0, 2);
  double cy = intr.at<double>(1, 2);

  // focus length
  double fx = intr.at<double>(0, 0);
  double fy = intr.at<double>(1, 1);

  // Scale by focal length for computing (X,Y)
  float constant_x = 1.0 / fx;
  float constant_y = 1.0 / fy;

  float bad_point = std::numeric_limits<float>::quiet_NaN();

  cloud.points.clear();
  cloud.points.resize(rgb_img.rows * rgb_img.cols);
  for (int v = 0; v < rgb_img.rows; ++v)
  for (int u = 0; u < rgb_img.cols; ++u)
  {
    unsigned int index = v * rgb_img.cols + u;

    uint16_t z_raw = depth_img.at<uint16_t>(v, u);

    float z = z_raw * 0.001; //convert to meters

    PointT& p = cloud.points[index];

    double z_mean, z_var; 

    // check for out of range or bad measurements
    if (z_raw != 0)
    {
      getGaussianMixtureDistribution(u, v, z_mean, z_var);

      // check for variance and z limits     
      if (z_var < max_var_z && z_mean < max_z)
      {
        // fill in XYZ
        p.x = z * (u - cx) * constant_x;
        p.y = z * (v - cy) * constant_y;
        p.z = z;
      }
      else
      {
        p.x = p.y = p.z = bad_point;
      }
    }
    else
    {
      p.x = p.y = p.z = bad_point;
    }
 
    // fill out color
    const cv::Vec3b& color = rgb_img.at<cv::Vec3b>(v,u);
    p.r = color[2];
    p.g = color[1];
    p.b = color[0];
  }

  // set cloud header
  cloud.header.frame_id = header.frame_id;
  // The point cloud stamp, in usec.
  cloud.header.stamp = header.stamp.sec * 1e6 + header.stamp.nsec * 1e-3;
  cloud.header.seq = header.seq;
    
  cloud.height = rgb_img.rows;
  cloud.width  = rgb_img.cols;
  cloud.is_dense = false;
}

bool RGBDFrame::save(
  const RGBDFrame& frame, 
  const std::string& path)
{
  // set the filenames
  std::string rgb_filename    = path + "/rgb.png";
  std::string depth_filename  = path + "/depth.png";
  std::string header_filename = path + "/header.yml";
  std::string intr_filename   = path + "/intr.yml"; 
  std::string cloud_filename  = path + "/cloud.pcd";
  
  // create the directory
  bool directory_result = boost::filesystem::create_directory(path); 

  if (!directory_result)    
  {
    std::cerr << "Could not create directory:" <<  path << std::endl;
    return false;
  }

  // save header
  cv::FileStorage fs_h(header_filename, cv::FileStorage::WRITE);
  fs_h << "frame_id"   << frame.header.frame_id;
  fs_h << "seq"        << (int)frame.header.seq;
  fs_h << "stamp_sec"  << (int)frame.header.stamp.sec;
  fs_h << "stamp_nsec" << (int)frame.header.stamp.nsec;
  fs_h << "index"      << frame.index;

  // save images 
  cv::imwrite(rgb_filename,   frame.rgb_img);
  cv::imwrite(depth_filename, frame.depth_img);
  
  // save intrinsic matrix
  cv::FileStorage fs_mat(intr_filename, cv::FileStorage::WRITE);
  fs_mat << "intr" << frame.intr;

  return true;
}

bool RGBDFrame::load(RGBDFrame& frame, const std::string& path)
{
  // set the filenames
  std::string rgb_filename    = path + "/rgb.png";
  std::string depth_filename  = path + "/depth.png";
  std::string header_filename = path + "/header.yml";
  std::string intr_filename   = path + "/intr.yml"; 

  // check if files exist
  if (!boost::filesystem::exists(rgb_filename)    ||
      !boost::filesystem::exists(depth_filename)  ||
      !boost::filesystem::exists(header_filename) ||
      !boost::filesystem::exists(intr_filename) )
  {
    std::cerr << "files for loading frame not found" << std::endl;
    return false;
  }

  // load header
  cv::FileStorage fs_h(header_filename, cv::FileStorage::READ);
  int seq, sec, nsec;

  fs_h["frame_id"]   >> frame.header.frame_id;
  fs_h["seq"]        >> seq;
  fs_h["stamp_sec"]  >> sec;
  fs_h["stamp_nsec"] >> nsec;

  frame.header.seq        = seq;
  frame.header.stamp.sec  = sec;
  frame.header.stamp.nsec = nsec;

  fs_h["index"] >> frame.index;

  // load images
  frame.rgb_img = cv::imread(rgb_filename);
  frame.depth_img = cv::imread(depth_filename, -1);

  // load intrinsic matrix
  cv::FileStorage fs_mat(intr_filename, cv::FileStorage::READ);
  fs_mat["intr"] >> frame.intr;

  return true;
}

} // namespace rgbdtools
