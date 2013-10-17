/**
 *  @file motion_estimation_icp_prob_model.h
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

#ifndef RGBDTOOLS_MOTION_ESTIMATION_KLT_H
#define RGBDTOOLS_MOTION_ESTIMATION_KLT_H

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "rgbdtools/types.h"
#include "rgbdtools/registration/motion_estimation.h"

namespace rgbdtools {

/** @brief Motion estimation based on aligning sparse features
 * against a persistent, dynamic model.
 * 
 * The model is build from incoming features through a Kalman Filter
 * update step.
 */  
class MotionEstimationKLT: public MotionEstimation
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief Constructor from ROS noehandles
     * @param nh the public nodehandle
     * @param nh_private the private notehandle
     */
    MotionEstimationKLT();
    
    /** @brief Default destructor
     */    
    virtual ~MotionEstimationKLT();

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
    
  private:

 };

} // namespace rgbdtools

#endif // RGBDTOOLS_MOTION_ESTIMATION_KLT_H

