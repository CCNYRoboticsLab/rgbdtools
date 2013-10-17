/**
 *  @file motion_estimation_icp_prob_model.cpp
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

#include "rgbdtools/registration/motion_estimation_klt.h"

namespace rgbdtools {

MotionEstimationKLT::MotionEstimationKLT():
  MotionEstimation()
{

}

MotionEstimationKLT::~MotionEstimationKLT()
{

}

bool MotionEstimationKLT::getMotionEstimationImpl(
  RGBDFrame& frame,
  const AffineTransform& prediction,
  AffineTransform& motion)
{

  return false;
}

} // namespace rgbdtools
