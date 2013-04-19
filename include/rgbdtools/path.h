/**
 *  @file path.h
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

#ifndef RGBDTOOLS_RGBD_PATH_H
#define RGBDTOOLS_RGBD_PATH_H

#include <vector>
#include <Eigen/Core>

#include "rgbdtools/types.h"

namespace rgbdtools {

typedef Eigen::aligned_allocator<AffineTransform> AffineTransformAllocator;
typedef std::vector<AffineTransform, AffineTransformAllocator> AffineTransformVector;

bool savePoses(const AffineTransformVector& path, 
               const std::string& file_path);

bool loadPoses(AffineTransformVector& path, 
               const std::string& file_path);

} // namespace rgbdtools

#endif // RGBDTOOLS_RGBD_PATH_H
