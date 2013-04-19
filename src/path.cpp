/**
 *  @file rgbd_keyframe.cpp
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

#include "rgbdtools/path.h"

namespace rgbdtools {
  
bool savePath(
  const AffineTransformVector& path, 
  const std::string& file_path)
{
  std::string filename  = file_path + "/path.txt";
  
  return false;
}

bool loadPath(
  AffineTransformVector& path, 
  const std::string& file_path)
{
  std::string filename  = file_path + "/path.txt";
 
  return false;
}
  

} // namespace rgbdtools
