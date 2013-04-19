/**
 *  @file keyframe_graph_solver_g2o.h
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

#ifndef RGBDTOOLS_KEYFRAME_GRAPH_SOLVER_G2O_H
#define RGBDTOOLS_KEYFRAME_GRAPH_SOLVER_G2O_H

#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/types_six_dof_quat.h>

#include "rgbdtools/graph/keyframe_graph_solver.h"

namespace rgbdtools {

/** @brief Graph-based global alignement using g2o (generalized 
 * graph optimizaiton)
 */
class KeyframeGraphSolverG2O: public KeyframeGraphSolver
{
  public:

    /** @brief Constructor from ROS noehandles
     */  
    KeyframeGraphSolverG2O();
    
    /** @brief Default destructor
     */
    ~KeyframeGraphSolverG2O();
 
    /** @brief Main method to call to perform graph solving using g2o.
     * 
     * @param keyframes vector of keyframes
     * @param associations vector of input keyframe associations
     */
    void solve(KeyframeVector& keyframes,
               KeyframeAssociationVector& associations);

    void solve(
      const std::vector<int>& keyframe_pose_indices,
      std::vector<AffineTransform>& path,
      KeyframeAssociationVector& associations);
    
  private:

    int vertexIdx;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    g2o::BlockSolverX * solver_ptr;
        
    /** @brief Adds a vertex to the g2o structure
     */
    void addVertex(const AffineTransform& vertex_pose,
                   int vertex_idx);
    
    /** @brief Adds an edge to the g2o structure
     */
    void addEdge(int from_idx,  int to_idx,
                 const AffineTransform& relative_pose,
                 const Eigen::Matrix<double,6,6>& information_matrix);
    
    /** @brief runs the optimization
     */
    void optimizeGraph();
    
    /** @brief copies the (optimized) poses from the g2o structure into
     * the keyframe vector
     * @param keyframes the vector of keyframes to modify the poses
     */
    //void updatePoses(KeyframeVector& keyframes);
    
    void getOptimizedPoses(std::vector<AffineTransform>& poses);
};

} // namespace rgbdtools

#endif // RGBDTOOLS_KEYFRAME_GRAPH_SOLVER_G2O_H
