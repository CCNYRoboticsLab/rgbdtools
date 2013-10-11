/**
 *  @file keyframe_graph_solver_g2o.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com> 
 *  @note based on GraphOptimizer_G2O.cpp by Miguel Algaba Borrego
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

#include "rgbdtools/graph/keyframe_graph_solver_g2o.h"

#include "g2o/core/factory.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"

G2O_USE_TYPE_GROUP(slam3d);

namespace rgbdtools {

KeyframeGraphSolverG2O::KeyframeGraphSolverG2O():
  KeyframeGraphSolver(),
  vertexIdx(0)
{
  // Create the linear solver.
  linear_solver_ = 
      new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();

  // Create the block solver on top of the linear solver.
  block_solver_ = new g2o::BlockSolverX(linear_solver_);

  // Create the algorithm to carry out the optimization.
  optimization_algorithm_ = new 
      g2o::OptimizationAlgorithmLevenberg(block_solver_);

  // create the optimizer to load the data and carry out the optimization
  optimizer_.setVerbose(true);
  optimizer_.setAlgorithm(optimization_algorithm_);
}

KeyframeGraphSolverG2O::~KeyframeGraphSolverG2O()
{
  delete linear_solver_;
  delete block_solver_;
  delete optimization_algorithm_;
}

void KeyframeGraphSolverG2O::solve(
  KeyframeVector& keyframes,
  const KeyframeAssociationVector& associations,
  AffineTransformVector& path)
{  
  // add vertices
  printf("Adding vertices...\n");
  for (unsigned int p_idx = 0; p_idx < path.size(); ++p_idx)
  {
    addVertex(path[p_idx], p_idx);   
  }
  
  // add edges odometry edges 
  printf("Adding VO edges...\n");
  
  if(path.size() > 1)
  {
    InformationMatrix path_inf = InformationMatrix::Identity();
  
    for (unsigned int p_idx = 0; p_idx < path.size()-1; ++p_idx)
    {
      int from_idx = p_idx;
      int to_idx   = p_idx+1;
         
      const AffineTransform& from_pose = path[from_idx];
      const AffineTransform& to_pose   = path[to_idx];
          
      AffineTransform tf = from_pose.inverse() * to_pose;
      
      InformationMatrix inf = path_inf * 100.0;
      addEdge(from_idx, to_idx, tf, inf);
    }
  }
  
  printf("Adding RANSAC edges...\n");
  InformationMatrix ransac_inf = InformationMatrix::Identity();
  
  for (unsigned int as_idx = 0; as_idx < associations.size(); ++as_idx)
  { 
    const KeyframeAssociation& association = associations[as_idx];
    
    // skip non-ransac associations
    if (association.type != KeyframeAssociation::RANSAC) continue;
    
    // get the keyframe indices
    int kf_from_idx = association.kf_idx_a;
    int kf_to_idx   = association.kf_idx_b;
        
    // get the corresponding frame indices (also matches path index)
    int from_idx = keyframes[kf_from_idx].index;
    int to_idx   = keyframes[kf_to_idx].index;
    
    // calculate the information matrix
    int n_matches = association.matches.size();
    InformationMatrix inf = ransac_inf;
    
    // add the edge
    addEdge(from_idx, to_idx, association.a2b, inf);
  }
  
  // run the optimization
  printf("Optimizing...\n");
  optimizeGraph();
  
  // update the path poses
  printf("Updating path poses...\n");
  getOptimizedPoses(path);
  
  // update the keyframe poses
  printf("Updating keyframe poses...\n");
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    RGBDKeyframe& keyframe = keyframes[kf_idx];
    printf("keyframe.index: %d\n", keyframe.index);
    keyframe.pose = path[keyframe.index];
  }
}

void KeyframeGraphSolverG2O::solve(
  KeyframeVector& keyframes,
  const KeyframeAssociationVector& associations)
{  
  // add vertices
  printf("Adding vertices...\n");
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    addVertex(keyframe.pose, kf_idx);   
  }
  
    // add edges odometry edges 
  printf("Adding VO edges...\n");
  
  InformationMatrix inf_identity = InformationMatrix::Identity();
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size()-1; ++kf_idx)
  {
    int from_idx = kf_idx;
    int to_idx   = kf_idx+1;
        
    const AffineTransform& from_pose = keyframes[from_idx].pose;
    const AffineTransform& to_pose   = keyframes[to_idx].pose;
        
    AffineTransform tf = from_pose.inverse() * to_pose;
    
    InformationMatrix inf = inf_identity * 100.0;
    addEdge(from_idx, to_idx, tf, inf);
  }
  
  // add edges
  printf("Adding RANSAC edges...\n");
  for (unsigned int as_idx = 0; as_idx < associations.size(); ++as_idx)
  {
    const KeyframeAssociation& association = associations[as_idx];
    int from_idx = association.kf_idx_a;
    int to_idx   = association.kf_idx_b;
    
    int matches = association.matches.size();
    InformationMatrix inf = inf_identity * matches;
    
    addEdge(from_idx, to_idx, association.a2b, inf);
  }
  
  // run the optimization
  printf("Optimizing...\n");
  optimizeGraph();
  
  // update the keyframe poses
  printf("Updating keyframe poses...\n");

  AffineTransformVector optimized_poses;
  optimized_poses.resize(keyframes.size());
  getOptimizedPoses(optimized_poses);
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    RGBDKeyframe& keyframe = keyframes[kf_idx];
    keyframe.pose = optimized_poses[kf_idx];
  }
}

void KeyframeGraphSolverG2O::addVertex(
  const AffineTransform& vertex_pose,
  int vertex_idx)
{
  // TODO: use eigen quaternion, not manual conversion 
  //Transform Eigen::Matrix4f into 3D traslation and rotation for g2o
  double yaw,pitch,roll; 
  yaw   = atan2f(vertex_pose(1,0),vertex_pose(0,0));
  pitch = asinf(-vertex_pose(2,0));
  roll  = atan2f(vertex_pose(2,1),vertex_pose(2,2));

  g2o::Vector3d t(vertex_pose(0,3),vertex_pose(1,3),vertex_pose(2,3));
  g2o::Quaterniond q;
  q.x()=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
  q.y()=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
  q.z()=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
  q.w()=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);

  g2o::SE3Quat pose(q,t); // vertex pose

  // TODO: smart pointers
  
  // set up node
  g2o::VertexSE3 *vc = new g2o::VertexSE3();
  vc->setEstimate(pose);
  vc->setId(vertex_idx);      

  // set first pose fixed
  if (vertex_idx == 0)
    vc->setFixed(true);

  // add to optimizer
  optimizer_.addVertex(vc);
}

void KeyframeGraphSolverG2O::addEdge(
  int from_idx,
  int to_idx,
  const AffineTransform& relative_pose,
  const Eigen::Matrix<double,6,6>& information_matrix)
{
  // TODO: use eigen quaternion, not manual conversion 
  //Transform Eigen::Matrix4f into 3D traslation and rotation for g2o
  double yaw,pitch,roll;
  yaw   = atan2f(relative_pose(1,0),relative_pose(0,0));
  pitch = asinf(-relative_pose(2,0));
  roll  = atan2f(relative_pose(2,1),relative_pose(2,2));

  g2o::Vector3d t(relative_pose(0,3),relative_pose(1,3),relative_pose(2,3));
  g2o::Quaterniond q;
  q.x()=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
  q.y()=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
  q.z()=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
  q.w()=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);

  // relative transformation
  g2o::SE3Quat transf(q,t); 

  // TODO: smart pointers
  
  g2o::EdgeSE3* edge = new g2o::EdgeSE3;
  edge->vertices()[0] = optimizer_.vertex(from_idx);
  edge->vertices()[1] = optimizer_.vertex(to_idx);
  edge->setMeasurement(transf);

  //Set the information matrix
  edge->setInformation(information_matrix);

  optimizer_.addEdge(edge);
}

void KeyframeGraphSolverG2O::optimizeGraph()
{
  //Prepare and run the optimization
  optimizer_.initializeOptimization();

  //Set the initial Levenberg-Marquardt lambda
  // FIXME(idryanov): This does not compile
  //optimizer_.setUserLambdaInit(0.01);

  //Run optimization
  optimizer_.optimize(20);
}

void KeyframeGraphSolverG2O::getOptimizedPoses(AffineTransformVector& poses)
{
  for (unsigned int idx = 0; idx < poses.size(); ++idx)
  {   
    //Transform the vertex pose from G2O quaternion to Eigen::Matrix4f
    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(
        optimizer_.vertex(idx));
    double optimized_pose_quat[7];
    vertex->getEstimateData(optimized_pose_quat);

    AffineTransform optimized_pose;
    double qx,qy,qz,qr,qx2,qy2,qz2,qr2;

    qx=optimized_pose_quat[3];
    qy=optimized_pose_quat[4];
    qz=optimized_pose_quat[5];
    qr=optimized_pose_quat[6];
    qx2=qx*qx;
    qy2=qy*qy;
    qz2=qz*qz;
    qr2=qr*qr;

    optimized_pose(0,0)=qr2+qx2-qy2-qz2;
    optimized_pose(0,1)=2*(qx*qy-qr*qz);
    optimized_pose(0,2)=2*(qz*qx+qr*qy);
    optimized_pose(0,3)=optimized_pose_quat[0];
    optimized_pose(1,0)=2*(qx*qy+qr*qz);
    optimized_pose(1,1)=qr2-qx2+qy2-qz2;
    optimized_pose(1,2)=2*(qy*qz-qr*qx);
    optimized_pose(1,3)=optimized_pose_quat[1];
    optimized_pose(2,0)=2*(qz*qx-qr*qy);
    optimized_pose(2,1)=2*(qy*qz+qr*qx);
    optimized_pose(2,2)=qr2-qx2-qy2+qz2;
    optimized_pose(2,3)=optimized_pose_quat[2];

    //Set the optimized pose to the vector of poses
    poses[idx] = optimized_pose;
  }
}

} // namespace ccny_rgbd
