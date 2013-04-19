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

namespace rgbdtools {

KeyframeGraphSolverG2O::KeyframeGraphSolverG2O():
  KeyframeGraphSolver(),
  vertexIdx(0)
{
  optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  optimizer.setVerbose(false);
  
  linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
  solver_ptr = new g2o::BlockSolverX(&optimizer, linearSolver);
  
  optimizer.setSolver(solver_ptr);
}

KeyframeGraphSolverG2O::~KeyframeGraphSolverG2O()
{

}

void KeyframeGraphSolverG2O::solve(
  const std::vector<int>& keyframe_pose_indices,
  std::vector<AffineTransform>& path,
  KeyframeAssociationVector& associations)
{  
  // add vertices
  printf("Adding vertices...\n");
  for (unsigned int p_idx = 0; p_idx < path.size(); ++p_idx)
  {
    addVertex(path[p_idx], p_idx);   
  }
  
  printf("Adding VO edges...\n");
  // add edges odometry edges
  assert(path.size() > 1);
  for (unsigned int p_idx = 0; p_idx < path.size()-1; ++p_idx)
  {
    int from_idx = p_idx;
    int to_idx   = p_idx+1;
       
    const AffineTransform& from_pose = path[from_idx];
    const AffineTransform& to_pose   = path[to_idx];
    
    Eigen::Matrix<double,6,6> inf = Eigen::Matrix<double,6,6>::Identity();
    
    AffineTransform tf = from_pose.inverse() * to_pose;
    
    addEdge(from_idx, to_idx, tf, inf);
  }
  
  printf("Adding RANSAC edges...\n");
  for (unsigned int as_idx = 0; as_idx < associations.size(); ++as_idx)
  { 
    const KeyframeAssociation& association = associations[as_idx];
    
    if (association.type != KeyframeAssociation::RANSAC) continue;
    
    int from_idx = association.kf_idx_a;
    int to_idx   = association.kf_idx_b;
    
    // get indices from the path
    from_idx =  keyframe_pose_indices[from_idx];
    to_idx   =  keyframe_pose_indices[to_idx];
    
    int matches = association.matches.size();
    
    Eigen::Matrix<double,6,6> inf = Eigen::Matrix<double,6,6>::Identity();
    inf = inf * matches;
    
    addEdge(from_idx, to_idx, association.a2b, inf);
  }
  
  // run the optimization
  printf("Optimizing...\n");
  optimizeGraph();
  
  // update the poses
  printf("Updating poses...\n");

  std::vector<AffineTransform> optimized_poses;
  optimized_poses.resize(path.size());
  getOptimizedPoses(optimized_poses);
  
  for (unsigned int p_idx = 0; p_idx < path.size(); ++p_idx)
  {
    path[p_idx] = optimized_poses[p_idx];
  }
}

void KeyframeGraphSolverG2O::solve(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{  
  // add vertices
  printf("Adding vertices...\n");
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    addVertex(keyframe.pose, kf_idx);   
  }
  
  // add edges
  printf("Adding edges...\n");
  for (unsigned int as_idx = 0; as_idx < associations.size(); ++as_idx)
  {
    const KeyframeAssociation& association = associations[as_idx];
    int from_idx = association.kf_idx_a;
    int to_idx   = association.kf_idx_b;
    
    int matches = association.matches.size();
    
    Eigen::Matrix<double,6,6> inf = Eigen::Matrix<double,6,6>::Identity();
    
    if (matches == 0)
    {
      // this is an odometry edge 
      inf = inf * 100.0;
    }
    else
    {
      // this is an SURF+RANSAC edge 
      inf = inf * matches;
    }
    
    addEdge(from_idx, to_idx, association.a2b, inf);
  }
  
  // run the optimization
  printf("Optimizing...\n");
  optimizeGraph();
  
  // update the poses
  printf("Updating poses...\n");

  std::vector<AffineTransform> optimized_poses;
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
  vc->estimate() = pose;
  vc->setId(vertex_idx);      

  // set first pose fixed
  if (vertex_idx == 0)
    vc->setFixed(true);

  // add to optimizer
  optimizer.addVertex(vc);
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
  edge->vertices()[0] = optimizer.vertex(from_idx);
  edge->vertices()[1] = optimizer.vertex(to_idx);
  edge->setMeasurement(transf);

  //Set the information matrix
  edge->setInformation(information_matrix);

  optimizer.addEdge(edge);
}

void KeyframeGraphSolverG2O::optimizeGraph()
{
  //Prepare and run the optimization
  optimizer.initializeOptimization();

  //Set the initial Levenberg-Marquardt lambda
  optimizer.setUserLambdaInit(0.01);

  //Run optimization
  optimizer.optimize(10);
}

/*
void KeyframeGraphSolverG2O::updatePoses(
  KeyframeVector& keyframes)
{
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    RGBDKeyframe& keyframe = keyframes[kf_idx];
    keyframe.pose = optimized_poses[kf_idx];
  }
}*/

void KeyframeGraphSolverG2O::getOptimizedPoses(std::vector<AffineTransform>& poses)
{
  for (unsigned int idx = 0; idx < poses.size(); ++idx)
  {   
    //Transform the vertex pose from G2O quaternion to Eigen::Matrix4f
    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(idx));
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
