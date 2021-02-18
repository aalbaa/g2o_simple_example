// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <cmath>

#include "vertex_se2.h"
#include "vertex_R2.h"
#include "edge_se2.h"
#include "edge_R2.h"
#include "types_tutorial_slam2d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

int main()
{
  int numPoses = 10;
  int numLandmarks = 1;
  int numNodes = numPoses + numLandmarks;

  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(
    g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);

  // adding the odometry to the optimizer
  // first adding all the vertices
  cerr << "Optimization: Adding robot poses ... ";
  for (size_t i = 0; i < numPoses; ++i) {
    // VertexSE2* robot =  new VertexSE2;
    // robot->setId(i);
    // robot->setEstimate( SE2(1., 0., 0.));
    VertexR2* robot =  new VertexR2;
    robot->setId(i);
    robot->setEstimate( Vector2(0, 0));
    optimizer.addVertex(robot);
  }
  cerr << "done." << endl;

  // second add the odometry constraints
  cerr << "Optimization: Adding odometry measurements ... ";
  for (size_t i = 0; i < numPoses - 1; ++i) {
    // EdgeSE2* odometry = new EdgeSE2;
    EdgeR2* odometry = new EdgeR2;
    odometry->vertices()[0] = optimizer.vertex( i);
    odometry->vertices()[1] = optimizer.vertex( i + 1);
    // odometry->setMeasurement( SE2(1.0, 0., 0.));
    odometry->setMeasurement( Vector2(1.0, 0.));
    // odometry->setInformation( Eigen::Matrix3d::Identity());
    odometry->setInformation( Eigen::Matrix2d::Identity());
    optimizer.addEdge(odometry);
  }
  cerr << "done." << endl;

  cerr << "Adding loop closures....";
  // Add loop closure edge
  // EdgeSE2* lc = new EdgeSE2;
  EdgeR2* lc = new EdgeR2;
  lc->vertices()[0] = optimizer.vertex( 0);
  lc->vertices()[1] = optimizer.vertex( numPoses - 1);
  // lc->setMeasurement( SE2( numPoses - 1 + 0.1, 0., 0.));
  lc->setMeasurement( Vector2( numPoses - 1 + 0.1, 0.));
  lc->setInformation( 0.1 * Eigen::Matrix2d::Identity());
  optimizer.addEdge( lc);

  cerr << "done." << endl;
  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // dump initial state to the disk
  optimizer.save("tutorial_before.g2o");

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  // VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
  VertexR2* firstRobotPose = dynamic_cast<VertexR2*>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  cerr << "Optimizing" << endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cerr << "done." << endl;

  optimizer.save("tutorial_after.g2o");

  // freeing the graph memory
  optimizer.clear();

  return 0;
}
