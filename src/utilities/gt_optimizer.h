#pragma once
#include <dirent.h>
#include "types/gt_defs.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "gt_closure_buffer.h"
#include "gt_closure_checker.h"

namespace gslam {
  class Optimizer {

  //ds exported data types
  public:

    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    //typedef g2o::BlockSolverX  SlamBlockSolver;
    //typedef g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType> SlamLinearSolver;

  public:

    static g2o::EdgeSE3* getPoseEdge(g2o::SparseOptimizer* optimizer_,
                                     const Identifier& id_from_,
                                     const Identifier& id_to_,
                                     const TransformMatrix3D& transform_from_to_,
                                     Eigen::Matrix<double, 6, 6> information_ = Eigen::Matrix<double, 6, 6>::Identity()) {
      g2o::EdgeSE3* edge_pose = new g2o::EdgeSE3;
      g2o::OptimizableGraph::Vertex* vertex_keyframe_from = optimizer_->vertex(id_from_);
      g2o::OptimizableGraph::Vertex* vertex_keyframe_to   = optimizer_->vertex(id_to_);
      assert(0 != vertex_keyframe_from);
      assert(0 != vertex_keyframe_to);
      edge_pose->setVertex(1, vertex_keyframe_from);
      edge_pose->setVertex(0, vertex_keyframe_to);
      edge_pose->setMeasurement(transform_from_to_.cast<double>());

      //ds free translational component
      information_.block<3,3>(0,0) *= 1e-4;
      edge_pose->setInformation(information_);
      return edge_pose;
    }
    static g2o::EdgeSE3* getClosureEdge(g2o::SparseOptimizer* optimizer_,
                                        const Identifier& id_query_,
                                        const Identifier& id_reference_,
                                        const TransformMatrix3D& transform_query_to_reference_,
                                        Eigen::Matrix<double, 6, 6> information_ = Eigen::Matrix<double, 6, 6>::Identity()) {
      g2o::EdgeSE3* edge_closure = new g2o::EdgeSE3;
      g2o::OptimizableGraph::Vertex* vertex_keyframe_from = optimizer_->vertex(id_query_);
      g2o::OptimizableGraph::Vertex* vertex_keyframe_to   = optimizer_->vertex(id_reference_);
      vertex_keyframe_to->setFixed(true);
      assert(0 != vertex_keyframe_from);
      assert(0 != vertex_keyframe_to);
      edge_closure->setVertex(1, vertex_keyframe_from);
      edge_closure->setVertex(0, vertex_keyframe_to);
      edge_closure->setMeasurement(transform_query_to_reference_.cast<double>());
      edge_closure->setInformation(information_);
      return edge_closure;
    }

    static g2o::SparseOptimizer * getOptimizer() {
      SlamLinearSolver* linearSolver = new SlamLinearSolver();
      linearSolver->setBlockOrdering(true);
      SlamBlockSolver* blockSolver=new SlamBlockSolver(linearSolver);
      //g2o::OptimizationAlgorithmLevenberg* solverGauss = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
      g2o::OptimizationAlgorithmGaussNewton* solverGauss = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
      g2o::SparseOptimizer * optimizer = new g2o::SparseOptimizer();
      optimizer->setAlgorithm(solverGauss);
      return optimizer;
    }

    static void clearFilesUNIX() {

      //ds directory handle
      DIR *pDirectory = opendir("g2o");

      //ds try to open the directory
      if (0 != pDirectory) {

        //ds file handle
        struct dirent *pFile;

        //ds delete all files
        while ((pFile = readdir(pDirectory)) != NULL) {

          //ds validate filename
          if ('.' != pFile->d_name[0]) {

              //ds construct full filename and delete
              std::string strFile("g2o/");
              strFile += pFile->d_name;
              //std::printf( "<Cg2oOptimizer>(clearFiles) removing file: %s\n", strFile.c_str( ) );
              std::remove(strFile.c_str());
          }
        }

        //ds close directory
        closedir(pDirectory);
      }
    }
  }; //class Mapper
} //namespace gtracker
