#pragma once

//#define SRRG_PROSLAM_USE_G3O

#ifdef SRRG_PROSLAM_USE_G3O
#include "g2o_slim/sparse_optimizer.h"
#else
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam3d/types_slam3d.h"
#endif

#include "types/world_map.h"

namespace proslam {

  //ds this class optimizes the SLAM pose graph by considering pose measurements only, connected landmarks are moved rigidly after the optimization
  class GraphOptimizer {

  //ds exported data types
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#ifndef SRRG_PROSLAM_USE_G3O
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
#endif

  //ds object management
  public:

    GraphOptimizer();
    ~GraphOptimizer();

  //ds interface
  public:

    void optimize(WorldMap* context_);
    void optimizePoses(WorldMap* context_);
    void optimizeLandmarks(WorldMap* context_);

  //ds getters/setters
  public:

    const Count numberOfOptimizations() const {return _number_of_optimizations;}

  //ds g2o wrapper functions
  public:

#ifdef SRRG_PROSLAM_USE_G3O

    static void setPoseEdge(g2o_slim::SparseOptimizer* optimizer_,
                            const Identifier& id_from_,
                            const Identifier& id_to_,
                            const TransformMatrix3D& transform_from_to_,
                            Eigen::Matrix<double, 6, 6> information_ = Eigen::Matrix<double, 6, 6>::Identity()) {

      //ds free translational component
      information_.block<3,3>(0,0) *= 1e-4;

      //ds add the edge (TODO flip indices once g2o_slim is updated)
      optimizer_->addEdge(id_to_, id_from_, transform_from_to_, information_);
    }

    static g2o_slim::SparseOptimizer* getOptimizer() {

      //ds configure optimizer at instantiation
      const Count number_of_iterations = 10;
      const real maximum_error_kernel  = 1e3;
      return new g2o_slim::SparseOptimizer(number_of_iterations, maximum_error_kernel);
    }

#else

    static void setPoseEdge(g2o::SparseOptimizer* optimizer_,
                            const Identifier& id_from_,
                            const Identifier& id_to_,
                            const TransformMatrix3D& transform_from_to_,
                            Eigen::Matrix<double, 6, 6> information_ = Eigen::Matrix<double, 6, 6>::Identity()) {
      g2o::EdgeSE3* edge_pose = new g2o::EdgeSE3();
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
      optimizer_->addEdge(edge_pose);
    }

    static g2o::SparseOptimizer* getOptimizer() {
      SlamLinearSolver* linearSolver = new SlamLinearSolver();
      linearSolver->setBlockOrdering(true);
      SlamBlockSolver* blockSolver=new SlamBlockSolver(linearSolver);
      g2o::OptimizationAlgorithmGaussNewton* solverGauss = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
      g2o::SparseOptimizer * optimizer = new g2o::SparseOptimizer();
      optimizer->setAlgorithm(solverGauss);
      return optimizer;
    }

#endif

  //ds attributes
  protected:

//ds optimization
#ifdef SRRG_PROSLAM_USE_G3O
    g2o_slim::SparseOptimizer* _optimizer;
#else
    g2o::SparseOptimizer* _optimizer;
#endif

    //ds informative only
    CREATE_CHRONOMETER(overall)
    Count _number_of_optimizations = 0;
  };
}
