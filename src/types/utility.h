#pragma once
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "items/appearance.h"
#include "items/landmark_item.h"

namespace proslam {

  //ds TODO this class MUST be purged
  class Utility {

  //ds exported data types
  public:

    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  //ds helpers
  public:

    static const Eigen::Matrix< real, 3, 1 > toOrientationRodrigues( const Eigen::Matrix< real, 3, 3 >& p_matRotation )
    {
        //ds orientation
        cv::Mat vecOrientation;

        //ds fill the matrix
        cv::Rodrigues( toCVMatrix( p_matRotation ), vecOrientation );

        //ds return in Eigen format
        return fromCVVector< real, 3 >( vecOrientation );
    }

    static const cv::Mat_< real > toCVMatrix( const Eigen::Matrix< real, 3, 3 >& p_matEigen )
    {
        //ds allocate cv vector
        cv::Mat_< real > matCV( 3, 3 );

        //ds fill the vector (column major)
        for( uint32_t u = 0; u < 3; ++u )
        {
            for( uint32_t v = 0; v < 3; ++v )
            {
                matCV.at< real >( u, v ) = p_matEigen( u, v );
            }
        }

        return matCV;
    }

    template< typename tType, uint32_t uRows > static const Eigen::Matrix< tType, uRows, 1 > fromCVVector( const cv::Vec< tType, uRows >& p_vecCV )
    {
        //ds allocate eigen matrix
        Eigen::Matrix< tType, uRows, 1 > vecEigen;

        //ds fill the vector (column major)
        for( uint32_t u = 0; u < uRows; ++u )
        {
            vecEigen( u ) = p_vecCV( u );
        }

        return vecEigen;
    }

    //ds TODO remove
    static const HBSTNode::BinaryMatchableVector getMatchables(const AppearancePtrVector& appearances_) {
      assert(appearances_.size() > 0);
      HBSTNode::BinaryMatchableVector matchables(appearances_.size());

      //ds copy raw data
      for (Index index_appearance = 0; index_appearance < appearances_.size(); ++index_appearance) {
        matchables[index_appearance] = new HBSTMatchable(index_appearance, appearances_[index_appearance]->descriptor);
      }
      return matchables;
    }

    //ds TODO remove
    static const AppearancePtrVector getAppearances(const FramePointPtrVector& framepoints_) {
      assert(framepoints_.size() > 0);
      AppearancePtrVector appearances(framepoints_.size());

      //ds copy raw data
      Count number_of_added_appearances = 0;
      for (Index index_appearance = 0; index_appearance < appearances.size(); ++index_appearance) {
        FramePoint* point = framepoints_[index_appearance];
        if (point->landmark() != 0) {
          appearances[number_of_added_appearances] = new Appearance(new LandmarkItem(point->landmark(), point->robotCoordinates()), point->descriptor());
          ++number_of_added_appearances;
        }
      }

      appearances.resize(number_of_added_appearances);
      return appearances;
    }

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
      g2o::OptimizationAlgorithmGaussNewton* solverGauss = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
      g2o::SparseOptimizer * optimizer = new g2o::SparseOptimizer();
      optimizer->setAlgorithm(solverGauss);
      return optimizer;
    }
  };
}
