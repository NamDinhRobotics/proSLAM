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
#include "types/world_map.h"

namespace proslam {

//ds this class optimizes the SLAM pose graph by considering pose measurements only, connected landmarks are moved rigidly after the optimization
class GraphOptimizer {

//ds exported data types
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  //! @brief g2o parameter identifiers
  enum G2oParameter {
    WORLD_OFFSET     = 0,
    CAMERA_LEFT      = 1,
    CAMERA_RIGHT     = 2,
    OFFSET_IMUtoLEFT = 3
  };

//ds object management
public:

  GraphOptimizer();
  ~GraphOptimizer();

//ds interface
public:

  //! @brief adds a new frame to the pose graph with all connected landmarks
  //! @param[in] frame_ the frame to add
  void addFrame(Frame* frame_);

  //! @brief triggers a full bundle adjustment optimization of the current pose graph
  void optimize();

//ds getters/setters
public:

  const Count numberOfOptimizations() const {return _number_of_optimizations;}

//ds g2o wrapper functions
protected:

  void _setPoseEdge(g2o::SparseOptimizer* optimizer_,
                    g2o::OptimizableGraph::Vertex* vertex_from_,
                    g2o::OptimizableGraph::Vertex* vertex_to_,
                    const TransformMatrix3D& transform_from_to_,
                    const real& information_factor_);

  void _setPointEdge(g2o::SparseOptimizer* optimizer_,
                     g2o::VertexSE3* vertex_frame_,
                     g2o::VertexPointXYZ* vertex_landmark_,
                     const PointCoordinates& framepoint_robot_coordinates,
                     const real& information_factor_);

//ds attributes
protected:

  //! @brief g2o optimizer (holding the pose graph)
  g2o::SparseOptimizer* _optimizer;

  //! @brief identifier space between frames and landmarks (1mio frames)
  Count _identifier_space = 1000000;

  //! @brief last frame vertex added (to be locked for optimization)
  g2o::VertexSE3* _vertex_frame_last_added;

  //! @brief bookkeeping: added frames
  std::map<Frame*, g2o::VertexSE3*> _frames_in_pose_graph;

  //! @brief bookkeeping: added landmarks
  std::map<Landmark*, g2o::VertexPointXYZ*> _landmarks_in_pose_graph;

  //! @brief base frame weight in graph TODO move to params
  const real base_information_frame = 1e5;

  //! @brief enable robust kernel for landmark measurements TODO move to params
  bool enable_robust_kernel_for_landmark_measurements = false;

  //ds informative only
  CREATE_CHRONOMETER(addition)
  CREATE_CHRONOMETER(optimization)
  Count _number_of_optimizations = 0;
};
}
