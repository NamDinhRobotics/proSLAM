#pragma once

//ds g2o
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

//ds proslam
#include "types/world_map.h"
#include "relocalization/closure.h"

namespace proslam {

//ds this class optimizes the SLAM pose graph by considering pose measurements only, connected landmarks are moved rigidly after the optimization
class GraphOptimizer {

//ds exported data types
public:

  //ds available block solver types (the less numbers, the faster it is - variable being the slowest)
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> BlockSolverVariable;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolver6x3;

  //ds available solvers
  typedef g2o::LinearSolverCSparse<BlockSolverVariable::PoseMatrixType> LinearSolverCSparseVariable;
  typedef g2o::LinearSolverCholmod<BlockSolverVariable::PoseMatrixType> LinearSolverCholmodVariable;
  typedef g2o::LinearSolverCSparse<BlockSolver6x3::PoseMatrixType> LinearSolverCSparse6x3;
  typedef g2o::LinearSolverCholmod<BlockSolver6x3::PoseMatrixType> LinearSolverCholmod6x3;

  //ds available optimization algorithm
  typedef g2o::OptimizationAlgorithmGaussNewton OptimizerGaussNewton;
  typedef g2o::OptimizationAlgorithmLevenberg OptimizerLevenberg;

  //! @brief g2o parameter identifiers
  enum G2oParameter {
    WORLD_OFFSET     = 0,
    CAMERA_LEFT      = 1,
    CAMERA_RIGHT     = 2,
    OFFSET_IMUtoLEFT = 3
  };

//ds object management
PROSLAM_MAKE_PROCESSING_CLASS(GraphOptimizer)

//ds interface
public:

  //! @brief saves a g2o graph of the provided world map to a file
  //! @param[in] world_map_ world map for which the pose graph is constructed
  //! @param[in] file_name_ desired file name for the g2o outfile
  void writePoseGraphToFile(const WorldMap* world_map_, const std::string& file_name_) const;

  //! @brief adds a new frame to the pose graph
  //! @param[in] frame_ the frame to add
  void addFrame(Frame* frame_);

  //! @brief adds a new frame to the pose graph with all connected landmarks
  //! @param[in] frame_ the frame to add including its captured landmarks
  void addFrameWithLandmarks(Frame* frame_);

  //! @brief triggers an adjustment of poses only
  //! @param[in] world_map_ map in which the optimization takes place
  void optimizeFrames(WorldMap* world_map_);

  //! @brief triggers a full bundle adjustment optimization of the current pose graph
  //! @param[in] world_map_ map in which the optimization takes place
  void optimizeFramesWithLandmarks(WorldMap* world_map_);

//ds getters/setters
public:

  const Count numberOfOptimizations() const {return _number_of_optimizations;}

//ds g2o wrapper functions
protected:

  void _setPoseEdge(g2o::OptimizableGraph* optimizer_,
                    g2o::OptimizableGraph::Vertex* vertex_from_,
                    g2o::OptimizableGraph::Vertex* vertex_to_,
                    const TransformMatrix3D& transform_from_to_,
                    const real& information_factor_,
                    const bool& free_translation_ = true,
                    const bool& enable_robust_kernel_ = false) const;

  void _setPointEdge(g2o::OptimizableGraph* optimizer_,
                     g2o::VertexSE3* vertex_frame_,
                     g2o::VertexPointXYZ* vertex_landmark_,
                     const PointCoordinates& framepoint_robot_coordinates,
                     const real& information_factor_) const;

//ds attributes
protected:

  //! @brief g2o optimizer (holding the pose graph)
  g2o::SparseOptimizer* _optimizer;

  //! @brief last frame vertex added (to be locked for optimization)
  g2o::VertexSE3* _vertex_frame_last_added;

  //! @brief bookkeeping: added frames
  std::map<Frame*, g2o::VertexSE3*> _frames_in_pose_graph;

  //! @brief bookkeeping: added local maps
  std::map<const Identifier, LocalMap*> _local_maps_in_graph;

  //! @brief bookkeeping: added landmarks
  std::map<Landmark*, g2o::VertexPointXYZ*> _landmarks_in_pose_graph;

  //ds informative only
  CREATE_CHRONOMETER(addition)
  CREATE_CHRONOMETER(optimization)
  Count _number_of_optimizations = 0;
};
}
