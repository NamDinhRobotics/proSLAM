#include "graph_optimizer.h"
#include "g2o/core/robust_kernel_impl.h"

//ds backwards compatibility with g2o
#ifdef SRRG_PROSLAM_G2O_HAS_NEW_OWNERSHIP_MODEL
#define ALLOCATE_SOLVER(OPTIMIZATION_ALGORITHM_, SOLVER_TYPE_, BLOCK_TYPE_) \
  std::unique_ptr<SOLVER_TYPE_> linear_solver = g2o::make_unique<SOLVER_TYPE_>(); \
  linear_solver->setBlockOrdering(true); \
  std::unique_ptr<BLOCK_TYPE_> block_solver = g2o::make_unique<BLOCK_TYPE_>(std::move(linear_solver)); \
  solver = new OPTIMIZATION_ALGORITHM_(std::move(block_solver));
#else
#define ALLOCATE_SOLVER(OPTIMIZATION_ALGORITHM_, SOLVER_TYPE_, BLOCK_TYPE_) \
  SOLVER_TYPE_* linear_solver = new SOLVER_TYPE_(); \
  linear_solver->setBlockOrdering(true); \
  BLOCK_TYPE_* block_solver = new BLOCK_TYPE_(linear_solver); \
  solver = new OPTIMIZATION_ALGORITHM_(block_solver);
#endif

namespace proslam {

GraphOptimizer::GraphOptimizer(GraphOptimizerParameters* parameters_): _parameters(parameters_),
                                                                       _optimizer(nullptr),
                                                                       _vertex_local_map_last_added(nullptr) {
  LOG_INFO(std::cerr << "GraphOptimizer::GraphOptimizer|constructed" << std::endl)
}

void GraphOptimizer::configure() {
  LOG_INFO(std::cerr << "GraphOptimizer::configure|configuring" << std::endl)

  //ds solver setup
  g2o::OptimizationAlgorithm* solver = nullptr;

  //ds allocate an optimizable graph - depending on chosen parameters
  if (_parameters->optimization_algorithm == "GAUSS_NEWTON" &&
      _parameters->linear_solver_type == "CHOLMOD" &&
      !_parameters->enable_full_bundle_adjustment) {
    ALLOCATE_SOLVER(OptimizerGaussNewton, LinearSolverCholmod6x3, BlockSolver6x3)
  }
  else if (_parameters->optimization_algorithm == "GAUSS_NEWTON" &&
      _parameters->linear_solver_type == "CSPARSE" &&
      !_parameters->enable_full_bundle_adjustment) {
    ALLOCATE_SOLVER(OptimizerGaussNewton, LinearSolverCSparse6x3, BlockSolver6x3)
  }

  else if (_parameters->optimization_algorithm == "GAUSS_NEWTON" &&
      _parameters->linear_solver_type == "CHOLMOD" &&
      _parameters->enable_full_bundle_adjustment) {
    ALLOCATE_SOLVER(OptimizerGaussNewton, LinearSolverCholmodVariable, BlockSolverVariable)
  }
  else if (_parameters->optimization_algorithm == "GAUSS_NEWTON" &&
      _parameters->linear_solver_type == "CSPARSE" &&
      _parameters->enable_full_bundle_adjustment) {
    ALLOCATE_SOLVER(OptimizerGaussNewton, LinearSolverCSparseVariable, BlockSolverVariable)
  }

  else if (_parameters->optimization_algorithm == "LEVENBERG" &&
      _parameters->linear_solver_type == "CHOLMOD" &&
      !_parameters->enable_full_bundle_adjustment) {
    ALLOCATE_SOLVER(OptimizerLevenberg, LinearSolverCholmod6x3, BlockSolver6x3)
  }
  else if (_parameters->optimization_algorithm == "LEVENBERG" &&
      _parameters->linear_solver_type == "CSPARSE" &&
      !_parameters->enable_full_bundle_adjustment) {
    ALLOCATE_SOLVER(OptimizerLevenberg, LinearSolverCSparse6x3, BlockSolver6x3)
  }

  else if (_parameters->optimization_algorithm == "LEVENBERG" &&
      _parameters->linear_solver_type == "CHOLMOD" &&
      _parameters->enable_full_bundle_adjustment) {
    ALLOCATE_SOLVER(OptimizerLevenberg, LinearSolverCholmodVariable, BlockSolverVariable)
  }
  else if (_parameters->optimization_algorithm == "LEVENBERG" &&
      _parameters->linear_solver_type == "CSPARSE" &&
      _parameters->enable_full_bundle_adjustment) {
    ALLOCATE_SOLVER(OptimizerLevenberg, LinearSolverCSparseVariable, BlockSolverVariable)
  }

  //ds if we couldn't allocate a solver
  if (!solver) {

    //ds critical
    throw std::runtime_error("GraphOptimizer::configure|unable to set solver, please check configuration");
  }

  //ds allocate optimizer (deleting a previous one)
  if (_optimizer) {delete _optimizer;}
  _optimizer = new g2o::SparseOptimizer();

  //ds set the solver
  _optimizer->setAlgorithm(solver);

  //ds clean bookkeeping
  _vertex_local_map_last_added = 0;
  _frames_in_pose_graph.clear();
  _local_maps_in_graph.clear();
  _landmarks_in_pose_graph.clear();

  //ds clean pose graph
  _optimizer->clear();
  _optimizer->setVerbose(false);

  //ds set world offset
  g2o::ParameterSE3Offset* parameter_world_offset = new g2o::ParameterSE3Offset();
  parameter_world_offset->setId(G2oParameter::WORLD_OFFSET);
  _optimizer->addParameter(parameter_world_offset);
  LOG_INFO(std::cerr << "GraphOptimizer::configure|allocated optimization algorithm: " << _parameters->optimization_algorithm
                     << " with solver: " << _parameters->linear_solver_type << std::endl)
  LOG_INFO(std::cerr << "GraphOptimizer::configure|configured" << std::endl)
}

GraphOptimizer::~GraphOptimizer(){
  LOG_INFO(std::cerr << "GraphOptimizer::~GraphOptimizer|destroying" << std::endl)
  _frames_in_pose_graph.clear();
  _landmarks_in_pose_graph.clear();
  if (_optimizer) {
    _optimizer->clear();
    _optimizer->clearParameters();
    delete _optimizer;
  }
  LOG_INFO(std::cerr << "GraphOptimizer::~GraphOptimizer|destroyed" << std::endl)
}

void GraphOptimizer::writePoseGraphToFile(const WorldMap* world_map_, const std::string& file_name_) const {
  if (world_map_->frames().empty()) {
    return;
  }
  LOG_INFO(std::cerr << "GraphOptimizer::writePoseGraphToFile|saving final pose graph to file: " << file_name_ << std::endl)

  //ds get a graph handle
  g2o::OptimizableGraph* pose_graph = new g2o::OptimizableGraph();
  g2o::VertexSE3* vertex_frame_last_added = 0;
  std::map<Landmark*, g2o::VertexPointXYZ*, std::less<Landmark*>, Eigen::aligned_allocator<std::pair<Landmark*, g2o::VertexPointXYZ*>>> landmarks_in_pose_graph;

  //ds set world parameter (required for landmark EdgeSE3PointXYZ measurements)
  g2o::ParameterSE3Offset* parameter_world_offset = new g2o::ParameterSE3Offset();
  parameter_world_offset->setId(G2oParameter::WORLD_OFFSET);
  pose_graph->addParameter(parameter_world_offset);

  //ds loop over all frames - adding frames and landmarks
  for (const FramePointerMapElement& frame: world_map_->frames()) {

    //ds get the frames pose to g2o representation
    g2o::VertexSE3* vertex_frame_current = new g2o::VertexSE3();
    vertex_frame_current->setId(frame.second->identifier());
    vertex_frame_current->setEstimate(frame.second->robotToWorld().cast<double>());
    pose_graph->addVertex(vertex_frame_current);

    //ds if its the first frame to be added (start or recently cleared pose graph)
    if (!vertex_frame_last_added) {

      //ds fix the initial vertex - no measurement to add
      vertex_frame_current->setFixed(true);
    } else {

      //ds we can connect it to the preceeding frame by adding the odometry measurement
      _setPoseEdge(pose_graph,
                  vertex_frame_current,
                  vertex_frame_last_added,
                  (frame.second->previous()->worldToRobot()*frame.second->robotToWorld()).cast<double>(),
                  _parameters->base_information_frame);
    }

    //ds add landmark measurements contained in the frame by scanning its framepoints
    for (FramePoint* framepoint: frame.second->points()) {

      //ds if the framepoint is linked to a landmark
      Landmark* landmark = framepoint->landmark();
      if (landmark) {
        g2o::VertexPointXYZ* vertex_landmark = 0;

        //ds check if the landmark not yet present in the graph
        if (landmarks_in_pose_graph.find(landmark) == landmarks_in_pose_graph.end()) {

          //ds allocate a new point vertex and add it to the graph
          vertex_landmark = new g2o::VertexPointXYZ( );
          vertex_landmark->setEstimate(landmark->coordinates().cast<double>());
          vertex_landmark->setId(landmark->identifier()+_parameters->identifier_space);
          pose_graph->addVertex(vertex_landmark);

          //ds bookkeep the landmark
          landmarks_in_pose_graph.insert(std::make_pair(landmark, vertex_landmark));
        } else {

          //ds retrieve existing vertex using our bookkeeping container
          vertex_landmark = landmarks_in_pose_graph[landmark];
        }

        //ds add framepoint position as measurement for the landmark
        _setPointEdge(pose_graph, vertex_frame_current, vertex_landmark, framepoint->robotCoordinates(), 1/framepoint->depthMeters());
      }
    }

    //ds next frame
    vertex_frame_last_added = vertex_frame_current;
  }

  //ds loop over all frames - adding loop closures
  for (const FramePointerMapElement& frame: world_map_->frames()) {

    //ds if the frame carries loop closures
    if (frame.second->localMap() && frame.second->localMap()->closures().size() > 0) {

      //ds for all linked loop closures
      for (const Closure::ClosureConstraint& closure: frame.second->localMap()->closures()) {

        //ds retrieve closure edge
        _setPoseEdge(pose_graph,
                     pose_graph->vertex(frame.second->localMap()->keyframe()->identifier()),
                     pose_graph->vertex(closure.local_map->keyframe()->identifier()),
                     closure.relation,
                     _parameters->base_information_frame);
      }
    }
  }

  //ds save the graph to disk
  pose_graph->save(file_name_.c_str());
  pose_graph->clear();
  landmarks_in_pose_graph.clear();
  LOG_INFO(std::cerr << "GraphOptimizer::writePoseGraphToFile|pose graph saved" << std::endl)
}

void GraphOptimizer::addPose(LocalMap* local_map_) {
  CHRONOMETER_START(addition)
  const g2o::Isometry3 local_map_to_world = local_map_->robotToWorld().cast<double>();

  //ds get the frames pose to g2o representation
  g2o::VertexSE3* vertex_current = new g2o::VertexSE3();
  vertex_current->setId(local_map_->identifier());
  vertex_current->setEstimate(local_map_to_world);
  _optimizer->addVertex(vertex_current);

  //ds if its the first frame to be added (start or recently cleared pose graph)
  if (!_vertex_local_map_last_added) {

    //ds fix the initial vertex - no measurement to add
    vertex_current->setFixed(true);
  } else {
    const g2o::Isometry3& world_to_local_map_previous = local_map_->previous()->worldToRobot().cast<double>();

    //ds compute information value based on landmark content
    real information_factor = _parameters->base_information_frame;

    //ds we can connect it to the preceeding frame by adding the odometry measurement
    _setPoseEdge(_optimizer,
                vertex_current,
                _vertex_local_map_last_added,
                world_to_local_map_previous*local_map_to_world,
                information_factor);
  }

  //ds if the local map has not been checked in a previous frame
  assert(_local_maps_in_graph.find(local_map_->identifier()) == _local_maps_in_graph.end());

  //ds add a new local map
  _local_maps_in_graph.insert(std::make_pair(local_map_->identifier(), local_map_));

  //ds for all loop closures on this local map
  for (const Closure::ClosureConstraint& closure: local_map_->closures()) {

    //ds compute information value (closure edges weight much more than pose edges to be able to deform the graph properly)
    const real information_factor = _parameters->base_information_frame*closure.omega*10;

    //ds retrieve reference frame (must be present)
    g2o::VertexSE3* vertex_reference = dynamic_cast<g2o::VertexSE3*>(_optimizer->vertex(closure.local_map->identifier()));
    assert(vertex_reference);

    //ds introduce loop closure constraint between the two local maps
    _setPoseEdge(_optimizer, vertex_current, vertex_reference, closure.relation, information_factor);
  }

  //ds bookkeep the added frame
  _vertex_local_map_last_added = vertex_current;
  CHRONOMETER_STOP(addition)
}

void GraphOptimizer::addPoseWithFactors(Frame* frame_) {
  CHRONOMETER_START(addition)

  //ds get the frames pose to g2o representation
  g2o::VertexSE3* vertex_frame_current = new g2o::VertexSE3();
  vertex_frame_current->setId(frame_->identifier());
  vertex_frame_current->setEstimate(frame_->robotToWorld().cast<double>());
  _optimizer->addVertex(vertex_frame_current);

  //ds if its the first frame to be added (start or recently cleared pose graph)
  if (!_vertex_local_map_last_added) {

    //ds add landmark measurements contained in the frame by scanning its framepoints - porting weights from previous optimization
    for (FramePoint* framepoint: frame_->points()) {

      //ds if the framepoint is linked to a landmark
      Landmark* landmark = framepoint->landmark();
      if (landmark) {
        g2o::VertexPointXYZ* vertex_landmark = 0;

        //ds check if the landmark not yet present in the graph
        if (_landmarks_in_pose_graph.find(landmark) == _landmarks_in_pose_graph.end()) {

          //ds allocate a new point vertex and add it to the graph
          vertex_landmark = new g2o::VertexPointXYZ( );
          vertex_landmark->setEstimate(landmark->coordinates().cast<double>());
          vertex_landmark->setId(landmark->identifier()+_parameters->identifier_space);
          _optimizer->addVertex(vertex_landmark);

          //ds bookkeep the landmark
          _landmarks_in_pose_graph.insert(std::make_pair(landmark, vertex_landmark));
        } else {

          //ds retrieve existing vertex using our bookkeeping container
          vertex_landmark = _landmarks_in_pose_graph[landmark];
        }

        //ds add framepoint position as measurement for the landmark - porting weight from previous optimization
        _setPointEdge(_optimizer, vertex_frame_current, vertex_landmark, framepoint->robotCoordinates(), landmark->numberOfUpdates()/framepoint->depthMeters());
      }
    }

    //ds fix the initial vertex - no measurement to add
    vertex_frame_current->setFixed(true);
  } else {

    //ds add landmark measurements contained in the frame by scanning its framepoints
    for (FramePoint* framepoint: frame_->points()) {

      //ds if the framepoint is linked to a landmark
      Landmark* landmark = framepoint->landmark();
      if (landmark) {
        g2o::VertexPointXYZ* vertex_landmark = 0;

        //ds check if the landmark not yet present in the graph
        if (_landmarks_in_pose_graph.find(landmark) == _landmarks_in_pose_graph.end()) {

          //ds allocate a new point vertex and add it to the graph
          vertex_landmark = new g2o::VertexPointXYZ( );
          vertex_landmark->setEstimate(landmark->coordinates().cast<double>());
          vertex_landmark->setId(landmark->identifier()+_parameters->identifier_space);
          _optimizer->addVertex(vertex_landmark);

          //ds bookkeep the landmark
          _landmarks_in_pose_graph.insert(std::make_pair(landmark, vertex_landmark));
        } else {

          //ds retrieve existing vertex using our bookkeeping container
          vertex_landmark = _landmarks_in_pose_graph[landmark];
        }

        //ds add framepoint position as measurement for the landmark
        _setPointEdge(_optimizer, vertex_frame_current, vertex_landmark, framepoint->robotCoordinates(), 1/framepoint->depthMeters());
      }
    }

    //ds we can connect it to the preceeding frame by adding the odometry measurement
    _setPoseEdge(_optimizer,
                vertex_frame_current,
                _vertex_local_map_last_added,
                frame_->previous()->worldToRobot()*frame_->robotToWorld(),
                _parameters->base_information_frame);
  }

  //ds bookkeep the added frame
  _vertex_local_map_last_added = vertex_frame_current;
  _frames_in_pose_graph.insert(std::make_pair(frame_, vertex_frame_current));
  CHRONOMETER_STOP(addition)
}

void GraphOptimizer::optimizePoseGraph(WorldMap* world_map_) {
  CHRONOMETER_START(optimization)

  //ds save current graph to file
//  const std::string file_name = "pose_graph_"+std::to_string(world_map_->currentFrame()->identifier())+".g2o";
//  _optimizer->save(file_name.c_str());

  //ds optimize graph
  _optimizer->initializeOptimization();
  _optimizer->optimize(_parameters->maximum_number_of_iterations);

  //ds directly backpropagate solution to frames and landmarks of local maps
  Count number_of_negligible_updates = 0;
  for (const std::pair<const Identifier, LocalMap*>& local_map_entry: _local_maps_in_graph) {
    LocalMap* local_map                = local_map_entry.second;
    g2o::VertexSE3* local_map_in_graph = dynamic_cast<g2o::VertexSE3*>(_optimizer->vertex(local_map_entry.first));
    assert(local_map && local_map_in_graph);

    const TransformMatrix3D robot_to_world_optimized = local_map_in_graph->estimate().cast<real>();

    //ds check if change is insignificant enough (happens for already optimal poses)
    const real delta = (robot_to_world_optimized.matrix()-local_map->robotToWorld().matrix()).norm();
    if (delta < _parameters->minimum_estimation_delta_for_update_meters) {
      local_map_in_graph->setEstimate(local_map->robotToWorld().cast<double>());
      ++number_of_negligible_updates;
      continue;
    }

    //ds update local map pose with optimized estimate (will automatically update contained frames and landmarks)
    local_map->setRobotToWorld(robot_to_world_optimized, true);

    //ds unlock the vertex for the next optimization
    local_map_in_graph->setFixed(false);
  }
  LOG_INFO(std::cerr << "GraphOptimizer::optimizePoseGraph|negligible pose backpropagations: "
                     << number_of_negligible_updates << "/" << _local_maps_in_graph.size()
                     << " (" << static_cast<real>(number_of_negligible_updates)/_local_maps_in_graph.size() << ")" << std::endl)

  //ds keep map origin locked (by construction the first frame added to the bookkeeping)
  _optimizer->vertex(0)->setFixed(true);

  //ds move current head to the new optimized position
  world_map_->setRobotToWorld(world_map_->currentFrame()->robotToWorld());
  ++_number_of_optimizations;
  CHRONOMETER_STOP(optimization)
}

void GraphOptimizer::optimizeFactorGraph(WorldMap* world_map_) {
  CHRONOMETER_START(optimization)

  //ds optimize graph (uncomment lines below for g2o graph dumping)
//  const std::string file_name = "factor_graph_"+std::to_string(world_map_->currentFrame()->identifier())+".g2o";
//  _optimizer->save(file_name.c_str());
  _optimizer->initializeOptimization();
  _optimizer->optimize(_parameters->maximum_number_of_iterations);

  //ds directly backpropagate solution to frames and landmarks
  for(std::pair<Frame*, g2o::VertexSE3*> frame_in_pose_graph: _frames_in_pose_graph) {
    frame_in_pose_graph.first->setRobotToWorld(frame_in_pose_graph.second->estimate().cast<real>());
  }
  for(std::pair<Landmark*, g2o::VertexPointXYZ*> landmark_in_pose_graph: _landmarks_in_pose_graph) {
    landmark_in_pose_graph.first->setCoordinates(landmark_in_pose_graph.second->estimate().cast<real>());
  }
  world_map_->setRobotToWorld(world_map_->currentFrame()->robotToWorld());
  ++_number_of_optimizations;

  //ds reset graph for next optimization
  _optimizer->clear();
  _vertex_local_map_last_added = 0;
  _frames_in_pose_graph.clear();
  _landmarks_in_pose_graph.clear();
  CHRONOMETER_STOP(optimization)
}

void GraphOptimizer::_setPoseEdge(g2o::OptimizableGraph* optimizer_,
                                  g2o::OptimizableGraph::Vertex* vertex_from_,
                                  g2o::OptimizableGraph::Vertex* vertex_to_,
                                  const g2o::Isometry3& transform_from_to_,
                                  const real& information_factor_) const {
  g2o::EdgeSE3* edge_pose = new g2o::EdgeSE3();
  edge_pose->setVertex(1, vertex_from_);
  edge_pose->setVertex(0, vertex_to_);
  edge_pose->setMeasurement(transform_from_to_);

  //ds information value
  Eigen::Matrix<double, 6, 6> information(information_factor_*Eigen::Matrix<double, 6, 6>::Identity());
  if (_parameters->free_translation_for_poses) {
    information.block<3,3>(0,0) *= _parameters->base_information_frame_factor_for_translation;
  }
  edge_pose->setInformation(information);
  if (_parameters->enable_robust_kernel_for_poses) {edge_pose->setRobustKernel(new g2o::RobustKernelCauchy());}
  optimizer_->addEdge(edge_pose);
}

void GraphOptimizer::_setPointEdge(g2o::OptimizableGraph* optimizer_,
                                   g2o::VertexSE3* vertex_frame_,
                                   g2o::VertexPointXYZ* vertex_landmark_,
                                   const PointCoordinates& framepoint_robot_coordinates,
                                   const real& information_factor_) const {
  g2o::EdgeSE3PointXYZ* landmark_edge = new g2o::EdgeSE3PointXYZ();

  //ds set 3d point measurement
  landmark_edge->setVertex(0, vertex_frame_);
  landmark_edge->setVertex(1, vertex_landmark_);
  landmark_edge->setMeasurement(framepoint_robot_coordinates);
  landmark_edge->setInformation(information_factor_*Eigen::Matrix<double, 3, 3>::Identity());
  landmark_edge->setParameterId(0, G2oParameter::WORLD_OFFSET);
  if (_parameters->enable_robust_kernel_for_landmarks) {landmark_edge->setRobustKernel(new g2o::RobustKernelCauchy());}
  optimizer_->addEdge(landmark_edge);
}
}
