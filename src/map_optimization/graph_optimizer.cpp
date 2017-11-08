#include "graph_optimizer.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"

namespace proslam {

GraphOptimizer::GraphOptimizer(GraphOptimizerParameters* parameters_): _parameters(parameters_),
                                                                       _optimizer(0),
                                                                       _vertex_frame_last_added(0) {
  LOG_DEBUG(std::cerr << "GraphOptimizer::GraphOptimizer|constructed" << std::endl)
}

void GraphOptimizer::configure() {
  LOG_DEBUG(std::cerr << "GraphOptimizer::GraphOptimizer|configuring" << std::endl)

  //ds allocate an optimizable graph
#ifdef SRRG_PROSLAM_G2O_HAS_NEW_OWNERSHIP_MODEL
  std::unique_ptr<SlamLinearSolver> linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(true);
  std::unique_ptr<SlamBlockSolver> blockSolver = g2o::make_unique<SlamBlockSolver>(std::move(linearSolver));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
#else
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(true);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
#endif

  //ds allocate optimizer
  if (_optimizer) {delete _optimizer;}
  _optimizer = new g2o::SparseOptimizer();
  _optimizer->setAlgorithm(solver);

  //ds clean bookkeeping
  _vertex_frame_last_added = 0;
  _frames_in_pose_graph.clear();
  _landmarks_in_pose_graph.clear();

  //ds clean pose graph
  _optimizer->clear();
  _optimizer->setVerbose(false);

  //ds set world offset
  g2o::ParameterSE3Offset* parameter_world_offset = new g2o::ParameterSE3Offset();
  parameter_world_offset->setId(G2oParameter::WORLD_OFFSET);
  _optimizer->addParameter(parameter_world_offset);
  LOG_DEBUG(std::cerr << "GraphOptimizer::GraphOptimizer|configured" << std::endl)
}

GraphOptimizer::~GraphOptimizer(){
  LOG_DEBUG(std::cerr << "GraphOptimizer::~GraphOptimizer|destroying" << std::endl)
  _frames_in_pose_graph.clear();
  _landmarks_in_pose_graph.clear();
  if (_optimizer) {
    _optimizer->clear();
    _optimizer->clearParameters();
    delete _optimizer;
  }
  LOG_DEBUG(std::cerr << "GraphOptimizer::~GraphOptimizer|destroyed" << std::endl)
}

void GraphOptimizer::writePoseGraphToFile(const WorldMap* world_map_, const std::string& file_name_) const {
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
                  frame.second->previous()->worldToRobot()*frame.second->robotToWorld(),
                  _parameters->base_information_frame,
                  _parameters->free_translation_for_pose_measurements);
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
      for (const LocalMap::Closure& closure: frame.second->localMap()->closures()) {

        //ds retrieve closure edge
        _setPoseEdge(pose_graph,
                     pose_graph->vertex(frame.second->localMap()->keyframe()->identifier()),
                     pose_graph->vertex(closure.local_map->keyframe()->identifier()),
                     closure.relation,
                     _parameters->base_information_frame,
                     _parameters->free_translation_for_pose_measurements,
                     _parameters->enable_robust_kernel_for_loop_closure_measurements);

        //ds merge landmarks - if not already done online
        if (!world_map_->parameters()->merge_landmarks) {
          for (const LandmarkCorrespondence* landmark_correspondence: closure.landmark_correspondences) {
            try {

              //ds retrieve query and reference vertices
              g2o::VertexPointXYZ* vertex_query     = landmarks_in_pose_graph.at(landmark_correspondence->query->landmark);
              g2o::VertexPointXYZ* vertex_reference = landmarks_in_pose_graph.at(landmark_correspondence->reference->landmark);

              //ds if not already merged
              if (vertex_query != vertex_reference) {

                //ds merge query into reference
                pose_graph->mergeVertices(vertex_reference, vertex_query, true);

                //ds update bookkeeping
                landmarks_in_pose_graph.at(landmark_correspondence->query->landmark) = vertex_reference;
              }
            }
            catch (const std::out_of_range& /*exception*/) {}
          }
        }
      }
    }
  }

  //ds save the graph to disk
  pose_graph->save(file_name_.c_str());
  pose_graph->clear();
  landmarks_in_pose_graph.clear();
  LOG_INFO(std::cerr << "GraphOptimizer::writePoseGraphToFile|pose graph saved" << std::endl)
}

void GraphOptimizer::addFrame(Frame* frame_) {
  CHRONOMETER_START(addition)

  //ds get the frames pose to g2o representation
  g2o::VertexSE3* vertex_frame_current = new g2o::VertexSE3();
  vertex_frame_current->setId(frame_->identifier());
  vertex_frame_current->setEstimate(frame_->robotToWorld().cast<double>());
  _optimizer->addVertex(vertex_frame_current);

  //ds if its the first frame to be added (start or recently cleared pose graph)
  if (!_vertex_frame_last_added) {

    //ds fix the initial vertex - no measurement to add
    vertex_frame_current->setFixed(true);
  } else {

    //ds we can connect it to the preceeding frame by adding the odometry measurement
    _setPoseEdge(_optimizer,
                vertex_frame_current,
                _vertex_frame_last_added,
                frame_->previous()->worldToRobot()*frame_->robotToWorld(),
                _parameters->base_information_frame,
                _parameters->free_translation_for_pose_measurements);
  }

  //ds if the frame carries loop closures
  if (frame_->localMap() && frame_->localMap()->closures().size() > 0) {

    //ds for all linked loop closures
    for (const LocalMap::Closure& closure: frame_->localMap()->closures()) {

      //ds reference frame
      Frame* reference_frame = closure.local_map->keyframe();

      //ds check if the reference frame is not contained in the current graph
      if (_frames_in_pose_graph.find(reference_frame) == _frames_in_pose_graph.end()) {

        //ds the vertex was not found - we have to add it to the graph again and fix it
        g2o::VertexSE3* vertex_reference = new g2o::VertexSE3();
        vertex_reference->setId(reference_frame->identifier());
        vertex_reference->setEstimate(reference_frame->robotToWorld().cast<double>());
        vertex_reference->setFixed(true);
        _optimizer->addVertex(vertex_reference);
        _frames_in_pose_graph.insert(std::make_pair(reference_frame, vertex_reference));
      } else {

        //ds fix reference vertex
        _optimizer->vertex(reference_frame->identifier())->setFixed(true);
      }

      //ds retrieve closure edge
      _setPoseEdge(_optimizer,
                   _optimizer->vertex(frame_->localMap()->keyframe()->identifier()),
                   _optimizer->vertex(reference_frame->identifier()),
                   closure.relation,
                   _parameters->base_information_frame,
                   _parameters->free_translation_for_pose_measurements,
                   _parameters->enable_robust_kernel_for_loop_closure_measurements);
    }
  }

  //ds bookkeep the added frame
  _vertex_frame_last_added = vertex_frame_current;
  _frames_in_pose_graph.insert(std::make_pair(frame_, vertex_frame_current));
  CHRONOMETER_STOP(addition)
}

void GraphOptimizer::addFrameWithLandmarks(Frame* frame_) {
  CHRONOMETER_START(addition)

  //ds get the frames pose to g2o representation
  g2o::VertexSE3* vertex_frame_current = new g2o::VertexSE3();
  vertex_frame_current->setId(frame_->identifier());
  vertex_frame_current->setEstimate(frame_->robotToWorld().cast<double>());
  _optimizer->addVertex(vertex_frame_current);

  //ds if its the first frame to be added (start or recently cleared pose graph)
  if (!_vertex_frame_last_added) {

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
                _vertex_frame_last_added,
                frame_->previous()->worldToRobot()*frame_->robotToWorld(),
                _parameters->base_information_frame,
                _parameters->free_translation_for_pose_measurements);
  }

  //ds bookkeep the added frame
  _vertex_frame_last_added = vertex_frame_current;
  _frames_in_pose_graph.insert(std::make_pair(frame_, vertex_frame_current));
  CHRONOMETER_STOP(addition)
}

void GraphOptimizer::optimizeFrames(WorldMap* world_map_) {
  CHRONOMETER_START(optimization)

  //ds optimize existing graph
//  const std::string file_name = "pose_graph_"+std::to_string(world_map_->currentFrame()->identifier())+".g2o";
//  _optimizer->save(file_name.c_str());
  _optimizer->initializeOptimization();
  _optimizer->optimize(_parameters->maximum_number_of_iterations);

  //ds directly backpropagate solution to frames
  for(std::pair<Frame*, g2o::VertexSE3*> frame_in_pose_graph: _frames_in_pose_graph) {
    frame_in_pose_graph.first->setRobotToWorld(frame_in_pose_graph.second->estimate().cast<real>());
  }

  //ds update all active landmark positions based on their last local map presence
  for (const LocalMap* local_map: world_map_->localMaps()) {

    //ds update landmark position rigidly
    for (Landmark::State* landmark_state: local_map->landmarks()) {
      landmark_state->landmark->setCoordinates(local_map->localMapToWorld()*landmark_state->coordinates_in_local_map);
    }
  }
  world_map_->setRobotToWorld(world_map_->currentFrame()->robotToWorld());
  ++_number_of_optimizations;

  //ds reset graph for next optimization
  _optimizer->clear();
  _vertex_frame_last_added = 0;
  _frames_in_pose_graph.clear();
  _landmarks_in_pose_graph.clear();
  CHRONOMETER_STOP(optimization)
}

void GraphOptimizer::optimizeFramesWithLandmarks(WorldMap* world_map_) {
  CHRONOMETER_START(optimization)

  //ds optimize existing graph
//  const std::string file_name = "pose_graph_"+std::to_string(world_map_->currentFrame()->identifier())+".g2o";
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
  _vertex_frame_last_added = 0;
  _frames_in_pose_graph.clear();
  _landmarks_in_pose_graph.clear();
  CHRONOMETER_STOP(optimization)
}

void GraphOptimizer::_setPoseEdge(g2o::OptimizableGraph* optimizer_,
                                  g2o::OptimizableGraph::Vertex* vertex_from_,
                                  g2o::OptimizableGraph::Vertex* vertex_to_,
                                  const TransformMatrix3D& transform_from_to_,
                                  const real& information_factor_,
                                  const bool& free_translation_,
                                  const bool& enable_robust_kernel_) const {
  g2o::EdgeSE3* edge_pose = new g2o::EdgeSE3();
  edge_pose->setVertex(1, vertex_from_);
  edge_pose->setVertex(0, vertex_to_);
  edge_pose->setMeasurement(transform_from_to_.cast<double>());

  //ds information value
  Eigen::Matrix<double, 6, 6> information(information_factor_*Eigen::Matrix<double, 6, 6>::Identity());
  if (free_translation_) {
    information.block<3,3>(0,0) *= _parameters->base_information_frame_factor_for_translation;
  }
  edge_pose->setInformation(information);
  if (enable_robust_kernel_) {edge_pose->setRobustKernel(new g2o::RobustKernelCauchy());}
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
  if (_parameters->enable_robust_kernel_for_landmark_measurements) {landmark_edge->setRobustKernel(new g2o::RobustKernelCauchy());}
  optimizer_->addEdge(landmark_edge);
}
}
