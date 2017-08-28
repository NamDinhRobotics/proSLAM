#include "graph_optimizer.h"
#include "g2o/core/robust_kernel_impl.h"

namespace proslam {

GraphOptimizer::GraphOptimizer() {
  LOG_DEBUG(std::cerr << "GraphOptimizer::GraphOptimizer|constructed" << std::endl)

  //ds allocate an optimizable graph
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(true);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmGaussNewton* solverGauss = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
  _optimizer = new g2o::SparseOptimizer();
  _optimizer->setAlgorithm(solverGauss);

  //ds clean bookkeeping
  _vertex_frame_last_added = 0;
  _frames_in_pose_graph.clear();
  _landmarks_in_pose_graph.clear();

  //ds clean pose graph
  _optimizer->clear();

  //ds set world parameter (required for landmark EdgeSE3PointXYZ measurements)
  g2o::ParameterSE3Offset* parameter_world_offset = new g2o::ParameterSE3Offset( );
  parameter_world_offset->setOffset(TransformMatrix3D::Identity().cast<double>());
  parameter_world_offset->setId(G2oParameter::WORLD_OFFSET);
  _optimizer->addParameter(parameter_world_offset);
}

GraphOptimizer::~GraphOptimizer(){
  LOG_DEBUG(std::cerr << "GraphOptimizer::~GraphOptimizer|destroying" << std::endl)
  _frames_in_pose_graph.clear();
  _landmarks_in_pose_graph.clear();
  _optimizer->save("pose_graph.g2o");
  delete _optimizer;
  LOG_DEBUG(std::cerr << "GraphOptimizer::~GraphOptimizer|destroyed" << std::endl)
}

void GraphOptimizer::addFrame(Frame* frame_) {
  CHRONOMETER_START(addition)

  //ds get the frames pose to g2o representation
  g2o::VertexSE3* vertex_frame_current = new g2o::VertexSE3();
  vertex_frame_current->setId(frame_->identifier());
  vertex_frame_current->setEstimate(frame_->robotToWorld().cast<double>());
  _optimizer->addVertex(vertex_frame_current);

  //ds frame information weight depends on related measurements: the more the higher
  Count number_of_measurements = 0;

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
        vertex_landmark->setId(landmark->identifier()+_identifier_space);
        _optimizer->addVertex(vertex_landmark);

        //ds bookkeep the landmark
        _landmarks_in_pose_graph.insert(std::make_pair(landmark, vertex_landmark));
      } else {

        //ds retrieve existing vertex using our bookkeeping container
        vertex_landmark = _landmarks_in_pose_graph[landmark];
      }

      //ds add framepoint position as measurement for the landmark
      _setPointEdge(_optimizer, vertex_frame_current, vertex_landmark, framepoint->robotCoordinates(), 1/framepoint->depthMeters());
      ++number_of_measurements;
    }
  }

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
                base_information_frame+number_of_measurements);
  }

  //ds bookkeep the added frame
  _vertex_frame_last_added = vertex_frame_current;
  _frames_in_pose_graph.insert(std::make_pair(frame_, vertex_frame_current));
  CHRONOMETER_STOP(addition)
}

void GraphOptimizer::optimize() {
  CHRONOMETER_START(optimization)

  //ds optimize existing graph
  _optimizer->initializeOptimization();
  _optimizer->setVerbose(false);
  _optimizer->optimize(10);

  //ds backpropagate solution to frames and landmarks
  for(std::pair<Frame*, g2o::VertexSE3*> frame_in_pose_graph: _frames_in_pose_graph) {
    frame_in_pose_graph.first->setRobotToWorld(frame_in_pose_graph.second->estimate().cast<real>());
  }
  for(std::pair<Landmark*, g2o::VertexPointXYZ*> landmark_in_pose_graph: _landmarks_in_pose_graph) {
    landmark_in_pose_graph.first->resetCoordinates(landmark_in_pose_graph.second->estimate().cast<real>());
  }
  ++_number_of_optimizations;

  //ds reset graph for next optimization
  _optimizer->clear();
  _vertex_frame_last_added = 0;
  _frames_in_pose_graph.clear();
  _landmarks_in_pose_graph.clear();
  CHRONOMETER_STOP(optimization)
}

void GraphOptimizer::_setPoseEdge(g2o::SparseOptimizer* optimizer_,
                                  g2o::OptimizableGraph::Vertex* vertex_from_,
                                  g2o::OptimizableGraph::Vertex* vertex_to_,
                                  const TransformMatrix3D& transform_from_to_,
                                  const real& information_factor_) {
  g2o::EdgeSE3* edge_pose = new g2o::EdgeSE3();
  edge_pose->setVertex(1, vertex_from_);
  edge_pose->setVertex(0, vertex_to_);
  edge_pose->setMeasurement(transform_from_to_.cast<double>());
  edge_pose->setInformation(information_factor_*Eigen::Matrix<double, 6, 6>::Identity());
  optimizer_->addEdge(edge_pose);
}

void GraphOptimizer::_setPointEdge(g2o::SparseOptimizer* optimizer_,
                                   g2o::VertexSE3* vertex_frame_,
                                   g2o::VertexPointXYZ* vertex_landmark_,
                                   const PointCoordinates& framepoint_robot_coordinates,
                                   const real& information_factor_) {
  g2o::EdgeSE3PointXYZ* landmark_edge = new g2o::EdgeSE3PointXYZ();

  //ds set 3d point measurement
  landmark_edge->setVertex(0, vertex_frame_);
  landmark_edge->setVertex(1, vertex_landmark_);
  landmark_edge->setMeasurement(framepoint_robot_coordinates);
  landmark_edge->setInformation(information_factor_*Eigen::Matrix<double, 3, 3>::Identity());
  landmark_edge->setParameterId(0, G2oParameter::WORLD_OFFSET);
  if (enable_robust_kernel_for_landmark_measurements) {landmark_edge->setRobustKernel(new g2o::RobustKernelCauchy());}
  optimizer_->addEdge(landmark_edge);
}
}
