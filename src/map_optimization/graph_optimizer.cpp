#include "graph_optimizer.h"

namespace proslam {

  GraphOptimizer::GraphOptimizer(): _optimizer(getOptimizer()) {
    std::cerr << "GraphOptimizer::GraphOptimizer|constructing" << std::endl;
    std::cerr << "GraphOptimizer::GraphOptimizer|constructed" << std::endl;
  }

  GraphOptimizer::~GraphOptimizer(){
    std::cerr << "GraphOptimizer::GraphOptimizer|destroying" << std::endl;
    delete _optimizer;
    std::cerr << "GraphOptimizer::GraphOptimizer|destroyed" << std::endl;
  }

  void GraphOptimizer::optimize(WorldMap* context_) {
    CHRONOMETER_START(overall)
    optimizePoses(context_);
    optimizeLandmarks(context_);
    ++_number_of_optimizations;
    CHRONOMETER_STOP(overall)
  }

  void GraphOptimizer::optimizePoses(WorldMap* context_) {
    _optimizer->clear();
    _optimizer->clearParameters();

    //ds added measurements
    LocalMapPointerVector local_maps_in_graph(0);
    
    //ds add pose measurements to the graph
    LocalMap* current_map = context_->currentLocalMap()->root();

    //ds add first pose
    g2o::VertexSE3* vertex_local_map = new g2o::VertexSE3();
    vertex_local_map->setId(current_map->identifier());
    vertex_local_map->setEstimate(current_map->localMapToWorld().cast<double>());
    _optimizer->addVertex(vertex_local_map);
    vertex_local_map->setFixed(true);
    local_maps_in_graph.push_back(current_map);
    current_map = current_map->next();

    //ds add other poses
    while (current_map) {

      //ds add the pose to g2o
      g2o::VertexSE3* vertex_local_map = new g2o::VertexSE3();
      vertex_local_map->setId(current_map->identifier());
      vertex_local_map->setEstimate(current_map->localMapToWorld().cast<double>());
      _optimizer->addVertex(vertex_local_map);

      //ds if the vertices are directly connected (no track break in between)
      if (current_map->identifier() - current_map->previous()->identifier() == 1) {
        setPoseEdge(_optimizer,
                    current_map->identifier(),
                    current_map->previous()->identifier(),
                    current_map->previous()->worldToLocalMap()*current_map->localMapToWorld());
      }

      //ds update previous
      local_maps_in_graph.push_back(current_map);

      //ds move to the next pose
      current_map = current_map->next();
    }

    //ds pose measurements: check for closures now as all id's are in the graph
    for (const LocalMap* local_map_query: context_->localMaps()) {
      for (const LocalMap::Closure& closure: local_map_query->closures()) {

        //ds fix reference vertex
        _optimizer->vertex(closure.local_map->identifier())->setFixed(true);

        //ds retrieve closure edge
        setPoseEdge(_optimizer,
                    local_map_query->identifier(),
                    closure.local_map->identifier(),
                    closure.relation,
                    closure.omega*10*Eigen::Matrix<double, 6, 6>::Identity());
      }
    }

    //ds optimize poses
    _optimizer->initializeOptimization();
    _optimizer->setVerbose(false);
    _optimizer->optimize(10);

    //ds backpropagate solution to tracking context: local maps
    for (LocalMap* local_map: local_maps_in_graph) {
      g2o::VertexSE3* vertex_local_map = dynamic_cast<g2o::VertexSE3*>(_optimizer->vertex(local_map->identifier()));
      assert(0 != vertex_local_map);
      local_map->update(vertex_local_map->estimate().cast<real>());
    }
    context_->setRobotToWorld(context_->currentLocalMap()->keyframe()->robotToWorld());

    //ds clear g2o memory
    _optimizer->clear();
    _optimizer->clearParameters();
  }

  //ds optimizes all landmarks by computing rudely the average without using g2o
  void GraphOptimizer::optimizeLandmarks(WorldMap* context_){

    //ds update all active landmark positions
    for (const LocalMap* local_map: context_->localMaps()) {
      for (Landmark::State* landmark_state: local_map->landmarks()) {

        //ds buffer current landmark
        Landmark* landmark = landmark_state->landmark;
        assert(landmark);

        //ds update landmark position
        landmark->resetCoordinates(local_map->localMapToWorld()*landmark_state->coordinates_in_local_map);
      }
    }
  }
}
