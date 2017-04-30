#include "graph_optimizer.h"

namespace proslam {

  GraphOptimizer::GraphOptimizer(): _optimizer(getOptimizer()) {
    std::cerr << "GraphOptimizer::GraphOptimizer|constructing" << std::endl;
    std::cerr << "GraphOptimizer::GraphOptimizer|constructed" << std::endl;
  }

  GraphOptimizer::~GraphOptimizer(){
    std::cerr << "GraphOptimizer::GraphOptimizer|destroying" << std::endl;
    std::cerr << "GraphOptimizer::GraphOptimizer|destroyed" << std::endl;
  }

  void GraphOptimizer::optimize(WorldMap* context_) {
    CHRONOMETER_START(overall)
    optimizePoses(context_);
    optimizeLandmarks(context_);
    ++_number_of_optimizations;
    CHRONOMETER_STOP(overall)
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

  void GraphOptimizer::optimizePoses(WorldMap* context_) {
    _optimizer->clear();
    _optimizer->clearParameters();

    //ds added measurements
    LocalMapPointerVector local_maps_in_graph(0);
    
    //ds add pose measurements to the graph
    LocalMap* current_map = context_->currentLocalMap()->root();

    //ds add first pose
    g2o::VertexSE3* vertex_local_map = new g2o::VertexSE3;
    vertex_local_map->setId(current_map->identifier());
    vertex_local_map->setEstimate(current_map->localMapToWorld().cast<double>());
    _optimizer->addVertex(vertex_local_map);
    vertex_local_map->setFixed(true);
    local_maps_in_graph.push_back(current_map);
    current_map = current_map->next();

    //ds add other poses
    while (current_map) {

      //ds add the pose to g2o
      g2o::VertexSE3* vertex_local_map = new g2o::VertexSE3;
      vertex_local_map->setId(current_map->identifier());
      vertex_local_map->setEstimate(current_map->localMapToWorld().cast<double>());
      _optimizer->addVertex(vertex_local_map);

      //ds if the vertices are directly connected (no track break in between)
      if (current_map->identifier() - current_map->previous()->identifier() == 1) {
        _optimizer->addEdge(getPoseEdge(_optimizer,
                                        current_map->identifier(),
                                        current_map->previous()->identifier(),
                                        current_map->previous()->worldToLocalMap()*current_map->localMapToWorld()));
      }

      //ds update previous
      local_maps_in_graph.push_back(current_map);

      //ds move to the next pose
      current_map = current_map->next();
    }

    //ds pose measurements: check for closures now as all id's are in the graph
    for (const LocalMap* local_map_query: context_->localMaps()) {

      for (const LocalMap::Closure& closure: local_map_query->closures()) {
        const LocalMap* local_map_reference = closure.local_map;
        const TransformMatrix3D transform_query_to_reference = closure.relation;

        //ds compute required transform delta
        const TransformMatrix3D transform_query_to_reference_current = local_map_reference->worldToLocalMap()*local_map_query->localMapToWorld();
        const Vector3 translation_delta = transform_query_to_reference.translation()-transform_query_to_reference_current.translation();

        //ds informative only
        if (4.0 < translation_delta.norm()) {
          std::printf("Mapper::optimizePoses|WARNING: adding large impact closure for local maps: [%06lu] - [%06lu] absolute translation (m): %f\n",
                      local_map_query->identifier(), local_map_reference->identifier(), translation_delta.norm());
        }

        //ds retrieve closure edge
        g2o::EdgeSE3* edge_closure = getClosureEdge(_optimizer,
                                                    local_map_query->identifier(),
                                                    local_map_reference->identifier(),
                                                    transform_query_to_reference,
                                                    closure.omega*10*Eigen::Matrix<double, 6, 6>::Identity());
        _optimizer->addEdge(edge_closure);
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
  }
}
