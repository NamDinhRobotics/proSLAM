#include "../map_optimization/graph_optimizer.h"

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
        landmark->resetCoordinates(local_map->robotToWorld()*landmark_state->robot_coordinates);
      }
    }
  }

  void GraphOptimizer::optimizePoses(WorldMap* context_) {
    _optimizer->clear();
    _optimizer->clearParameters();

    //ds vertex linking
    g2o::VertexSE3* vertex_local_map_previous = 0;
    TransformMatrix3D world_to_frame_previous(TransformMatrix3D::Identity());

    //ds added measurements
    LocalMapPointerVector local_maps_in_graph(0);
    
    //ds pose measurements
    for (LocalMap* local_map: context_->localMaps()) {

      //ds add the pose to g2o
      g2o::VertexSE3* vertex_local_map = new g2o::VertexSE3;
      vertex_local_map->setId(local_map->identifier());
      vertex_local_map->setEstimate(local_map->robotToWorld().cast<double>());
      _optimizer->addVertex(vertex_local_map);

      //ds if previous pose is not set (first frame)
      if (0 == vertex_local_map_previous) {
        vertex_local_map->setFixed(true);
      } else {
        _optimizer->addEdge(getPoseEdge(_optimizer,
                                        vertex_local_map->id(),
                                        vertex_local_map_previous->id(),
                                        world_to_frame_previous*local_map->robotToWorld()));
      }

      //ds update previous
      vertex_local_map_previous = vertex_local_map;
      world_to_frame_previous = local_map->worldToRobot();
      local_maps_in_graph.push_back(local_map);
    }

    //ds pose measurements: check for closures now as all id's are in the graph
    for (const LocalMap* local_map_query: context_->localMaps()) {

      for (const LocalMap::Closure& closure: local_map_query->closures()) {
        const LocalMap* local_map_reference = closure.local_map;
        const TransformMatrix3D transform_query_to_reference = closure.relation;

        //ds compute required transform delta
        const TransformMatrix3D transform_query_to_reference_current = local_map_reference->worldToRobot()*local_map_query->robotToWorld();
        const Vector3 translation_delta = transform_query_to_reference.translation()-transform_query_to_reference_current.translation();

        //ds informative only
        if (4.0 < translation_delta.norm()) {
          std::printf("Mapper::optimizePoses|WARNING: adding large impact closure: [%06lu] > [%06lu] absolute translation (m): %f\n",
                      local_map_query->identifier(), local_map_reference->identifier(), translation_delta.norm());
        }

        //ds retrieve closure edge
        g2o::EdgeSE3* edge_closure = getClosureEdge(_optimizer,
                                                    local_map_query->identifier(),
                                                    local_map_reference->identifier(),
                                                    transform_query_to_reference);
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
      local_map->setRobotToWorld(vertex_local_map->estimate().cast<real>());
    }
    context_->setRobotToWorld(context_->currentLocalMap()->robotToWorld());
  }
}
