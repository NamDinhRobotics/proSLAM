#include "gt_mapper.h"

namespace gslam {

  Mapper::Mapper(): _optimizer(Optimizer::getOptimizer()) {
    LOG_INFO("Mapper::Mapper", "constructed");
  }

  Mapper::~Mapper(){
    LOG_INFO("Mapper::Mapper", "destroying");
    LOG_INFO("Mapper::Mapper", "destroyed");
  }

  void Mapper::optimize(TrackingContext* context_) {
    CHRONOMETER_START(overall)
    optimizePoses(context_);
    optimizeLandmarks(context_);
    CHRONOMETER_STOP(overall)
  }

  // optimizes all landmarks by computing rudely the average. no g2o
  void Mapper::optimizeLandmarks(TrackingContext* context_){

    //ds reset all active landmark positions (redundant calls)
    for (const KeyFrame* keyframe: context_->keyframes()) {
      for (LandmarkItem* item: keyframe->items()) {
        item->landmark()->resetCoordinates();
      }
    }

    //ds update all active landmark positions
    for (const KeyFrame* keyframe: context_->keyframes()) {
      for (LandmarkItem* item: keyframe->items()) {

        //ds buffer current landmark
        Landmark* landmark = item->landmark();
        assert(landmark);

        //ds compute new coordinates according to keyframe position
        const PointCoordinates& coordinates_in_world = keyframe->robotToWorld()*item->spatials();
        const Matrix3& information(Matrix3::Identity());
        landmark->updateCoordinates(coordinates_in_world, information);
        landmark->setIsOptimized(true);
        landmark->setIsClosed(true);
      }
    }
  }

  void Mapper::optimizePoses(TrackingContext* context_) {

    //ds compute string for file naming
    //const std::string identifier                = std::to_string(_id_last_optimization)+ "-" +std::to_string(context_->keyframes().back()->index());
    //const std::string file_pose_raw             = "g2o/graph_"+ identifier +"_pose_raw.g2o";
    //const std::string file_pose_optimized       = "g2o/graph_"+ identifier +"_pose_optimized.g2o";
    //const std::string file_full_raw             = "g2o/graph_"+ identifier +"_full_raw.g2o";
    //const std::string file_full_ptimized        = "g2o/graph_"+ identifier +"_full_optimized.g2o";
    //const std::string file_full_optimized_clean = "g2o/graph_"+ identifier +"_full_optimized_clean.g2o";
    _id_last_optimization = context_->keyframes().back()->index();
    _optimizer->clear();
    _optimizer->clearParameters();

    //ds add the camera as parameter
    g2o::ParameterCamera* parameter_camera = new g2o::ParameterCamera();
    parameter_camera->setId(0);
    parameter_camera->setOffset(context_->currentKeyframe()->camera()->cameraToRobot().cast<double>());
    parameter_camera->setKcam(static_cast<double>(context_->currentKeyframe()->camera()->cameraMatrix()(0,0)),
                              static_cast<double>(context_->currentKeyframe()->camera()->cameraMatrix()(1,1)),
                              static_cast<double>(context_->currentKeyframe()->camera()->cameraMatrix()(0,2)),
                              static_cast<double>(context_->currentKeyframe()->camera()->cameraMatrix()(1,2)));
    _optimizer->addParameter(parameter_camera);

    g2o::ParameterSE3Offset* camera_offset = new g2o::ParameterSE3Offset();
    camera_offset->setId(1);
    camera_offset->setOffset(Eigen::Isometry3d::Identity());
    _optimizer->addParameter(camera_offset);

    //ds vertex linking
    g2o::VertexSE3* vertex_keyframe_previous = 0;
    TransformMatrix3D world_to_frame_previous(TransformMatrix3D::Identity());

    //ds added measurements
    KeyFramePtrVector keyframes_in_graph(0);
    
    //ds pose measurements
    for (KeyFrame* keyframe: context_->keyframes()) {

      //ds add the pose to g2o
      g2o::VertexSE3* vertex_keyframe = new g2o::VertexSE3;
      vertex_keyframe->setId(keyframe->index());
      vertex_keyframe->setEstimate(keyframe->robotToWorld().cast<double>());
      _optimizer->addVertex(vertex_keyframe);

      //ds if previous pose is not set (first frame)
      if (0 == vertex_keyframe_previous) {
        vertex_keyframe->setFixed(true);
      } else {
        _optimizer->addEdge(Optimizer::getPoseEdge(_optimizer,
                                                   vertex_keyframe->id(),
                                                   vertex_keyframe_previous->id(),
                                                   world_to_frame_previous*keyframe->robotToWorld()));
      }

      //ds update previous
      vertex_keyframe_previous = vertex_keyframe;
      world_to_frame_previous = keyframe->worldToRobot();
      keyframes_in_graph.push_back(keyframe);
    }

    //ds pose measurements: check for closures now as all id's are in the graph
    for (const KeyFrame* keyframe_query: context_->keyframes()) {

      for (const KeyFrame::KeyFrameCorrespondence& keyframe_correspondence: keyframe_query->closures()) {
        const KeyFrame* keyframe_reference = keyframe_correspondence.keyframe;
        const TransformMatrix3D transform_query_to_reference = keyframe_correspondence.relation.transform;

        //ds compute required transform delta
        const TransformMatrix3D transform_query_to_reference_current = keyframe_reference->worldToRobot()*keyframe_query->robotToWorld();
        const Vector3 translation_delta = transform_query_to_reference.translation()-transform_query_to_reference_current.translation();

        //ds check threshold
        if (4.0 < translation_delta.norm()) {
          std::cerr << "Mapper::optimizePoses|WARNING: adding large impact closure: " << keyframe_query->index() << " > " << keyframe_reference->index() 
                    << " translation L1: " << translation_delta.norm() << std::endl;
        }

        //ds retrieve closure edge
        g2o::EdgeSE3* edge_closure = Optimizer::getClosureEdge(_optimizer,
                                                               keyframe_query->index(),
                                                               keyframe_reference->index(),
                                                               transform_query_to_reference);
        _optimizer->addEdge(edge_closure);
      }
    }

    //ds optimize poses
    //_optimizer->save(file_pose_raw.c_str());
    _optimizer->initializeOptimization();
    _optimizer->setVerbose(false);
    _optimizer->optimize(10);
    //_optimizer->save(file_pose_optimized.c_str());

    //ds backpropagate solution to tracking context: keyframes
    for (KeyFrame* keyframe: keyframes_in_graph) {
      g2o::VertexSE3* vertex_keyframe = dynamic_cast<g2o::VertexSE3*>(_optimizer->vertex(keyframe->index()));
      assert(0 != vertex_keyframe);
      keyframe->setRobotToWorld(vertex_keyframe->estimate().cast<gt_real>());
    }
    context_->setRobotToWorldPrevious(context_->currentKeyframe()->robotToWorld());

    //std::cerr << "optimized poses: " << keyframes_in_graph.size() << " with closures: " << number_of_closures_added << "/" << number_of_closures_checked << std::endl;
  }
} //namespace gtracker
