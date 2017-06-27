#include "map_viewer.h"

#include "srrg_gl_helpers/opengl_primitives.h"
#include "types/local_map.h"

namespace proslam {

  using namespace srrg_gl_helpers;
  using namespace srrg_core_viewers;

  MapViewer::MapViewer(const real& object_scale_,
                       const std::string& window_title_): _iterator_permanent_frame_update(0),
                                                          _iterator_permanent_landmark_update(0),
                                                          _option_stepwise_playback(true),
                                                          _object_scale(object_scale_),
                                                          _point_size(2),
                                                          _requested_playback_steps(0),
                                                          _window_title(window_title_),
                                                          _camera_left_to_robot(TransformMatrix3D::Identity()),
                                                          _world_to_robot_origin(TransformMatrix3D::Identity()),
                                                          _robot_viewpoint(TransformMatrix3D::Identity()),
                                                          _world_to_robot(TransformMatrix3D::Identity()) {
    setWindowTitle(_window_title.c_str());
    setFPSIsDisplayed(true);

    //ds set keyboard descriptions
    setKeyDescription(Qt::Key_1, "Toggles map points display");
    setKeyDescription(Qt::Key_2, "Toggles trajectory display at maximum granularity");
    setKeyDescription(Qt::Key_3, "Switches view to robot ego perspective");
    setKeyDescription(Qt::Key_4, "Toggles robot ground truth trajectory (red)");
    setKeyDescription(Qt::Key_5, "Decreases camera size by factor 2");
    setKeyDescription(Qt::Key_6, "Increases camera size by factor 2");
    setKeyDescription(Qt::Key_7, "Decreases point size by factor 2");
    setKeyDescription(Qt::Key_8, "Increases point size by factor 2");
    setKeyDescription(Qt::Key_Space, "Toggles stepwise/benchmark mode");

    _frame_queue_for_local_map.clear();
    _permanent_frames.clear();
    _active_framepoints.clear();
    _tracked_landmarks.clear();
    _temporary_landmarks.clear();
    _permanent_landmarks.clear();
    std::cerr << SUPERDUPER_BAR << std::endl;
    LOG_INFO(std::cerr << "MapViewer::MapViewer|switched to stepwise mode (press [Space] for switch, press [ARROW_UP] for stepping)" << std::endl)
    std::cerr << SUPERDUPER_BAR << std::endl;
    LOG_DEBUG(std::cerr << "MapViewer::MapViewer|constructed" << std::endl)
  }

  MapViewer::~MapViewer() {
    LOG_DEBUG(std::cerr << "MapViewer::~MapViewer|destroying" << std::endl)
    _frame_queue_for_local_map.clear();
    _permanent_frames.clear();
    _active_framepoints.clear();
    _tracked_landmarks.clear();
    _temporary_landmarks.clear();
    _permanent_landmarks.clear();
    LOG_DEBUG(std::cerr << "MapViewer::~MapViewer|destroyed" << std::endl)
  }

  void MapViewer::update(const Frame* current_frame_,
                         const FramePointerVector& frame_queue_for_local_map_,
                         const FramePointerMap& permanent_frames_,
                         const std::vector<Landmark*>& tracked_landmarks_,
                         const LandmarkPointerMap& temporary_landmarks_,
                         const LandmarkPointerMap& permanent_landmarks_,
                         const bool& graph_optimized_) {

    //ds start data transfer (this will lock the calling thread if the GUI is busy) - released when the function is quit
    std::lock_guard<std::mutex> lock(_mutex_data_exchange);

    //ds update robot position
    _world_to_robot = current_frame_->worldToRobot();

    //ds always update frame queue
    _frame_queue_for_local_map.resize(frame_queue_for_local_map_.size());
    for (Index index = 0; index < frame_queue_for_local_map_.size(); ++index) {
      _frame_queue_for_local_map[index].robot_to_world              = frame_queue_for_local_map_[index]->robotToWorld();
      _frame_queue_for_local_map[index].robot_to_world_ground_truth = frame_queue_for_local_map_[index]->robotToWorldGroundTruth();
      _frame_queue_for_local_map[index].is_keyframe                 = frame_queue_for_local_map_[index]->isKeyframe();
      _frame_queue_for_local_map[index].is_closed                   = false;
    }

    //ds buffer framepoints to draw
    _active_framepoints.resize(current_frame_->activePoints().size());
    for (Index index = 0; index < current_frame_->activePoints().size(); ++index) {
      _active_framepoints[index] = current_frame_->activePoints()[index]->worldCoordinates();
    }

    //ds buffer tracked landmarks to draw
    _tracked_landmarks.resize(tracked_landmarks_.size());
    Count number_of_tracked_landmarks_added = 0;
    for (Index index = 0; index < tracked_landmarks_.size(); ++index) {
      if (tracked_landmarks_[index]->areCoordinatesValidated()) {
        _tracked_landmarks[number_of_tracked_landmarks_added].world_coordinates = tracked_landmarks_[index]->coordinates();
        _tracked_landmarks[number_of_tracked_landmarks_added].is_near           = tracked_landmarks_[index]->isNear();
        ++number_of_tracked_landmarks_added;
      }
    }
    _tracked_landmarks.resize(number_of_tracked_landmarks_added);

    //ds buffer temporary landmarks to draw
    _temporary_landmarks.resize(temporary_landmarks_.size());
    Count number_of_temporary_landmarks_added = 0;
    for (LandmarkPointerMap::const_iterator iterator = temporary_landmarks_.begin(); iterator != temporary_landmarks_.end(); iterator++) {
      if (iterator->second->areCoordinatesValidated()) {
        _temporary_landmarks[number_of_temporary_landmarks_added].world_coordinates            = iterator->second->coordinates();
        _temporary_landmarks[number_of_temporary_landmarks_added].is_near                      = iterator->second->isNear();
        _temporary_landmarks[number_of_temporary_landmarks_added].is_in_loop_closure_query     = iterator->second->isInLoopClosureQuery();
        _temporary_landmarks[number_of_temporary_landmarks_added].is_in_loop_closure_reference = iterator->second->isInLoopClosureReference();
        ++number_of_temporary_landmarks_added;
      }
    }
    _temporary_landmarks.resize(number_of_temporary_landmarks_added);

    //ds if the graph was recently optimized
    if (graph_optimized_) {

      //ds update all frames in the map
      for (FramePointerMap::const_iterator iterator = permanent_frames_.begin(); iterator != permanent_frames_.end(); iterator++) {

        //ds check if we got a closed frame (keyframe) at hand
        const bool closed = (iterator->second->isKeyframe() && !iterator->second->localMap()->closures().empty());

        try {
          _permanent_frames.at(iterator->first).robot_to_world = iterator->second->robotToWorld();
          _permanent_frames.at(iterator->first).is_closed      = closed;
        } catch (const std::out_of_range& /*exception*/) {
          _permanent_frames.insert(std::make_pair(iterator->first, DrawableFrame(iterator->second->robotToWorld(),
                                                                                 iterator->second->robotToWorldGroundTruth(),
                                                                                 iterator->second->isKeyframe(),
                                                                                 closed)));
        }
      }

      //ds update all landmarks in the map
      for (LandmarkPointerMap::const_iterator iterator = permanent_landmarks_.begin(); iterator != permanent_landmarks_.end(); iterator++) {
        if (iterator->second->areCoordinatesValidated()) {
          try {
            _permanent_landmarks.at(iterator->first).world_coordinates            = iterator->second->coordinates();
            _permanent_landmarks.at(iterator->first).is_near                      = iterator->second->isNear();
            _permanent_landmarks.at(iterator->first).is_in_loop_closure_query     = iterator->second->isInLoopClosureQuery();
            _permanent_landmarks.at(iterator->first).is_in_loop_closure_reference = iterator->second->isInLoopClosureReference();
          } catch (const std::out_of_range& /*exception*/) {
            _permanent_landmarks.insert(std::make_pair(iterator->first, DrawableLandmark(iterator->second->coordinates(),
                                                                                   iterator->second->isNear(),
                                                                                   iterator->second->isInLoopClosureQuery(),
                                                                                   iterator->second->isInLoopClosureReference())));
          }
        }
      }
    } else {

      //ds check if we have to initialize the iterator
      if (_permanent_frames.empty()) {
        _iterator_permanent_frame_update = permanent_frames_.begin();
      }

      //ds only add new frames
      for (FramePointerMap::const_iterator iterator = _iterator_permanent_frame_update; iterator != permanent_frames_.end(); iterator++) {

        //ds check if we got a closed frame (keyframe) at hand
        const bool closed = (iterator->second->isKeyframe() && !iterator->second->localMap()->closures().empty());
        _permanent_frames.insert(std::make_pair(iterator->first, DrawableFrame(iterator->second->robotToWorld(),
                                                                               iterator->second->robotToWorldGroundTruth(),
                                                                               iterator->second->isKeyframe(),
                                                                               closed)));
      }

      //ds check if we have to initialize the iterator
      if (_permanent_landmarks.empty()) {
        _iterator_permanent_landmark_update = permanent_landmarks_.begin();
      }

      //ds only add new landmarks
      for (LandmarkPointerMap::const_iterator iterator = _iterator_permanent_landmark_update; iterator != permanent_landmarks_.end(); iterator++) {
        if (iterator->second->areCoordinatesValidated()) {
          _permanent_landmarks.insert(std::make_pair(iterator->first, DrawableLandmark(iterator->second->coordinates(),
                                                                                       iterator->second->isNear(),
                                                                                       iterator->second->isInLoopClosureQuery(),
                                                                                       iterator->second->isInLoopClosureReference())));
        }
      }
    }
  }

  void MapViewer::_drawFrame(const DrawableFrame& frame_, const Vector3& color_rgb_) {
    glPushMatrix();
    glMultMatrixf((frame_.robot_to_world*_camera_left_to_robot).cast<float>().data());

    //ds check if the frame is closed and if so highlight it accordingly
    if (frame_.is_closed) {
      glColor3f(0.0, 1.0, 0.0);
    } else {
      glColor3f(color_rgb_.x(), color_rgb_.y(), color_rgb_.z());
    }

    //ds draw camera box
    drawPyramidWireframe(_object_scale, _object_scale);
    glPopMatrix();

    //ds draw pure odometry pose in red
    if (_ground_truth_drawn) {
      const TransformMatrix3D camera_to_world_ground_truth = frame_.robot_to_world_ground_truth*_camera_left_to_robot;
      glPushMatrix();
      glMultMatrixf(camera_to_world_ground_truth.cast<float>().data());
      glColor3f(1, 0, 0);
      drawPyramidWireframe(_object_scale, _object_scale);
      glPopMatrix();
    }
  }

  void MapViewer::_drawLandmarks() {
    glBegin(GL_POINTS);

    //ds highlight the currently seen landmarks
    for (const DrawableLandmark& landmark: _tracked_landmarks) {
      if (landmark.is_near) {
        glColor3f(0, 0, 1);
      } else {
        glColor3f(1, 0, 1);
      }
      glVertex3f(landmark.world_coordinates.x(), landmark.world_coordinates.y(), landmark.world_coordinates.z());
    }

    //ds draw temporary landmarks
    for (const DrawableLandmark& landmark: _temporary_landmarks) {

      //ds specific coloring for closure landmarks
      if (landmark.is_in_loop_closure_query) {
        glColor3f(0, 1, 0);
      } else if (landmark.is_in_loop_closure_reference) {
        glColor3f(0, 0.5, 0);
      } else {
        glColor3f(0.5, 0.5, 0.5);
      }
      glVertex3f(landmark.world_coordinates.x(), landmark.world_coordinates.y(), landmark.world_coordinates.z());
    }

    //ds draw permanent landmarks
    for (const std::pair<Identifier, DrawableLandmark>& landmark: _permanent_landmarks) {

      //ds specific coloring for closure landmarks
      if (landmark.second.is_in_loop_closure_query) {
        glColor3f(0, 1.0, 0);
      } else if (landmark.second.is_in_loop_closure_reference) {
        glColor3f(0, 0.5, 0);
      } else {
        glColor3f(0.5, 0.5, 0.5);
      }
      glVertex3f(landmark.second.world_coordinates.x(), landmark.second.world_coordinates.y(), landmark.second.world_coordinates.z());
    }
    glEnd();
  }

  void MapViewer::_drawFramepoints() {
    glBegin(GL_POINTS);
      for (const PointCoordinates& world_coordinates: _active_framepoints) {
        glColor3f(0.75, 0.75, 0.75);
        glVertex3f(world_coordinates.x(), world_coordinates.y(), world_coordinates.z());
      }
    glEnd();
  }

  void MapViewer::draw(){

    //ds start drawing - during that we cannot exchange the container content
    std::lock_guard<std::mutex> lock(_mutex_data_exchange);

    //ds no specific lighting
    glDisable(GL_LIGHTING);

    //ds set viewpoint
    TransformMatrix3D world_to_robot(_world_to_robot_origin);
    if(_follow_robot) {
      world_to_robot = _robot_viewpoint*_world_to_robot;
    }
    glPushMatrix();
    glMultMatrixf(world_to_robot.cast<float>().data());
    glPointSize(_point_size);
    glLineWidth(_object_scale);

    //ds draw the local map generating head
    for (const DrawableFrame& frame_for_local_map: _frame_queue_for_local_map) {
      _drawFrame(frame_for_local_map, Vector3(0, 0, 1));
    }

    //ds for all frames in the map - obtain the current root
    for (const std::pair<Identifier, DrawableFrame>& frame: _permanent_frames) {

      //ds check if we have a keyframe and drawing is enabled - change to most visible color based on Qt version
      if (frame.second.is_keyframe && _local_maps_drawn) {
#if QT_VERSION >= 0x050000
      _drawFrame(frame.second, Vector3(0, 0, 1));
#else
      _drawFrame(frame.second, Vector3(0.5, 0.5, 1));
#endif
      } else if (_frames_drawn) {
#if QT_VERSION >= 0x050000
      _drawFrame(frame.second, Vector3(0, 0, 1));
#else
      _drawFrame(frame.second, Vector3(0.75, 0.75, 1));
#endif
      }
    }

    //ds if desired, draw landmarks into map
    if (_landmarks_drawn) {
      _drawLandmarks();
      _drawFramepoints();
    }
    glPopMatrix();
  }

  void MapViewer::keyPressEvent(QKeyEvent* event_){
    switch( event_->key( ) ) {
      case Qt::Key_1: {
        if(_landmarks_drawn) {
          _landmarks_drawn = false;
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|landmarks drawn - DISABLED" << std::endl)
        }
        else {
          _landmarks_drawn = true;
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|landmarks drawn - ENABLED" << std::endl)
        }
        break;
      }
      case Qt::Key_2: {
        if(_frames_drawn) {
          _frames_drawn = false;
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|all frames drawn - DISABLED" << std::endl)
        }
        else {
          _frames_drawn = true;
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|all frames drawn - ENABLED" << std::endl)
        }
        break;
      }
      case Qt::Key_3: {
        if(_follow_robot) {
          _follow_robot = false;
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|follow robot - DISABLED" << std::endl)
        }
        else {
          _follow_robot = true;
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|follow robot - ENABLED" << std::endl)
        }
        break;
      }
      case Qt::Key_4: {
        if(_ground_truth_drawn) {
          _ground_truth_drawn = false;
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|draw ground truth - DISABLED" << std::endl)
        }
        else {
          _ground_truth_drawn = true;
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|draw ground truth - ENABLED" << std::endl)
        }
        break;
      }
      case Qt::Key_5: {
        _object_scale /= 2;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|decreasing object scale to: " << _object_scale << std::endl)
        break;
      }
      case Qt::Key_6: {
        _object_scale *= 2;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|increasing object scale to: " << _object_scale << std::endl)
        break;
      }
      case Qt::Key_7: {
        _point_size /= 2;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|decreasing point size to: " << _point_size << std::endl)

        break;
      }
      case Qt::Key_8: {
        _point_size *= 2;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|increasing point size to: " << _point_size << std::endl)
        break;
      }
      case Qt::Key_Space: {
        _option_stepwise_playback = !_option_stepwise_playback;

        //ds if we switched back to benchmark - reset the steps
        if (!_option_stepwise_playback) {
          _requested_playback_steps = 0;
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|switched to benchmark mode" << std::endl)
        } else {
          LOG_INFO(std::cerr << "MapViewer::keyPressEvent|switched to stepwise mode, press [ARROW_UP] to step" << std::endl)
        }
        break;
      }
      case Qt::Key_Up: {
        if (_option_stepwise_playback) {
          ++_requested_playback_steps;
        }
        break;
      }
      case Qt::Key_Down: {
        break;
      }
      case Qt::Key_Left: {
        break;
      }
      case Qt::Key_Right: {
        break;
      }
      default: {
        SimpleViewer::keyPressEvent(event_);
        break;
      }
    }
    draw();
    updateGL();
  }

  QString MapViewer::helpString( ) const {
      return "See 'Keyboard' tab for controls";
  }
}
