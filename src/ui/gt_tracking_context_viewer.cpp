#include "gt_tracking_context_viewer.h"
#include "srrg_gl_helpers/opengl_primitives.h"
#include <QKeyEvent>

namespace gslam {

  using namespace srrg_gl_helpers;
  using namespace srrg_core_viewers;

  TrackingContextViewer::TrackingContextViewer(WorldContext* context_, const gt_real& object_scale_): _context(context_),
                                                                                                      _object_scale(object_scale_){

    //ds set keyboard descriptors
    setKeyDescription(Qt::Key_1, "Toggles map points display");
    setKeyDescription(Qt::Key_2, "Toggles trajectory display at maximum granularity");
    setKeyDescription(Qt::Key_3, "Switches view to robot ego perspective");
    setKeyDescription(Qt::Key_4, "Toggles robot ground truth trajectory (red)");
    setKeyDescription(Qt::Key_5, "Decreases camera size by factor 2");
    setKeyDescription(Qt::Key_6, "Increases camera size by factor 2");
    setKeyDescription(Qt::Key_7, "Decreases point size by factor 2");
    setKeyDescription(Qt::Key_8, "Increases point size by factor 2");
  }

  void TrackingContextViewer::drawFrame(const Frame* frame_, const Vector3& color_rgb_) {
    assert(frame_ != 0);
    assert(frame_->camera() != 0);
    const Camera* camera = frame_->camera();
    const TransformMatrix3D camera_to_world_pose = frame_->robotToWorld()*camera->cameraToRobot();
    glPushMatrix();
    glMultMatrixf(camera_to_world_pose.cast<float>().data());

    //ds check if the frame is a closed keyframe and highlight it accordingly
    if (frame_->isKeyFrame() && (static_cast<const KeyFrame*>(frame_))->closures().size() > 0) {
      glColor3f(0.0, 1.0, 0.0);
    } else {
      glColor3f(color_rgb_.x(), color_rgb_.y(), color_rgb_.z());
    }

    drawPyramidWireframe(_object_scale, _object_scale);
    glPopMatrix();

    //ds draw pure odometry pose in red
    if (_ground_truth_drawn) {
      const TransformMatrix3D camera_to_world_pose_odometry = frame_->robotToWorldOdometry()*camera->cameraToRobot();
      glPushMatrix();
      glMultMatrixf(camera_to_world_pose_odometry.cast<float>().data());
      glColor3f(1.0, 0.0, 0.0);
      drawPyramidWireframe(_object_scale, _object_scale);
      glPopMatrix();
    }
  }

  void TrackingContextViewer::drawLandmarks() {
    glPointSize(_point_size);
    glBegin(GL_POINTS);
    for (LandmarkPtrMap::iterator it=_context->currentTrackingContext()->landmarks().begin(); it!=_context->currentTrackingContext()->landmarks().end(); it++) {

      //ds buffer landmark
      const Landmark* landmark = it->second;

      //ds if in context
      if (landmark->isContained() && landmark->isValidated() && (landmark->numberOfUpdates() > 3 || landmark->isActive() || landmark->isClosed())) {
        Vector3 color = Vector3(0, 0, 0);
        if (landmark->isActive() && landmark->isValidated()) {
          if (landmark->isByVision()) {
            color = Vector3(1.0, 0.0, 1.0);
          } else {
            color = Vector3(0.0, 0.0, 1.0);
          }
        } /*else if (!landmark->isValidated()) {
          color = Vector3(1.0, 0.0, 0.0);
        }*/ else if (landmark->isInLoopClosureQuery()) {
          color = Vector3(0.0, 1.0, 0.0);
        } else if (landmark->isInLoopClosureReference()) {
          color = Vector3(0.0, 0.5, 0.0);
        } else if (landmark->isInMapMergeQuery()) {
          color = Vector3(1.0, 0.5, 0.0);
        } else if (landmark->isInMapMergeReference()) {
          color = Vector3(0.7, 0.3, 0.0);
        } else {
//          if (landmark->isByVision()) {
//            color = Vector3(0.5, 0.0, 0.5);
//          } else {
            color = Vector3(0.5, 0.5, 0.5);
//          }
        }
        glColor3f(color.x(), color.y(), color.z());
        glVertex3f(landmark->coordinates().x(), landmark->coordinates().y(), landmark->coordinates().z());
      }
    }
    glEnd();
  }

  void TrackingContextViewer::draw(){
    if (!_context) {
      return;
    }
    if (!_context->currentTrackingContext()) {
      return;
    }
    const Frame* current_frame = _context->currentTrackingContext()->currentFrame();
    if(! current_frame) {
      return;
    }
    
    TransformMatrix3D world_to_robot(_world_to_robot_origin);
    if(_follow_robot) {
      world_to_robot=current_frame->worldToRobot();
    }

    glPushMatrix();
    glMultMatrixf(world_to_robot.cast<float>().data());
    glPushAttrib(GL_POINT_SIZE|GL_COLOR);
    glPointSize(_point_size);
    glPushAttrib(GL_LINE_WIDTH|GL_POINT_SIZE);
    glLineWidth(0.1);

    //ds always draw the keyframe generating head
    for (const Frame* frame_for_keyframe: _context->currentTrackingContext()->frameQueueForKeyframe()) {
      drawFrame(frame_for_keyframe, Vector3(0.0, 0.0, 1.0));
    }

    //ds for all frames in the map
    for (FramePtrMap::const_iterator it=_context->currentTrackingContext()->frames().begin(); it!=_context->currentTrackingContext()->frames().end(); it++){

      //ds check if we have a keyframe and drawing is enabled
      if (it->second->isKeyFrame() && _key_frames_drawn) {
        drawFrame(it->second, Vector3(0.5, 0.5, 0.8));
      } else if (_frames_drawn) {
        drawFrame(it->second, Vector3(0.75, 0.75, 0.9));
      }
    }

    if (_landmarks_drawn) {
      drawLandmarks();
    }
    glPopMatrix();
  }

  void TrackingContextViewer::keyPressEvent(QKeyEvent* event_){
    switch( event_->key( ) ) {
      case Qt::Key_1: {
        if(_landmarks_drawn) {_landmarks_drawn = false;}
        else {_landmarks_drawn = true;}
        break;
      }
      case Qt::Key_2: {
        if(_frames_drawn) {_frames_drawn = false;}
        else {_frames_drawn = true;}
        break;
      }
      case Qt::Key_3: {
        if(_follow_robot) {_follow_robot = false;}
        else {_follow_robot = true;}
        break;
      }
      case Qt::Key_4: {
        if(_ground_truth_drawn) {_ground_truth_drawn = false;}
        else {_ground_truth_drawn = true;}
        break;
      }
      case Qt::Key_5: {
        _object_scale /= 2;
        break;
      }
      case Qt::Key_6: {
        _object_scale *= 2;
        break;
      }
      case Qt::Key_7: {
        _point_size /= 2;
        break;
      }
      case Qt::Key_8: {
        _point_size *= 2;
        break;
      }
      default: {
        SimpleViewer::keyPressEvent(event_);
        break;
      }
    }

    //ds always update ui
    draw();
    updateGL();
  }

  QString TrackingContextViewer::helpString( ) const {
      return "See keyboard tab for controls";
  }
}
