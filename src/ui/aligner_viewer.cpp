#include "aligner_viewer.h"
#include "srrg_gl_helpers/opengl_primitives.h"
#include <QKeyEvent>
#include "../types/contexts/gt_frame.h"

namespace gslam {

  using namespace srrg_gl_helpers;
  using namespace srrg_core_viewers;

  AlignerViewer::AlignerViewer() {
    setWindowTitle("AlignerViewer");
  }

  void AlignerViewer::setTransformQueryToReferenceGuess(const TransformMatrix3D& transform_query_to_reference_) {
    _points_query_guess.resize(_points_query.size());
    for (Index u = 0; u < _points_query.size(); ++u) {
      _points_query_guess[u].first  = transform_query_to_reference_*_points_query[u].first;
      _points_query_guess[u].second = _points_query[u].second;
    }
  }

  void AlignerViewer::setTransformQueryToReference(const TransformMatrix3D& transform_query_to_reference_) {
    _transform_query_to_reference = transform_query_to_reference_;
    _points_query_transformed.resize(_points_query.size());
    for (Index u = 0; u < _points_query.size(); ++u) {
      _points_query_transformed[u].first  = _transform_query_to_reference*_points_query[u].first;
      _points_query_transformed[u].second = PointColorRGB(0, 1, 0);
    }
  }

  void AlignerViewer::draw(){
    
    glPushMatrix();
    glMultMatrixf(TransformMatrix3D::Identity().cast<float>().data());
    glPushAttrib(GL_POINT_SIZE|GL_COLOR);
    glPushAttrib(GL_LINE_WIDTH|GL_POINT_SIZE);

    //ds draw camera head
    glPushMatrix();
    glMultMatrixf(TransformMatrix3D::Identity().cast<float>().data());
    glColor3f(0, 0, 0);
    drawPyramidWireframe(0.5, 0.5);
    glPopMatrix();

    //ds if verbose draw is enabled
    if (_draw_verbose) {
      glColor3f(0.5, 0.5, 0.5);
      glBegin(GL_LINES);

      //ds if drawing transformed solution
      if (_apply_transform) {
        for (Index u = 0; u < _points_reference.size(); ++u) {
          glVertex3f(_points_reference[u].first.x(), _points_reference[u].first.y(), _points_reference[u].first.z());
          glVertex3f(_points_query_transformed[u].first.x(), _points_query_transformed[u].first.y(), _points_query_transformed[u].first.z());
        }
      } else {
        for (Index u = 0; u < _points_reference.size(); ++u) {
          glVertex3f(_points_reference[u].first.x(), _points_reference[u].first.y(), _points_reference[u].first.z());
          glVertex3f(_points_query_guess[u].first.x(), _points_query_guess[u].first.y(), _points_query_guess[u].first.z());
        }
      }
      glEnd();
    }

    //ds draw available clouds
    if(0 < _points_reference.size()) {
      glPointSize(3);
      glBegin(GL_POINTS);
      for(const PointDrawable& point: _points_reference) {
        glColor3f(point.second.x(), point.second.y(), point.second.z());
        glVertex3f(point.first.x(), point.first.y(), point.first.z());
      }
      glEnd();
    }

    //ds check if transformed is requested
    if (_apply_transform) {
      if(0 < _points_query_transformed.size()) {
        glPointSize(3);
        glBegin(GL_POINTS);
        for(const PointDrawable& point: _points_query_transformed) {
          glColor3f(point.second.x(), point.second.y(), point.second.z());
          glVertex3f(point.first.x(), point.first.y(), point.first.z());
        }
        glEnd();
      }
    } else {
      if(0 < _points_query_guess.size()) {
        glPointSize(3);
        glBegin(GL_POINTS);
        for(const PointDrawable& point: _points_query_guess) {
          glColor3f(point.second.x(), point.second.y(), point.second.z());
          glVertex3f(point.first.x(), point.first.y(), point.first.z());
        }
        glEnd();
      }
    }
    glPopMatrix();
  }

  void AlignerViewer::keyPressEvent(QKeyEvent* event_){
    switch(event_->key()) {
      case Qt::Key_Space: {
        _apply_transform = !_apply_transform;
        break;
      }
      case Qt::Key_V: {
        _draw_verbose = !_draw_verbose;
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

  QString AlignerViewer::helpString( ) const {
      return "See keyboard tab for controls";
  }
}
