#include "simple_point_viewer.h"
#include "srrg_gl_helpers/opengl_primitives.h"
#include <QKeyEvent>
#include "../types/contexts/gt_frame.h"

namespace gslam {

  using namespace srrg_gl_helpers;
  using namespace srrg_core_viewers;

  SimplePointViewer::SimplePointViewer() {
    setWindowTitle("SimplePointViewer");
  }

  void SimplePointViewer::draw(){
    
    glPushMatrix();
    glMultMatrixf(TransformMatrix3D::Identity().cast<float>().data());
    glPushAttrib(GL_POINT_SIZE|GL_COLOR);
    glPushAttrib(GL_LINE_WIDTH|GL_POINT_SIZE);

    //ds draw camera head
    glPushMatrix();
    glMultMatrixf(TransformMatrix3D::Identity().cast<float>().data());
    glColor3f(1, 0, 0);
    drawPyramidWireframe(0.5, 0.5);
    glPopMatrix();

    //ds draw reference cloud in blue
    if(0 < _points.size()) {
      glPointSize(3);
      glBegin(GL_POINTS);
      for(const PointDrawable& point: _points) {
        glColor3f(point.second.x(), point.second.y(), point.second.z());
        glVertex3f(point.first.x(), point.first.y(), point.first.z());
      }
      glEnd();
    }
    glPopMatrix();
  }

  void SimplePointViewer::keyPressEvent(QKeyEvent* event_){
    switch(event_->key()) {
      default: {
        SimpleViewer::keyPressEvent(event_);
        break;
      }
    }

    //ds always update ui
    draw();
    updateGL();
  }

  QString SimplePointViewer::helpString( ) const {
      return "See keyboard tab for controls";
  }
}
