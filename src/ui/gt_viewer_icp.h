#pragma once
#include <QGLViewer/qglviewer.h>
#include <QKeyEvent>
#include "core/gt_relocalizer.h"

namespace gslam {
  class ViewerICP: public QGLViewer {
  public:
    ViewerICP(const CorrespondenceCollection* closure): _correspondences(closure->correspondences),
                                                                              _robot_pose_query_to_reference_initial(TransformMatrix3D::Identity()),
                                                                              _robot_pose_query_to_reference_final(closure->transform_frame_query_to_frame_reference) {
      std::string title = "icp_viewer: " + std::to_string(closure->id_query) + " > " + std::to_string(closure->id_reference)
                        + " | avg error: " + std::to_string(closure->icp_inlier_ratio )
                        + " | inliers: " + std::to_string(closure->icp_number_of_inliers)
                        + " | iterations: " + std::to_string(closure->icp_number_of_iterations);
      setWindowTitle(title.c_str());
      showMaximized();
    }

    virtual ~ViewerICP() {}

  protected:
    virtual void draw(){
      glPushAttrib( GL_ENABLE_BIT );
      glDisable( GL_LIGHTING );
      glPointSize( 5.0 );

      //ds draw WORLD coordinate frame
      glColor3f( 1.0, 1.0, 1.0 );
      glLineWidth( 1.0 );
      drawAxis( 1.0 );

      //ds set line width and point size for landmarks
      glLineWidth( 1.0 );
      glPointSize( 5.0 );

      //ds draw reference cloud in blue
      if(0 < _correspondences.size()) {
        for(const Correspondence* correspondence: _correspondences) {

          Vector3 vecREFERENCE;
          Vector3 vecQUERYInitial;

          //ds reference camera point
          if (_draw_in_camera) {
            vecREFERENCE    = correspondence->item_reference->spatials();
            vecQUERYInitial = correspondence->item_query->spatials();
          } else {
            vecREFERENCE    = correspondence->item_reference->landmark()->coordinates();
            vecQUERYInitial = correspondence->item_query->landmark()->coordinates();
          }

          //ds draw the line
          if (_draw_verbose) {
            glColor3f( 0.5, 0.5, 0.5 );
            glBegin( GL_LINES );
            glVertex3f( vecREFERENCE.x( ), vecREFERENCE.y( ), vecREFERENCE.z( ) );
            glVertex3f( vecQUERYInitial.x( ), vecQUERYInitial.y( ), vecQUERYInitial.z( ) );
            glEnd( );
          }

          glBegin( GL_POINTS );
          glColor3f( 0.25, 0.25, 1.0 );
          glVertex3f( vecREFERENCE.x( ), vecREFERENCE.y( ), vecREFERENCE.z( ) );

          if (_draw_verbose || !_draw_optimized_points) {
            glColor3f( 1.0, 0.0, 0.0 );
            glVertex3f( vecQUERYInitial.x( ), vecQUERYInitial.y( ), vecQUERYInitial.z( ) );
          }

          glEnd( );

          if(_draw_optimized_points) {
              const Vector3 vecQUERYFinal( _robot_pose_query_to_reference_final*vecQUERYInitial );

              //ds draw the line
              if (_draw_verbose) {
                glColor3f( 1.0, 0.0, 0.0 );
                glBegin( GL_LINES );
                glVertex3f( vecQUERYInitial.x( ), vecQUERYInitial.y( ), vecQUERYInitial.z( ) );
                glVertex3f( vecQUERYFinal.x( ), vecQUERYFinal.y( ), vecQUERYFinal.z( ) );
                glEnd( );
              }

              glBegin( GL_POINTS );
              glColor3f( 0.0, 0.5, 0.0 );
              glVertex3f( vecQUERYFinal.x( ), vecQUERYFinal.y( ), vecQUERYFinal.z( ) );
              glEnd( );
          }
        }
      }

      glPopAttrib( ); //GL_ENABLE_BIT
    }

    virtual void init(){
      setSceneRadius(25.0);
      setBackgroundColor(QColor(0, 0, 0));
    }

    virtual void keyPressEvent(QKeyEvent* event_){
      switch( event_->key( ) ) {
        case Qt::Key_Space: {
          if( _draw_optimized_points ) {_draw_optimized_points = false;}
          else { _draw_optimized_points = true;}
          draw( );
          updateGL( );
          break;
        }
        case Qt::Key_V: {
          if( _draw_verbose ) {_draw_verbose = false;}
          else { _draw_verbose = true;}
          draw( );
          updateGL( );
          break;
        }
        case Qt::Key_C: {
          if( _draw_in_camera ) {_draw_in_camera = false;}
          else { _draw_in_camera = true;}
          draw( );
          updateGL( );
          break;
        }
        default: {
          QGLViewer::keyPressEvent( event_ );
          break;
        }
      }
    }

    virtual QString helpString() const {return "TODO";}

  private:
    const CorrespondencePointerVector _correspondences;
    TransformMatrix3D _robot_pose_query_to_reference_initial;
    TransformMatrix3D _robot_pose_query_to_reference_final;
    bool _draw_optimized_points = false;
    bool _draw_verbose   = false;
    bool _draw_in_camera = false;

  }; //class CloudViewer
} //namespace gtracker

