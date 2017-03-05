#include "viewer_input_images.h"

using namespace std;

namespace proslam {

  TrackerViewer::TrackerViewer(TrackingContext* world_): _world(world_), _cv_wait_key_timeout_milliseconds(0) {
    std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;
  }
  
  void TrackerViewer::initDrawing(){
    if (!_world) {
      return;
    }
    const Frame* current_frame = _world->currentFrame();
    if(! current_frame) {
      return;
    }
    if (current_frame->hasIntensity()) {
      cv::cvtColor(current_frame->intensityImage(), _current_image, CV_GRAY2RGB);
    }
  }

  void TrackerViewer::drawFeatures(){

    //ds check if we have an active frame
    const Frame* current_frame = _world->currentFrame();
    if (! current_frame) {
      return;
    }

    //ds for all points in the current frame
    for (const FramePoint* point: current_frame->points()) {

      //ds if the point is representing a landmark view
      if (point->landmark()) {
        cv::Scalar color = CV_COLOR_CODE_WHITE;

        //ds compute intensity -> old landmarks appear brighter
        const real intensity = std::min(static_cast<real>(1.0), point->age()/static_cast<real>(25));

        //ds check validity
        if (point->landmark()->isValidated()) {

          //ds check landmark kind: by vision or by depth
          if (point->landmark()->isByVision()) {
            color = cv::Scalar(100+intensity*155, 0, 100+intensity*155);
          } else {
            color = cv::Scalar(100+intensity*155, 0, 0);
          }
        } else {
          color = CV_COLOR_CODE_RED;
        }

        //ds check if the landmark is part of a loop closure
        if (point->landmark()->isInLoopClosureQuery() || point->landmark()->isInLoopClosureReference()) {
          color = CV_COLOR_CODE_DARKGREEN;
        }

        //ds draw reprojection circle - if valid
        if (point->reprojectionCoordinates().x() > 0 || point->reprojectionCoordinates().y() > 0) {
          cv::circle(_current_image, cv::Point(point->reprojectionCoordinates().x(), point->reprojectionCoordinates().y()), 4, color);
          if (current_frame->hasIntensityExtra()) {
            cv::circle(_current_image, cv::Point(point->reprojectionCoordinatesExtra().x()+current_frame->camera()->imageCols(), point->reprojectionCoordinatesExtra().y()), 4, color);
          }
        } /*else {
          cv::circle(_current_image, cv::Point(point->imageCoordinates().x(), point->imageCoordinates().y()), 4, CV_COLOR_CODE_RED);
          if (current_frame->hasIntensityExtra()) {
            cv::circle(_current_image, cv::Point(point->imageCoordinatesExtra().x()+current_frame->camera()->imageCols(), point->imageCoordinatesExtra().y()), 4, CV_COLOR_CODE_RED);
          }
        }*/

        //ds draw the point
        cv::circle(_current_image, cv::Point(point->imageCoordinates().x(), point->imageCoordinates().y()), 2, color, -1);
      }
    }
  }

  void TrackerViewer::drawFeatureTracking(){
    const Frame* current_frame = _world->currentFrame();
    if (! current_frame)
      return;
    for (const FramePoint* frame_point: current_frame->points()) {
      if (frame_point->landmark()) {

        //ds compute intensity -> old landmarks appear brighter
        const real intensity = std::min(static_cast<real>(1.0), frame_point->age()/static_cast<real>(25));
        const cv::Point current_point(frame_point->imageCoordinates().x(), frame_point->imageCoordinates().y());
        const cv::Point previous_point(frame_point->previous()->imageCoordinates().x(), frame_point->previous()->imageCoordinates().y());
        cv::line(_current_image, current_point, previous_point, cv::Scalar(0, 100+intensity*155, 0));
      }
    }
  }

  const bool TrackerViewer::updateGUI(){
    if (!_current_image.empty()) {
      cv::imshow(_window_name.c_str(), _current_image);

      int last_key_stroke = cv::waitKey(_cv_wait_key_timeout_milliseconds);
      if(last_key_stroke != -1) {
        switch(last_key_stroke) {

          case KeyStroke::Escape: {
            std::cerr << "termination requested" << std::endl;
            return false;
          }
          case KeyStroke::Backspace: {
            if(_cv_wait_key_timeout_milliseconds > 0) {
              _cv_wait_key_timeout_milliseconds = 0;
              std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;
            }
            else {
              _cv_wait_key_timeout_milliseconds = 1;
              std::cerr << "switched to benchmark mode (press backspace for switch)" << std::endl;
            }
            break;
          }
          case KeyStroke::Num1: {
            _display_depth_image = !_display_depth_image;
            break;
          }
          default:{
            break;
          }
        }
      }
    }

    return true;
  }

  void TrackerViewer::switchToStepwiseMode() {
    _cv_wait_key_timeout_milliseconds = 0;
    std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;
  }
}
