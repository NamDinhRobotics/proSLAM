#include "viewer_input_images.h"

using namespace std;

namespace proslam {

  ViewerInputImages::ViewerInputImages(const WorldMap* world_, const std::string& window_name_): _world(world_),
                                                                                                 _tracker(0),
                                                                                                 _cv_wait_key_timeout_milliseconds(0),
                                                                                                 _window_name(window_name_) {
    std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;
  }
  
  void ViewerInputImages::initDrawing(){
    if (!_world) {
      return;
    }
    const Frame* current_frame = _world->currentFrame();
    if(! current_frame) {
      return;
    }
    cv::cvtColor(current_frame->intensityImageLeft(), _current_image, CV_GRAY2RGB);
  }

  void ViewerInputImages::drawFeatures(){

    //ds check if we have an active frame
    const Frame* current_frame = _world->currentFrame();
    if (! current_frame) {
      return;
    }

    //ds for all points in the current frame
    for (const FramePoint* point: current_frame->activePoints()) {

      //ds if the point is linked to a landmark
      if (point->landmark()) {
        cv::Scalar color = CV_COLOR_CODE_WHITE;

        //ds compute intensity -> old landmarks appear brighter
        const real intensity = std::min(static_cast<real>(1.0), point->trackLength()/static_cast<real>(25));

        //ds check validity
        if (point->landmark()->areCoordinatesValidated()) {

          //ds check landmark kind: by vision or by depth
          if (point->landmark()->isNear()) {
            color = cv::Scalar(100+intensity*155, 0, 0);
          } else {
            color = cv::Scalar(100+intensity*155, 0, 100+intensity*155);
          }
        } else {
          color = CV_COLOR_CODE_RED;
        }

        //ds check if the landmark is part of a loop closure
        if (point->landmark()->isInLoopClosureQuery() || point->landmark()->isInLoopClosureReference()) {
          color = CV_COLOR_CODE_DARKGREEN;
        }

        //ds draw reprojection circle - if valid
        if (point->reprojectionCoordinatesLeft().x() > 0 || point->reprojectionCoordinatesLeft().y() > 0) {
          cv::circle(_current_image, cv::Point(point->reprojectionCoordinatesLeft().x(), point->reprojectionCoordinatesLeft().y()), 4, color);
          cv::circle(_current_image, cv::Point(point->reprojectionCoordinatesRight().x()+current_frame->cameraLeft()->imageCols(), point->reprojectionCoordinatesRight().y()), 4, color);
        } /*else {
          cv::circle(_current_image, cv::Point(point->imageCoordinates().x(), point->imageCoordinates().y()), 4, CV_COLOR_CODE_RED);
          cv::circle(_current_image, cv::Point(point->imageCoordinatesExtra().x()+current_frame->camera()->imageCols(), point->imageCoordinatesExtra().y()), 4, CV_COLOR_CODE_RED);
        }*/

        //ds draw the point
        cv::circle(_current_image, cv::Point(point->imageCoordinatesLeft().x(), point->imageCoordinatesLeft().y()), 2, color, -1);
      }
    }
  }

  void ViewerInputImages::drawFeatureTracking(){
    const Frame* current_frame = _world->currentFrame();
    if (! current_frame)
      return;
    for (const FramePoint* frame_point: current_frame->activePoints()) {
      if (frame_point->landmark()) {

        //ds compute intensity -> old landmarks appear brighter
        const real intensity = std::min(static_cast<real>(1.0), frame_point->trackLength()/static_cast<real>(25));
        const cv::Point current_point(frame_point->imageCoordinatesLeft().x(), frame_point->imageCoordinatesLeft().y());
        const cv::Point previous_point(frame_point->previous()->imageCoordinatesLeft().x(), frame_point->previous()->imageCoordinatesLeft().y());
        cv::line(_current_image, current_point, previous_point, cv::Scalar(0, 100+intensity*155, 0));
      } else if (frame_point->previous()) {

        //ds draw point track line
        const cv::Point current_point(frame_point->imageCoordinatesLeft().x(), frame_point->imageCoordinatesLeft().y());
        const cv::Point previous_point(frame_point->previous()->imageCoordinatesLeft().x(), frame_point->previous()->imageCoordinatesLeft().y());
        cv::circle(_current_image, current_point, 1, CV_COLOR_CODE_GREEN, -1);
        cv::line(_current_image, current_point, previous_point, CV_COLOR_CODE_GREEN);
      }
    }
  }

  const bool ViewerInputImages::updateGUI(){
    if (!_current_image.empty()) {

//      //ds debugging - tracker related display
//      if (_tracker) {
//
//        //ds visualize binning grid
//        cv::Mat display_bin_map(_current_image);
//        const Count& bin_size = _tracker->binSize();
//        for (Index row = 0; row < _tracker->numberOfRowsBin(); ++row) {
//          for (Index col = 0; col < _tracker->numberOfColsBin(); ++col) {
//            if ((row+1)*bin_size < _tracker->numberOfRowsImage() &&
//                (col+1)*bin_size < _tracker->numberOfColsImage() ) {
//            cv::rectangle(display_bin_map, cv::Point2i(col*bin_size, row*bin_size), cv::Point2i((col+1)*bin_size, (row+1)*bin_size), cv::Scalar(0, 0, 255));
//            }
//          }
//        }
//        cv::imshow("debug", display_bin_map);
//        cv::waitKey(0);
//      }

      cv::imshow(_window_name.c_str(), _current_image);
      int last_key_stroke = cv::waitKey(_cv_wait_key_timeout_milliseconds);
      if(last_key_stroke != -1) {
        switch(last_key_stroke) {

          case KeyStroke::Escape: {
            std::cerr << "termination requested" << std::endl;
            return false;
          }
          case KeyStroke::Backspace: {
            switchMode();
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

  void ViewerInputImages::switchMode() {
    if(_cv_wait_key_timeout_milliseconds > 0) {
      _cv_wait_key_timeout_milliseconds = 0;
      std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;
    }
    else {
      _cv_wait_key_timeout_milliseconds = 1;
      std::cerr << "switched to benchmark mode (press backspace for switch)" << std::endl;
    }
  }
}
