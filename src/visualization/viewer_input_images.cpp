#include "viewer_input_images.h"

namespace proslam {

ViewerInputImages::ViewerInputImages(const std::string& window_name_): _cv_wait_key_timeout_milliseconds(0),
                                                                       _window_name(window_name_) {
  _active_framepoints.clear();
  LOG_INFO(std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl)
}

ViewerInputImages::~ViewerInputImages() {

  //ds release framepoint copy handle
  _active_framepoints.clear();

  //ds release cv window
  cv::destroyWindow(_window_name);
}

void ViewerInputImages::update(const Frame* frame_) {

  //ds start data transfer (this will lock the calling thread if the GUI is busy) - released when the function is quit
  std::lock_guard<std::mutex> lock(_mutex_data_exchange);

  //ds buffer current framepoints to enable lockless display
  _active_framepoints.clear();
  _active_framepoints.reserve(frame_->activePoints().size());
  _active_framepoints.insert(_active_framepoints.end(), frame_->activePoints().begin(), frame_->activePoints().end());

  //ds set current image
  cv::cvtColor(frame_->intensityImageLeft(), _current_image, CV_GRAY2RGB);
}

void ViewerInputImages::drawFeatures() {

  //ds for all points in the current frame
  for (const FramePoint* point: _active_framepoints) {

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
      }

      //ds draw the point
      cv::circle(_current_image, cv::Point(point->imageCoordinatesLeft().x(), point->imageCoordinatesLeft().y()), 2, color, -1);
    }
  }
}

void ViewerInputImages::drawFeatureTracking() {

  //ds for all points in the current frame
  for (const FramePoint* frame_point: _active_framepoints) {
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

const bool ViewerInputImages::updateGUI() {
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
          switchMode();
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
    LOG_INFO(std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl)
  }
  else {
    _cv_wait_key_timeout_milliseconds = 1;
    LOG_INFO(std::cerr << "switched to benchmark mode (press backspace for switch)" << std::endl)
  }
}
}
