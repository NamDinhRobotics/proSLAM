#include "image_viewer.h"

#include "types/landmark.h"

namespace proslam {

ImageViewer::ImageViewer(const std::string& window_title_): _window_title(window_title_) {
  _active_framepoints.clear();
  LOG_DEBUG(std::cerr << "ImageViewer::ImageViewer|constructed" << std::endl)
}

ImageViewer::~ImageViewer() {
  LOG_DEBUG(std::cerr << "ImageViewer::~ImageViewer|destroying" << std::endl)

  //ds release framepoint copy handle
  _active_framepoints.clear();

  //ds release cv window
  cv::destroyWindow(_window_title);
  LOG_DEBUG(std::cerr << "ImageViewer::~ImageViewer|destroyed" << std::endl)
}

void ImageViewer::update(const Frame* frame_) {

  //ds start data transfer (this will lock the calling thread if the GUI is busy) - released when the function is quit
  std::lock_guard<std::mutex> lock(_mutex_data_exchange);

  //ds buffer current framepoints to enable lockless display
  _active_framepoints.resize(frame_->activePoints().size());
  for (Index index = 0; index < frame_->activePoints().size(); ++index) {
    const FramePoint* framepoint = frame_->activePoints()[index];
    _active_framepoints[index].image_coordinates          = framepoint->imageCoordinatesLeft();
    _active_framepoints[index].reprojection_coordinates   = framepoint->reprojectionCoordinatesLeft();
    _active_framepoints[index].has_landmark               = framepoint->landmark();

    if (framepoint->landmark()) {
      _active_framepoints[index].is_landmark_near = framepoint->landmark()->isNear();
      _active_framepoints[index].is_valid         = framepoint->landmark()->areCoordinatesValidated();
      _active_framepoints[index].is_merged        = (framepoint->landmark()->isInLoopClosureQuery() || framepoint->landmark()->isInLoopClosureReference());
    }
    else {
      _active_framepoints[index].is_valid  = false;
      _active_framepoints[index].is_merged = false;
    }

    _active_framepoints[index].has_previous = framepoint->previous();
    if (framepoint->previous()) {_active_framepoints[index].image_coordinates_previous = framepoint->previous()->imageCoordinatesLeft();}

    _active_framepoints[index].color_intensity = std::min(static_cast<real>(1.0), framepoint->trackLength()/static_cast<real>(25));
  }

  //ds set current image
  cv::cvtColor(frame_->intensityImageLeft(), _current_image, CV_GRAY2RGB);
}

void ImageViewer::draw() {

  //ds if a valid image is set
  if (!_current_image.empty()) {

    //ds lock data transfer while drawing
    std::lock_guard<std::mutex> lock(_mutex_data_exchange);

    //ds draw framepoints
    _drawPoints();

    //ds draw optical flow
    _drawTracking();

    //ds display the image
    cv::imshow(_window_title.c_str(), _current_image);
    cv::waitKey(1);
  }
}

void ImageViewer::_drawPoints() {

  //ds for all points in the current frame
  for (const DrawableFramePoint& point: _active_framepoints) {

    //ds if the point is linked to a landmark
    if (point.has_landmark) {
      cv::Scalar color = CV_COLOR_CODE_WHITE;

      //ds check validity
      if (point.is_valid) {

        //ds check landmark kind: by vision or by depth
        if (point.is_landmark_near) {
          color = cv::Scalar(100+point.color_intensity*155, 0, 0);
        } else {
          color = cv::Scalar(100+point.color_intensity*155, 0, 100+point.color_intensity*155);
        }
      } else {
        color = CV_COLOR_CODE_RED;
      }

      //ds check if the landmark is part of a loop closure
      if (point.is_merged) {
        color = CV_COLOR_CODE_DARKGREEN;
      }

      //ds draw reprojection circle - if valid
      if (point.reprojection_coordinates.x() > 0 || point.reprojection_coordinates.y() > 0) {
        cv::circle(_current_image, cv::Point(point.reprojection_coordinates.x(), point.reprojection_coordinates.y()), 4, color);
      }

      //ds draw the point
      cv::circle(_current_image, cv::Point(point.image_coordinates.x(), point.image_coordinates.y()), 2, color, -1);
    }
  }
}

void ImageViewer::_drawTracking() {

  //ds for all points in the current frame
  for (const DrawableFramePoint& point: _active_framepoints) {
    if (point.has_landmark) {
      const cv::Point current_point(point.image_coordinates.x(), point.image_coordinates.y());
      const cv::Point previous_point(point.image_coordinates_previous.x(), point.image_coordinates_previous.y());
      cv::line(_current_image, current_point, previous_point, cv::Scalar(0, 100+point.color_intensity*155, 0));
    } else if (point.has_previous) {
      const cv::Point current_point(point.image_coordinates.x(), point.image_coordinates.y());
      const cv::Point previous_point(point.image_coordinates_previous.x(), point.image_coordinates_previous.y());
      cv::circle(_current_image, current_point, 1, CV_COLOR_CODE_GREEN, -1);
      cv::line(_current_image, current_point, previous_point, CV_COLOR_CODE_GREEN);
    }
  }
}
}
