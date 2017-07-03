#include "image_viewer.h"

#include "types/landmark.h"

namespace proslam {

ImageViewer::ImageViewer(const std::string& window_title_): _window_title(window_title_),
                                                            _current_frame(0) {
  LOG_DEBUG(std::cerr << "ImageViewer::ImageViewer|constructed" << std::endl)
}

ImageViewer::~ImageViewer() {
  LOG_DEBUG(std::cerr << "ImageViewer::~ImageViewer|destroying" << std::endl)

  //ds release cv window
  cv::destroyWindow(_window_title);
  LOG_DEBUG(std::cerr << "ImageViewer::~ImageViewer|destroyed" << std::endl)
}

void ImageViewer::update(const Frame* frame_) {

  //ds start data transfer (this will lock the calling thread if the GUI is busy) - released when the function is quit
  std::lock_guard<std::mutex> lock(_mutex_data_exchange);

  //ds update handle
  _current_frame = frame_;

  //ds set current image
  cv::cvtColor(frame_->intensityImageLeft(), _current_image, CV_GRAY2RGB);
}

void ImageViewer::draw() {

  //ds lock data transfer while drawing
  std::lock_guard<std::mutex> lock(_mutex_data_exchange);

  //ds if a valid image is set
  if (!_current_image.empty()) {

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
  if (_current_frame) {

    //ds for all points in the current frame
    for (const FramePoint* point: _current_frame->activePoints()) {

      //ds if the point is linked to a landmark
      if (point->landmark()) {
        cv::Scalar color = CV_COLOR_CODE_WHITE;

        //ds check validity
        if (point->landmark()->areCoordinatesValidated()) {

          //ds compute intensity
          const real color_intensity = std::min(static_cast<real>(1.0), point->trackLength()/static_cast<real>(25));

          //ds check landmark kind: by vision or by depth
          if (point->landmark()->isNear()) {
            color = cv::Scalar(100+color_intensity*155, 0, 0);
          } else {
            color = cv::Scalar(100+color_intensity*155, 0, 100+color_intensity*155);
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
}

void ImageViewer::_drawTracking() {
  if (_current_frame) {

    //ds for all points in the current frame
    for (const FramePoint* point: _current_frame->activePoints()) {
      if (point->landmark()) {
        const cv::Point current_point(point->imageCoordinatesLeft().x(), point->imageCoordinatesLeft().y());
        const cv::Point previous_point(point->previous()->imageCoordinatesLeft().x(), point->previous()->imageCoordinatesLeft().y());
        cv::line(_current_image, current_point, previous_point, CV_COLOR_CODE_GREEN);
      } else if (point->previous()) {
        const cv::Point current_point(point->imageCoordinatesLeft().x(), point->imageCoordinatesLeft().y());
        const cv::Point previous_point(point->previous()->imageCoordinatesLeft().x(), point->previous()->imageCoordinatesLeft().y());
        cv::circle(_current_image, current_point, 1, CV_COLOR_CODE_GREEN, -1);
        cv::line(_current_image, current_point, previous_point, CV_COLOR_CODE_GREEN);
      }
    }
  }
}
}
