#include "image_viewer.h"

#include "types/landmark.h"

namespace proslam {

ImageViewer::ImageViewer(ImageViewerParameters* parameters_): _parameters(parameters_),
                                                              _current_frame(0) {
  LOG_DEBUG(std::cerr << "ImageViewer::ImageViewer|constructed" << std::endl)
}

ImageViewer::~ImageViewer() {
  LOG_DEBUG(std::cerr << "ImageViewer::~ImageViewer|destroyed" << std::endl)
}

void ImageViewer::configure() {
  //ds nothing to do
}

void ImageViewer::update(const Frame* frame_) {

  //ds start data transfer (this will lock the calling thread if the GUI is busy) - released when the function is quit
  std::lock_guard<std::mutex> lock(_mutex_data_exchange);

  //ds update handle
  _current_frame = frame_;

  //ds set current image
  cv::cvtColor(frame_->intensityImageLeft(), _current_image, CV_GRAY2RGB);

  //ds buffer secondary image (if existing and desired)
  if (!_current_image_secondary.empty()                                         &&
      _parameters->tracker_mode == CommandLineParameters::TrackerMode::RGB_DEPTH) {

    //ds upscale depth image to make it more visible
    _current_image_secondary = frame_->intensityImageRight().clone()*5;
  }
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

    //ds display the image(s)
    cv::imshow(_parameters->window_title.c_str(), _current_image);
    if (!_current_image_secondary.empty()                                         &&
        _parameters->tracker_mode == CommandLineParameters::TrackerMode::RGB_DEPTH) {
      cv::imshow(_parameters->window_title_secondary.c_str(), _current_image_secondary);
    }
    cv::waitKey(1);

//    //ds backup images for saving
//    _image_to_save = _current_image.clone();
//    _image_to_save_secondary = _current_image_secondary.clone();
  }
}

void ImageViewer::saveToDisk() {
//  if (_current_frame) {
//    char buffer_file_name[32];
//    std::snprintf(buffer_file_name, 32, "images/image-rgb-%04lu.jpg", _number_of_saved_images);
//    cv::imwrite(buffer_file_name, _image_to_save);
////    std::snprintf(buffer_file_name, 32, "images/image-depth-%04lu.png", _number_of_saved_images);
////    cv::imwrite(buffer_file_name, _image_to_save_secondary);
//    ++_number_of_saved_images;
//  }
}

void ImageViewer::_drawPoints() {
  if (_current_frame) {

    //ds for all points in the current frame
    for (const FramePoint* point: _current_frame->points()) {

      //ds if the point is linked to a landmark
      if (point->landmark()) {
        cv::Scalar color = CV_COLOR_CODE_BLUE;

        //ds check if the landmark is part of a loop closure
        if (point->landmark()->isInLoopClosureQuery() || point->landmark()->isInLoopClosureReference()) {
          color = CV_COLOR_CODE_DARKGREEN;
        }

        //ds draw reprojection circle
        cv::circle(_current_image, cv::Point(point->reprojectionCoordinatesLeft().x(), point->reprojectionCoordinatesLeft().y()), 4, color);

        //ds draw the point
        const cv::Point2i projection(point->imageCoordinatesLeft().x(), point->imageCoordinatesLeft().y());
        cv::circle(_current_image, projection, 2, color, -1);

        //ds draw track length
        cv::putText(_current_image, std::to_string(point->trackLength()), projection+cv::Point2i(5, 5), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.25, CV_COLOR_CODE_RED);
      }
    }
  }
}

void ImageViewer::_drawTracking() {
  if (_current_frame) {

    //ds for all points in the current frame
    for (const FramePoint* point: _current_frame->points()) {
      const cv::Point current_point(point->imageCoordinatesLeft().x(), point->imageCoordinatesLeft().y());
      if (point->landmark()) {
        const cv::Point previous_point(point->previous()->imageCoordinatesLeft().x(), point->previous()->imageCoordinatesLeft().y());
        cv::line(_current_image, current_point, previous_point, CV_COLOR_CODE_GREEN);
      } else if (point->previous()) {
        const cv::Point previous_point(point->previous()->imageCoordinatesLeft().x(), point->previous()->imageCoordinatesLeft().y());
        cv::circle(_current_image, current_point, 1, CV_COLOR_CODE_GREEN, -1);
        cv::line(_current_image, current_point, previous_point, CV_COLOR_CODE_GREEN);
      }

      //ds draw tracking circle
      cv::circle(_current_image, current_point, _current_frame->projectionTrackingDistancePixels(), CV_COLOR_CODE_GREEN);
    }
  }
}
}
