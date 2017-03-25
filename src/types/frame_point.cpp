#include "frame_point.h"

namespace proslam {

  Identifier FramePoint::_instances = 0;

  FramePoint::FramePoint(const cv::KeyPoint& keypoint_left_,
                         const cv::Mat& descriptor_left_,
                         const cv::KeyPoint& keypoint_right_,
                         const cv::Mat& descriptor_right_,
                         Frame* frame_): _index(_instances),
                                         _keypoint_left(keypoint_left_),
                                         _keypoint_right(keypoint_right_),
                                         _descriptor_left(descriptor_left_),
                                         _descriptor_right(descriptor_right_),
                                         _disparity(keypoint_left_.pt.x-keypoint_right_.pt.x),
                                         _image_coordinates_left(PointCoordinates(keypoint_left_.pt.x, keypoint_left_.pt.y, 1)),
                                         _image_coordinates_right(PointCoordinates(keypoint_right_.pt.x, keypoint_right_.pt.y, 1)) {
    assert(_disparity > 0.0);
    ++_instances;
    setFrame(frame_);
  }

  void FramePoint::setPrevious(FramePoint* previous_) {
    _previous = previous_;
    setLandmark(previous_->landmark());
    setAge(previous_->age()+1);
    setRoot(previous_->root());
  }
}
