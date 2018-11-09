#include "frame_point.h"
#include "landmark.h"

namespace proslam {

Count FramePoint::_instances = 0;

FramePoint::FramePoint(const cv::KeyPoint& keypoint_left_,
                       const cv::Mat& descriptor_left_,
                       const cv::KeyPoint& keypoint_right_,
                       const cv::Mat& descriptor_right_,
                       Frame* frame_): row(keypoint_left_.pt.y),
                                       col(keypoint_left_.pt.x),
                                       _identifier(_instances),
                                       _keypoint_left(keypoint_left_),
                                       _keypoint_right(keypoint_right_),
                                       _descriptor_left(descriptor_left_),
                                       _descriptor_right(descriptor_right_),
                                       _disparity_pixels(keypoint_left_.pt.x-keypoint_right_.pt.x),
                                       _image_coordinates_left(PointCoordinates(keypoint_left_.pt.x, keypoint_left_.pt.y, 1)),
                                       _image_coordinates_right(PointCoordinates(keypoint_right_.pt.x, keypoint_right_.pt.y, 1)) {
  ++_instances;
  setFrame(frame_);
}

FramePoint::FramePoint(const IntensityFeature* feature_left_,
                       const IntensityFeature* feature_right_,
                       Frame* frame_): FramePoint(feature_left_->keypoint, feature_left_->descriptor, feature_right_->keypoint,  feature_right_->descriptor, frame_) {
  _feature_left  = feature_left_;
  _feature_right = feature_right_;
}

FramePoint::~FramePoint() {
  delete _feature_left;
  delete _feature_right;
}

void FramePoint::setPrevious(FramePoint* previous_) {

  //ds update linked list
  previous_->_next = this;
  _previous = previous_;

  //ds update current
  setLandmark(previous_->landmark());
  setTrackLength(previous_->trackLength()+1);
  setOrigin(previous_->origin());
}
} //namespace proslam
