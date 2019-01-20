#include "frame_point.h"
#include "landmark.h"

namespace proslam {

Count FramePoint::_instances = 0;

FramePoint::FramePoint(const IntensityFeature* feature_left_,
                       const IntensityFeature* feature_right_,
                       const real& descriptor_distance_triangulation_,
                       Frame* frame_): row(feature_left_->keypoint.pt.y),
                                       col(feature_left_->keypoint.pt.x),
                                       _identifier(_instances),
                                       _frame(frame_),
                                       _keypoint_left(feature_left_->keypoint),
                                       _keypoint_right(feature_right_->keypoint),
                                       _descriptor_left(feature_left_->descriptor),
                                       _descriptor_right(feature_right_->descriptor),
                                       _disparity_pixels(feature_left_->keypoint.pt.x-feature_right_->keypoint.pt.x),
                                       _descriptor_distance_triangulation(descriptor_distance_triangulation_),
                                       _image_coordinates_left(ImageCoordinates(feature_left_->keypoint.pt.x, feature_left_->keypoint.pt.y, 1)),
                                       _image_coordinates_right(ImageCoordinates(feature_right_->keypoint.pt.x, feature_right_->keypoint.pt.y, 1)) {
  ++_instances;
}

FramePoint::FramePoint(const IntensityFeature* feature_left_,
                       Frame* frame_): row(feature_left_->keypoint.pt.y),
                                       col(feature_left_->keypoint.pt.x),
                                       _identifier(_instances),
                                       _frame(frame_),
                                       _keypoint_left(feature_left_->keypoint),
                                       _descriptor_left(feature_left_->descriptor),
                                       _disparity_pixels(0),
                                       _descriptor_distance_triangulation(0),
                                       _image_coordinates_left(ImageCoordinates(feature_left_->keypoint.pt.x, feature_left_->keypoint.pt.y, 1)) {
  ++_instances;
}

FramePoint::~FramePoint() {
  clear();
}

void FramePoint::setPrevious(FramePoint* previous_, const bool& move_origin_) {

  //ds update linked list
  previous_->_next = this;
  _previous = previous_;
  _has_unreliable_depth = previous_->_has_unreliable_depth;

  //ds update current
  setTrackLength(previous_->trackLength()+1);
  if (move_origin_) {
    setOrigin(previous_->origin());
  }
}

void FramePoint::clear() {

  //ds detach from track
  if (_previous) {
    _previous->_next = nullptr;
    _previous        = nullptr;
  }
  if (_next) {
    _next->_previous = nullptr;
    if (_next->_origin == this) {
      _next->_origin = _next;
    }
  }
  if (_landmark) {
    if (_landmark->origin() == this) {
      assert(_next);
      _landmark->setOrigin(_next);
    }
    _landmark = nullptr;
  }

  //ds reset
  _next = nullptr;
  setTrackLength(0);
  setOrigin(this);
}
} //proslam
