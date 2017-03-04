#include "frame_point.h"

namespace gslam {
  using namespace std;

  Identifier FramePoint::_instances = 0;

  FramePoint::FramePoint(const cv::KeyPoint& keypoint_, const cv::Mat& descriptor_, Frame* frame_): _index(_instances),
                                                                                                    _keypoint(keypoint_),
                                                                                                    _descriptor(descriptor_),
                                                                                                    _image_coordinates(PointCoordinates(keypoint_.pt.x, keypoint_.pt.y, 1)) {
    ++_instances;

    //ds set available dynamic properties
    setFrame(frame_);
  }

  FramePoint::FramePoint(const cv::KeyPoint& keypoint_, const cv::Mat& descriptor_, Frame* frame_, FramePoint* previous_point_): _index(_instances),
                                                                                                                                 _keypoint(keypoint_),
                                                                                                                                 _descriptor(descriptor_),
                                                                                                                                 _image_coordinates(PointCoordinates(keypoint_.pt.x, keypoint_.pt.y, 1)) {
    ++_instances;

    //ds set available dynamic properties
    setFrame(frame_);
    setPrevious(previous_point_);
    setLandmark(previous_point_->landmark());
    setAge(previous_point_->age()+1);
    setRoot(previous_point_->root());
    setReprojectionCoordinates(PointCoordinates::Zero());
  }

  FramePoint::FramePoint(const cv::KeyPoint& keypoint_left_,
                         const cv::Mat& descriptor_left_,
                         const cv::KeyPoint& keypoint_right_,
                         const cv::Mat& descriptor_right_,
                         Frame* frame_): _index(_instances),
                                         _keypoint(keypoint_left_),
                                         _keypoint_extra(keypoint_right_),
                                         _descriptor(descriptor_left_),
                                         _descriptor_extra(descriptor_right_),
                                         _disparity(keypoint_left_.pt.x-keypoint_right_.pt.x),
                                         _image_coordinates(PointCoordinates(keypoint_left_.pt.x, keypoint_left_.pt.y, 1)),
                                         _image_coordinates_extra(PointCoordinates(keypoint_right_.pt.x, keypoint_right_.pt.y, 1)),
                                         _is_stereo(true) {
    assert(_disparity > 0.0);
    ++_instances;

    //ds set available dynamic properties
    setFrame(frame_);
  }

  FramePoint::FramePoint(const cv::KeyPoint& keypoint_left_,
                         const cv::Mat& descriptor_left_,
                         const cv::KeyPoint& keypoint_right_,
                         const cv::Mat& descriptor_right_,
                         Frame* frame_,
                         FramePoint* previous_point_): _index(_instances),
                                                       _keypoint(keypoint_left_),
                                                       _keypoint_extra(keypoint_right_),
                                                       _descriptor(descriptor_left_),
                                                       _descriptor_extra(descriptor_right_),
                                                       _disparity(keypoint_left_.pt.x-keypoint_right_.pt.x),
                                                       _image_coordinates(PointCoordinates(keypoint_left_.pt.x, keypoint_left_.pt.y, 1)),
                                                       _image_coordinates_extra(PointCoordinates(keypoint_right_.pt.x, keypoint_right_.pt.y, 1)),
                                                       _is_stereo(true) {
    assert(_disparity > 0.0);
    ++_instances;

    //ds set available dynamic properties
    setFrame(frame_);
    setPrevious(previous_point_);
    setLandmark(previous_point_->landmark());
    setAge(previous_point_->age()+1);
    setRoot(previous_point_->root());
    setReprojectionCoordinates(PointCoordinates::Zero());
  }

  void FramePoint::setDepth(const gt_real& depth_meters_) {
    assert(depth_meters_ > 0.0);
    _depth = depth_meters_;
    _has_depth = true;
  }

  void FramePoint::setDepthByVision(const gt_real& depth_meters_) {
    assert(depth_meters_ > 0.0);
    assert(!_has_depth);
    _depth = depth_meters_;
    _has_depth_by_vision = true;
  }

} //namespace gtracker
