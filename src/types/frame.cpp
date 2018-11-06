#include "frame.h"

#include "world_map.h"

namespace proslam {

Count Frame::_instances = 0;

Frame::Frame(const WorldMap* context_,
             Frame* previous_,
             Frame* next_,
             const TransformMatrix3D& robot_to_world_,
             const real& maximum_depth_near_,
             const double& timestamp_image_left_seconds_): _identifier(_instances),
                                                           _timestamp_image_left_seconds(timestamp_image_left_seconds_),
                                                           _previous(previous_),
                                                           _next(next_),
                                                           _maximum_depth_near(maximum_depth_near_),
                                                           _local_map(0),
                                                           _root(context_->rootFrame()) {
  ++_instances;
  setRobotToWorld(robot_to_world_);
  _created_points.clear();
  _active_points.clear();
  _keypoints_left.clear();
  _keypoints_right.clear();
}

Frame::~Frame() {
  clear();
}

const Count Frame::countPoints(const Count& min_track_length_,
                               const ThreeValued& has_landmark_) const {
  Count count = 0;
  for (const FramePoint* frame_point: points()){
    if(frame_point->trackLength() < min_track_length_) {
      continue;
    }
    if(has_landmark_!= Unknown){
      if(has_landmark_ == True && !frame_point->landmark()) {
        continue;
      }
      if(has_landmark_ == False && frame_point->landmark()) {
        continue;
      }
    }
    ++count;
  }
  return count;
}

void Frame::setRobotToWorld(const TransformMatrix3D& robot_to_world_, const bool update_local_map_) {
  _robot_to_world = robot_to_world_;
  _world_to_robot = _robot_to_world.inverse();
  if (_camera_left) {
    _camera_left_to_world = robot_to_world_*_camera_left->cameraToRobot();
    _world_to_camera_left = _camera_left_to_world.inverse();
  }

  //ds if the frame is a keyframe
  if (_is_keyframe && update_local_map_) {
    assert(_local_map);

    //ds update the local map position
    _local_map->setLocalMapToWorld(_robot_to_world);
  }

  //ds update framepoint world coordinates
  updateActivePoints();
}

FramePoint* Frame::createFramepoint(const cv::KeyPoint& keypoint_left_,
                                    const cv::Mat& descriptor_left_,
                                    const cv::KeyPoint& keypoint_right_,
                                    const cv::Mat& descriptor_right_,
                                    const PointCoordinates& camera_coordinates_left_,
                                    FramePoint* previous_point_) {
  assert(_camera_left != 0);

  //ds allocate a new point connected to the previous one
  FramePoint* frame_point = new FramePoint(keypoint_left_,
                                           descriptor_left_,
                                           keypoint_right_,
                                           descriptor_right_,
                                           this);
  frame_point->setCameraCoordinatesLeft(camera_coordinates_left_);
  frame_point->setRobotCoordinates(_camera_left->cameraToRobot()*camera_coordinates_left_);
  frame_point->setWorldCoordinates(this->robotToWorld()*frame_point->robotCoordinates());

  //ds if the point is not linked (no track)
  if (previous_point_ == 0) {

    //ds this point has no predecessor
    frame_point->setOrigin(frame_point);
  } else {

    //ds connect the framepoints
    frame_point->setPrevious(previous_point_);
  }

  //ds update depth statistics for this frame
  const real depth_meters = camera_coordinates_left_.z();

  //ds update depth based on quality
  frame_point->setDepthMeters(depth_meters);
  if (frame_point->depthMeters() < _maximum_depth_near) {
    frame_point->setIsNear(true);
  }

  //ds bookkeep each generated point for resize immune memory management (TODO remove costly bookkeeping)
  _created_points.push_back(frame_point);
  return frame_point;
}

void Frame::clear() {
  for (const FramePoint* frame_point: _created_points) {
    delete frame_point;
  }
  _created_points.clear();
  _active_points.clear();
  _keypoints_left.clear();
  _keypoints_right.clear();
}

void Frame::updateActivePoints() {
  for (FramePoint* point: _active_points) {
    point->setWorldCoordinates(_robot_to_world*point->robotCoordinates());
  }
}
}
