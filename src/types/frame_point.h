#pragma once
#include "definitions.h"

namespace proslam {
  
//ds forward declarations
class Landmark;
class Frame;

//ds this class encapsulates the triangulation information of a salient point in the image and can be linked to a previous FramePoint instance and a Landmark
class FramePoint {
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//ds object handling: specific instantiation controlled by Frame class (factory)
protected:

  //ds construct a new framepoint, owned by the provided Frame
  FramePoint(const cv::KeyPoint& keypoint_left_,
             const cv::Mat& descriptor_left_,
             const cv::KeyPoint& keypoint_right_,
             const cv::Mat& descriptor_right_,
             Frame* frame_);

  //ds prohibit default construction
  FramePoint() = delete;

//ds getters/setters
public:

  //ds unique identifier for a framepoint (exists once in memory)
  inline const Index identifier() const {return _identifier;}

  //ds FramePoint in the previous image
  inline FramePoint* previous() const {return _previous;}
  void setPrevious(FramePoint* previous_);
  inline FramePoint* next() const {return _next;}

  //ds FramePoint in the image where it was first detected (track start)
  inline const FramePoint* origin() const {return _origin;}
  inline FramePoint* origin() {return _origin;}
  void setOrigin(FramePoint* origin_) {_origin = origin_;}

  //ds Frame to which the point belongs
  inline const Frame* frame() const {return _frame;}
  void setFrame(Frame* frame_) {_frame = frame_;}

  inline Landmark* landmark() {return _landmark;}
  inline const Landmark* landmark() const {return _landmark;}
  void setLandmark(Landmark* landmark_) {_landmark = landmark_;}

  inline const real depthMeters() const {return _depth_meters;}
  void setDepthMeters(const real& depth_meters_) {_depth_meters = depth_meters_;}

  //ds flag telling us if the framepoint is close by the camera (depending on the maximumDepthClose value of a Frame)
  inline void setIsNear(const bool& is_near_) {_is_near = is_near_;}
  inline const bool isNear() const {return _is_near;}

  //ds frame point track length (number of previous elements)
  inline const Count trackLength() const {return _track_length;}
  void setTrackLength(const Count& track_length_) {_track_length = track_length_;}

  inline const PointCoordinates& imageCoordinatesLeft() const {return _image_coordinates_left;}
  inline const PointCoordinates& imageCoordinatesRight() const {return _image_coordinates_right;}

  inline const PointCoordinates cameraCoordinatesLeft() const {return _camera_coordinates_left;}
  void setCameraCoordinatesLeft(const PointCoordinates& coordinates_) {_camera_coordinates_left = coordinates_;}

  inline const PointCoordinates robotCoordinates() const {return _robot_coordinates;}
  void setRobotCoordinates(const PointCoordinates& robot_coordinates_) {_robot_coordinates = robot_coordinates_;}

  inline const PointCoordinates worldCoordinates() const {return _world_coordinates;}
  void setWorldCoordinates(const PointCoordinates& world_coordinates_) {_world_coordinates = world_coordinates_;}

  inline const cv::KeyPoint& keypointLeft() const {return _keypoint_left;}
  inline const cv::KeyPoint& keypointRight() const {return _keypoint_right;}

  inline const cv::Mat& descriptorLeft() const {return _descriptor_left;}
  inline const cv::Mat& descriptorRight() const {return _descriptor_right;}

  //ds visualization only
  inline const PointCoordinates reprojectionCoordinatesLeft() const {return _reprojection_coordinates_left;}
  void setReprojectionCoordinatesLeft(const PointCoordinates& reprojection_coordinates_) {_reprojection_coordinates_left = reprojection_coordinates_; }
  inline const PointCoordinates reprojectionCoordinatesRight() const {return _reprojection_coordinates_right;}
  void setReprojectionCoordinatesRight(const PointCoordinates& reprojection_coordinates_) {_reprojection_coordinates_right = reprojection_coordinates_;}

//ds attributes
protected:

  //ds unique identifier for a framepoint (exists once in memory)
  const Identifier _identifier;

  //ds connections to temporal and ownership elements
  FramePoint* _previous = 0; //ds FramePoint in the previous image
  FramePoint* _next     = 0; //ds FramePoint in the next image (updated as soon as previous is called)
  FramePoint* _origin   = 0; //ds FramePoint in the image where it was first detected (track start)
  Frame* _frame         = 0; //ds Frame to which the point belongs

  //ds triangulation information (set by StereoFramePointGenerator)
  const cv::KeyPoint _keypoint_left;
  const cv::KeyPoint _keypoint_right;
  const cv::Mat _descriptor_left;
  const cv::Mat _descriptor_right;

  //ds spatial properties
  PointCoordinates _image_coordinates_left;
  PointCoordinates _image_coordinates_right;
  PointCoordinates _camera_coordinates_left = PointCoordinates::Zero(); //ds 3D point in left camera coordinate frame
  PointCoordinates _robot_coordinates       = PointCoordinates::Zero(); //ds 3D point in robot coordinate frame
  PointCoordinates _world_coordinates       = PointCoordinates::Zero(); //ds 3D point in world coordinate frame (make sure they are updated!)
  real _depth_meters = -1;

  //ds connected landmark (if any)
  Landmark* _landmark = 0;

  //ds framepoint quality: near or far points
  bool _is_near = false;

  //ds frame point track length (number of previous elements)
  Count _track_length = 0;

  //ds grant access to factory for constructor calls
  friend Frame;

  //ds visualization only
  PointCoordinates _reprojection_coordinates_left  = PointCoordinates::Zero();
  PointCoordinates _reprojection_coordinates_right = PointCoordinates::Zero();

//ds class specific
private:

  //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
  static Count _instances;
};

typedef std::vector<FramePoint*, Eigen::aligned_allocator<FramePoint*>> FramePointPointerVector;
typedef FramePoint*** FramePointMatrix;
}
