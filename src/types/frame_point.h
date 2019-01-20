#pragma once
#include "definitions.h"
#include "srrg_hbst/types/binary_tree.hpp"

namespace proslam {
  
//ds forward declarations
class Landmark;
class Frame;

//ds HBST: readability
typedef srrg_hbst::BinaryMatchable<Landmark*, SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS> HBSTMatchable;
typedef srrg_hbst::BinaryNode<HBSTMatchable, real> HBSTNode;
typedef HBSTNode::MatchableVector AppearanceVector;
typedef srrg_hbst::BinaryTree<HBSTNode> HBSTTree;

//! @struct container holding spatial and appearance information (used in findStereoKeypoints)
struct IntensityFeature {

  IntensityFeature(): row(0), col(0), index_in_vector(0) {}

  IntensityFeature(const cv::KeyPoint& keypoint_,
                   const cv::Mat& descriptor_,
                   const size_t& index_in_vector_): keypoint(keypoint_),
                                                    descriptor(descriptor_),
                                                    row(keypoint_.pt.y),
                                                    col(keypoint_.pt.x),
                                                    index_in_vector(index_in_vector_) {}
  cv::KeyPoint keypoint;  //ds geometric: feature location in 2D
  cv::Mat descriptor;     //ds appearance: feature descriptor
  int32_t row;            //ds pixel column coordinate (v)
  int32_t col;            //ds pixel row coordinate (u)
  size_t index_in_vector; //ds inverted index for vector containing this

};

typedef std::vector<IntensityFeature*> IntensityFeaturePointerVector;

//ds this class encapsulates the triangulation information of a salient point in the image and can be linked to a previous FramePoint instance and a Landmark
class FramePoint {
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds prohibit default construction
  FramePoint() = delete;

//ds object handling: specific instantiation controlled by Frame class (factory)
protected:

  //ds frmaepoint generated from a rigid stereo RGB input
  FramePoint(const IntensityFeature* feature_left_,
             const IntensityFeature* feature_right_,
             const real& descriptor_distance_triangulation_,
             Frame* frame_);

  //ds framepoint generated from RGB-D input
  FramePoint(const IntensityFeature* feature_left_,
             Frame* frame_);

  ~FramePoint();

//ds getters/setters
public:

  //ds unique identifier for a framepoint (exists once in memory)
  inline const Index identifier() const {return _identifier;}

  //ds FramePoint in the previous image
  inline FramePoint* previous() const {return _previous;}
  void setPrevious(FramePoint* previous_, const bool& move_origin_ = true);
  inline FramePoint* next() const {return _next;}
  void setNext(FramePoint* next_) {_next = next_;}

  //! @brief removes framepoint from track
  void clear();

  //ds FramePoint in the image where it was first detected (track start)
  inline const FramePoint* origin() const {return _origin;}
  inline FramePoint* origin() {return _origin;}
  void setOrigin(FramePoint* origin_) {_origin = origin_;}

  //ds Frame to which the point belongs
  inline const Frame* frame() const {return _frame;}

  inline Landmark* landmark() {return _landmark;}
  inline const Landmark* landmark() const {return _landmark;}
  void setLandmark(Landmark* landmark_) {_landmark = landmark_;}

  inline const real depthMeters() const {return _camera_coordinates_left.z();}

  //ds frame point track length (number of previous elements)
  inline const Count trackLength() const {return _track_length;}
  void setTrackLength(const Count& track_length_) {_track_length = track_length_;}

  void setDescriptorDistanceTriangulation(const real& descriptor_distance_triangulation) {_descriptor_distance_triangulation = descriptor_distance_triangulation;}
  inline const real& descriptorDistanceTriangulation() const {return _descriptor_distance_triangulation;}

  void setEpipolarOffset(const int32_t& epipolar_offset_) {_epipolar_offset = epipolar_offset_;}
  inline const int32_t& epipolarOffset() const {return _epipolar_offset;}

  inline const PointCoordinates& imageCoordinatesLeft() const {return _image_coordinates_left;}
  inline const PointCoordinates& imageCoordinatesRight() const {return _image_coordinates_right;}

  inline const PointCoordinates cameraCoordinatesLeft() const {return _camera_coordinates_left;}
  void setCameraCoordinatesLeft(const PointCoordinates& coordinates_) {_camera_coordinates_left = coordinates_;}

  inline const PointCoordinates robotCoordinates() const {return _robot_coordinates;}
  void setRobotCoordinates(const PointCoordinates& robot_coordinates_) {_robot_coordinates = robot_coordinates_;}

  inline const PointCoordinates worldCoordinates() const {return _world_coordinates;}
  void setWorldCoordinates(const PointCoordinates& world_coordinates_) {_world_coordinates = world_coordinates_;}

  //! @brief associated landmark coordinates in current camera frame
  inline const PointCoordinates cameraCoordinatesLeftLandmark() const {return _camera_coordinates_left_landmark;}
  void setCameraCoordinatesLeftLandmark(const PointCoordinates& camera_coordinates_) {_camera_coordinates_left_landmark = camera_coordinates_;}

  //ds measured properties
  inline const cv::KeyPoint& keypointLeft() const {return _keypoint_left;}
  inline const cv::KeyPoint& keypointRight() const {return _keypoint_right;}
  inline const cv::Mat& descriptorLeft() const {return _descriptor_left;}
  inline const cv::Mat& descriptorRight() const {return _descriptor_right;}
  inline const real& disparityPixels() const {return _disparity_pixels;}

  //ds reset allocated object counter
  static void reset() {_instances = 0;}

  inline const bool& hasUnreliableDepth() const {return _has_unreliable_depth;}
  void setHasUnreliableDepth(const bool& has_unreliable_depth_) {_has_unreliable_depth = has_unreliable_depth_;}

//ds constant properties
public:

  //! @brief the framepoints position in the image
  const int32_t row;
  const int32_t col;

//ds attributes
protected:

  //ds unique identifier for a framepoint (exists once in memory)
  const Identifier _identifier;

  //ds connections to temporal and ownership elements
  FramePoint* _previous = nullptr; //ds FramePoint in the previous image
  FramePoint* _next     = nullptr; //ds FramePoint in the next image (updated as soon as previous is called)
  FramePoint* _origin   = nullptr; //ds FramePoint in the image where it was first detected (track start)
  Frame* _frame         = nullptr; //ds Frame to which the point belongs

  //ds triangulation information (set by StereoFramePointGenerator)
  const cv::KeyPoint _keypoint_left;
  const cv::KeyPoint _keypoint_right;
  const cv::Mat _descriptor_left;
  const cv::Mat _descriptor_right;
  const real _disparity_pixels;
  real _descriptor_distance_triangulation;
  const ImageCoordinates _image_coordinates_left;
  const ImageCoordinates _image_coordinates_right;

  //! @brief epipolar offset at triangulation (0 for regular, horizontal triangulation)
  int32_t _epipolar_offset = 0;

  //ds point position in various coordinate frames
  PointCoordinates _camera_coordinates_left = PointCoordinates::Zero(); //ds 3D point in left camera coordinate frame
  PointCoordinates _robot_coordinates       = PointCoordinates::Zero(); //ds 3D point in robot coordinate frame
  PointCoordinates _world_coordinates       = PointCoordinates::Zero(); //ds 3D point in world coordinate frame (make sure they are updated!)
  PointCoordinates _camera_coordinates_left_landmark = PointCoordinates::Zero(); //ds associated landmark coordinates in local camera frame

  //! @brief set if point is intended to be used only for orientation estimation (i.e. depth not estimated safely or point at infinity)
  bool _has_unreliable_depth = false;

  //ds connected landmark (if any)
  Landmark* _landmark = nullptr;

  //ds frame point track length (number of previous elements)
  Count _track_length = 0;

  //ds grant access to factory for constructor calls
  friend Frame;

  //ds visualization only
  cv::Point2f _projection_estimate_left;
  cv::Point2f _projection_estimate_right;
  cv::Point2f _projection_estimate_right_corrected;
  cv::Point2f _projection_estimate_left_optimized;

//ds class specific
private:

  //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
  static Count _instances;

//ds visualization only
public:

  inline const cv::Point2f projectionEstimateLeft() const {return _projection_estimate_left;}
  inline const cv::Point2f projectionEstimateRight() const {return _projection_estimate_right;}
  inline const cv::Point2f projectionEstimateRightCorrected() const {return _projection_estimate_right_corrected;}
  inline const cv::Point2f projectionEstimateLeftOptimized() const {return _projection_estimate_left_optimized;}
  void setProjectionEstimateLeft(const cv::Point2f& projection_estimate_) {_projection_estimate_left = projection_estimate_;}
  void setProjectionEstimateRight(const cv::Point2f& projection_estimate_) {_projection_estimate_right = projection_estimate_;}
  void setProjectionEstimateRightCorrected(const cv::Point2f& projection_estimate_) {_projection_estimate_right_corrected = projection_estimate_;}
  void setProjectionEstimateLeftOptimized(const cv::Point2f& projection_estimate_left_optimized_) {_projection_estimate_left_optimized = projection_estimate_left_optimized_;}

};

typedef std::vector<FramePoint*> FramePointPointerVector;
typedef FramePoint*** FramePointMatrix;
}
