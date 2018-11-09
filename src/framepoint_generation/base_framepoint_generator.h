#pragma once
#include "types/frame.h"
#include "intensity_feature_matcher.h"



namespace proslam {

//! @class this class computes potential framepoints in a stereo image pair by triangulation
class BaseFramePointGenerator {

//ds object handling
PROSLAM_MAKE_PROCESSING_CLASS(BaseFramePointGenerator)

//ds functionality
public:

  //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
  virtual void compute(Frame* frame_, Frame* frame_previous_) = 0;

  //ds detects keypoints and stores them in a vector (called within compute)
  void detectKeypoints(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_);

  //ds extracts the defined descriptors for the given keypoints (called within compute)
  void computeDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_);

  //ds adjust detector thresholds (for all image streams)
  void adjustDetectorThresholds();

//ds getters/setters
public:

  //ds enable external access to descriptor extractor
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor() const {return _descriptor_extractor;}

  //ds access to framepoints stored in an image-like matrix (pixel wise)
  FramePointMatrix framepointsInImage() {return _framepoints_in_image;}

  //ds other properties
  void setCameraLeft(const Camera* camera_left_) {_camera_left = camera_left_;}
  const int32_t& numberOfRowsImage() const {return _number_of_rows_image;}
  const int32_t& numberOfColsImage() const {return _number_of_cols_image;}
  const Count& targetNumberOfKeypoints() const {return _target_number_of_keypoints;}
  void setTargetNumberOfKeyoints(const Count& target_number_of_keypoints_);

  const int32_t matchingDistanceTrackingThreshold() const {return _parameters->matching_distance_tracking_threshold;}
  const Count numberOfAvailablePoints() const {return _number_of_available_points;}
  const Count numberOfDetectedKeypoints() const {return _number_of_detected_keypoints;}

  IntensityFeatureMatcher& featureMatcherLeft() {return _feature_matcher_left;}

//ds settings
protected:

  const Camera* _camera_left = nullptr;

  //ds image dimensions
  int32_t _number_of_rows_image;
  int32_t _number_of_cols_image;

  //ds point detection properties
  Count _target_number_of_keypoints;
  Count _target_number_of_keypoints_per_detector;
  Count _number_of_detected_keypoints;
  Count _number_of_available_points;

  //ds quick access
  real _focal_length_pixels;
  real _principal_point_offset_u_pixels;
  real _principal_point_offset_v_pixels;

  //! @brief grid of detectors (equally distributed over the image with size=number_of_detectors_per_dimension*number_of_detectors_per_dimension)
  cv::Ptr<cv::FastFeatureDetector>** _detectors = nullptr;
  real** _detector_thresholds                   = nullptr;

  //! @brief number of detectors
  //! @brief the same for all image streams
  uint32_t _number_of_detectors;

  //! @brief image region for each detector
  //! @brief the same for all image streams
  cv::Rect** _detector_regions = nullptr;

  //! @brief currently triangulated framepoints stored in a image-like matrix (pixel access) for easy tracking
  FramePointMatrix _framepoints_in_image = nullptr;

  //ds descriptor extraction
  cv::Ptr<cv::DescriptorExtractor> _descriptor_extractor;

  //! @brief feature matching class (maintains features in a 2D lattice corresponding to the image and a vector)
  IntensityFeatureMatcher _feature_matcher_left;
  std::vector<IntensityFeature> _keypoints_with_descriptors_left;

private:

  //ds informative only
  CREATE_CHRONOMETER(keypoint_detection)
  CREATE_CHRONOMETER(descriptor_extraction)

};

//ds custom exception (thrown in getCoordinatesInCamera if no triangulation could be achieved)
class ExceptionTriangulation: public std::exception {
public:

  ExceptionTriangulation(const std::string& what_): _what(what_){}
  ~ExceptionTriangulation(){}

public:

  virtual const char* what() const throw() {return _what.c_str();}

private:

  const std::string _what;

};
} //namespace proslam
