#pragma once
#include "types/frame.h"

namespace proslam {

//ds this class computes potential framepoints in a stereo image pair by triangulation
class BaseFramePointGenerator {

//ds exported types
public:

  //ds container holding spatial and appearance information (used in findStereoKeypoints)
  struct KeypointWithDescriptor {
    cv::KeyPoint keypoint;
    cv::Mat descriptor;
  };

//ds object handling
PROSLAM_MAKE_PROCESSING_CLASS(BaseFramePointGenerator)

//ds functionality
public:

  //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
  virtual void compute(Frame* frame_) = 0;

  //ds detects keypoints and stores them in a vector (called within compute)
  void detectKeypoints(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_);

  //ds extracts the defined descriptors for the given keypoints (called within compute)
  void extractDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_);

  //ds adjust detector thresholds (for all image streams)
  void adjustDetectorThresholds();

//ds getters/setters
public:

  //ds enable external access to descriptor extractor
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor() const {return _descriptor_extractor;}

  //ds access to framepoints stored in an image-like matrix (pixel wise)
  FramePointMatrix framepointsInImage() {return _framepoints_in_image;}

  //ds other properties
  void setCameraLeft(const Camera* camera_left_) {_camera_left=camera_left_;}
  const Count& numberOfRowsImage() const {return _number_of_rows_image;}
  const Count& numberOfColsImage() const {return _number_of_cols_image;}
  const real maximumDepthNearMeters() const {return _maximum_depth_near_meters;}
  const real maximumDepthFarMeters() const {return _maximum_depth_far_meters;}
  const Count& targetNumberOfKeypoints() const {return _target_number_of_keypoints;}
  void setTargetNumberOfKeyoints(const Count& target_number_of_keypoints_);

  const int32_t matchingDistanceTrackingThreshold() const {return _parameters->matching_distance_tracking_threshold;}
  const Count numberOfAvailablePoints() const {return _number_of_available_points;}
  const Count numberOfDetectedKeypoints() const {return _number_of_detected_keypoints;}

  //ds settings
protected:

  const Camera* _camera_left;

  //ds input properties
  Count _number_of_rows_image;
  Count _number_of_cols_image;

  //ds point detection properties
  Count _target_number_of_keypoints;
  Count _target_number_of_keypoints_per_detector;
  Count _number_of_detected_keypoints;
  Count _number_of_available_points;

  //ds triangulation properties
  real _focal_length_pixels;
  real _principal_point_offset_u_pixels;
  real _principal_point_offset_v_pixels;
  real _maximum_depth_near_meters;
  real _maximum_depth_far_meters;

  //! @brief grid of detectors (equally distributed over the image with size=number_of_detectors_per_dimension*number_of_detectors_per_dimension)
  cv::Ptr<cv::FastFeatureDetector>** _detectors;
  real** _detector_thresholds;

  //! @brief number of detectors
  //! @brief the same for all image streams
  uint32_t _number_of_detectors;

  //! @brief image region for each detector
  //! @brief the same for all image streams
  cv::Rect** _detector_regions;

  //! @brief currently triangulated framepoints stored in a image-like matrix (pixel access) for easy tracking
  FramePointMatrix _framepoints_in_image;

  //ds descriptor extraction
  cv::Ptr<cv::DescriptorExtractor> _descriptor_extractor;

  //ds inner memory buffers (operated on in compute)
  std::vector<KeypointWithDescriptor> _keypoints_with_descriptors_left;

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
}
