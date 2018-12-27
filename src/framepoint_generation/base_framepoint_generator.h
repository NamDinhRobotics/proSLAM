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

  //ds initializes the framepoint generator (e.g. detects keypoints and computes descriptors)
  virtual void initialize(Frame* frame_, const bool& extract_features_ = true) = 0;

  //ds computes framepoints stored in a image-like matrix (_framepoints_in_image)
  virtual void compute(Frame* frame_) = 0;

  //ds detects keypoints and stores them in a vector (called within initialize)
  void detectKeypoints(const cv::Mat& intensity_image_,
                       std::vector<cv::KeyPoint>& keypoints_,
                       const bool ignore_minimum_detector_threshold_ = false);

  //ds extracts the defined descriptors for the given keypoints (called within compute)
  void computeDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_);

  //! @brief computes tracks between current and previous image points based on appearance
  //! @param[out] previous_points_without_tracks_ lost points
  virtual void track(Frame* frame_,
                     Frame* frame_previous_,
                     const TransformMatrix3D& camera_left_previous_in_current_,
                     FramePointPointerVector& lost_points_,
                     const bool track_by_appearance_ = true) = 0;

  //! @brief adjust detector thresholds (for all image streams)
  void adjustDetectorThresholds();

  //! @brief attempts to recover framepoints in the current image using the more precise pose estimate, retrieved after pose optimization
  //! @brief param[in] current_frame_ the affected frame carrying points to be recovered
  virtual void recoverPoints(Frame* current_frame_, const FramePointPointerVector& lost_points_) const = 0;

  //! @brief brutal midpoint triangulation to obtain a 3D point in the current camera frame
  const PointCoordinates getPointInCamera(const cv::Point2f& image_point_previous_,
                                          const cv::Point2f& image_point_current_,
                                          const TransformMatrix3D& camera_previous_to_current_,
                                          const Matrix3& camera_calibration_matrix_) const;

//ds getters/setters
public:

  //ds enable external access to descriptor extractor
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor() const {return _descriptor_extractor;}

  //ds other properties
  void setCameraLeft(const Camera* camera_left_) {_camera_left = camera_left_;}
  const int32_t& numberOfRowsImage() const {return _number_of_rows_image;}
  const int32_t& numberOfColsImage() const {return _number_of_cols_image;}
  const Count& targetNumberOfKeypoints() const {return _target_number_of_keypoints;}
  void setProjectionTrackingDistancePixels(const int32_t& projection_tracking_distance_pixels_) {_projection_tracking_distance_pixels = projection_tracking_distance_pixels_;}
  void setMaximumDescriptorDistanceTracking(const real& maximum_descriptor_distance_tracking_) {_maximum_descriptor_distance_tracking = maximum_descriptor_distance_tracking_;}

  const int32_t matchingDistanceTrackingThreshold() const {return _parameters->minimum_descriptor_distance_tracking;}
  const Count& numberOfDetectedKeypoints() const {return _number_of_detected_keypoints;}
  const Count& numberOfTrackedLandmarks() const {return _number_of_tracked_landmarks;}

//ds settings
protected:

  const Camera* _camera_left = nullptr;

  //ds image dimensions
  int32_t _number_of_rows_image = 0;
  int32_t _number_of_cols_image = 0;

  //ds point detection properties
  Count _target_number_of_keypoints              = 0;
  Count _target_number_of_keypoints_per_detector = 0;
  Count _number_of_detected_keypoints            = 0;

  //ds quick access
  real _focal_length_pixels             = 0;
  real _principal_point_offset_u_pixels = 0;
  real _principal_point_offset_v_pixels = 0;

  //! @brief grid of detectors (equally distributed over the image with size=number_of_detectors_per_dimension*number_of_detectors_per_dimension)
  cv::Ptr<cv::FastFeatureDetector>** _detectors = nullptr;
  real** _detector_thresholds                   = nullptr;
  real** _detector_thresholds_accumulated       = nullptr;

  //! @brief number of detectors
  //! @brief the same for all image streams
  uint32_t _number_of_detectors = 0;

  //! @brief image region for each detector
  //! @brief the same for all image streams
  cv::Rect** _detector_regions = nullptr;

  //! @brief number of detections since last adjustDetectorThresholds() call
  Count _number_of_detections = 0;

  //ds descriptor extraction
  cv::Ptr<cv::DescriptorExtractor> _descriptor_extractor;

  //ds feature density regularization
  Count _number_of_rows_bin      = 0;
  Count _number_of_cols_bin      = 0;
  FramePointMatrix _bin_map_left = nullptr;

  //! @brief feature matching class (maintains features in a 2D lattice corresponding to the image and a vector)
  IntensityFeatureMatcher _feature_matcher_left;
  std::vector<IntensityFeature> _keypoints_with_descriptors_left;

  //! @brief currently active projection tracking distance (adjusted dynamically at runtime)
  int32_t _projection_tracking_distance_pixels = 0;

  //! @brief current maximum descriptor distance for tracking
  real _maximum_descriptor_distance_tracking   = 0;

  //! @brief status
  Count _number_of_tracked_landmarks = 0;

private:

  //ds informative only
  CREATE_CHRONOMETER(keypoint_detection)
  CREATE_CHRONOMETER(descriptor_extraction)

};

typedef std::shared_ptr<BaseFramePointGenerator> BaseFramePointGeneratorPtr;

} //namespace proslam
