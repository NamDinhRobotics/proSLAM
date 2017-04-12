#pragma once
#include "types/frame.h"

namespace proslam {

  //ds this class computes potential framepoints in a stereo image pair by triangulation
  class StereoTriangulator {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds exported types
    public:

      //ds container holding spatial and appearance information (used in findStereoKeypoints)
      struct KeypointWithDescriptor {
        cv::KeyPoint keypoint;
        cv::Mat descriptor;
        int32_t row; //ds keypoint v coordinate
        int32_t col; //ds keypoint u coordinate
      };

      //ds readability: a 2d array of pointers to framepoints
      typedef FramePoint*** FramePointMatrix;

    //ds object handling
    public:

      //ds the stereo camera setup must be provided
      StereoTriangulator(const Camera* camera_left_, const Camera* camera_right_);

      //ds cleanup of dynamic structures
      ~StereoTriangulator();

      //ds prohibit default construction
      StereoTriangulator() = delete;

    //ds functionality
    public:

      //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
      void compute(Frame* frame_);

      //ds detects keypoints and stores them in a vector (called within compute)
      void detectKeypoints(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_) const;

      //ds regularizes the detected keypoints using binning (called within compute)
      void binKeypoints(std::vector<cv::KeyPoint>& keypoints_, cv::KeyPoint** bin_map_) const;

      //ds extracts the defined descriptors for the given keypoints (called within compute)
      void extractDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_) const;

      //ds initializes structures for the epipolar stereo keypoint search (called within compute)
      void initialize(const std::vector<cv::KeyPoint>& keypoints_left_,
                      const std::vector<cv::KeyPoint>& keypoints_right_,
                      const cv::Mat& descriptors_left_,
                      const cv::Mat& descriptors_right_);

      //ds computes all potential stereo keypoints (exhaustive in matching distance) and stores them as framepoints (called within compute)
      void findStereoKeypoints(Frame* frame_);

      //ds adjusts the detector and matching thresholds to maintain constant detection (called within compute)
      void calibrateDetectionThresholds();

      //ds computes 3D position of a stereo keypoint pair in the keft camera frame (called within findStereoKeypoints)
      const PointCoordinates getCoordinatesInCameraLeft(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) const;

    //ds getters/setters
    public:

      //ds enable external access to keypoint detection
#if CV_MAJOR_VERSION == 2
      cv::FeatureDetector* keypointDetector() const {return _keypoint_detector;}
#elif CV_MAJOR_VERSION == 3
      cv::Ptr<cv::FastFeatureDetector> keypointDetector() const {return _keypoint_detector;}
#else
      #error OpenCV version not supported
#endif

      //ds enable external access to descriptor computation
#if CV_MAJOR_VERSION == 2
      const cv::DescriptorExtractor* descriptorExtractor() const {return _descriptor_extractor;}
#elif CV_MAJOR_VERSION == 3
      cv::Ptr<cv::DescriptorExtractor> descriptorExtractor() const {return _descriptor_extractor;}
#else
      #error OpenCV version not supported
#endif

      //ds access to framepoints stored in an image-like matrix (pixel wise)
      FramePointMatrix framepointsInImage() {return _framepoints_in_image;}

      //ds clears framepoint image matrix
      void clearFramepointsInImage();

      //ds other properties
      const Count numberOfRowsImage() const {return _number_of_rows_image;}
      const Count numberOfColsImage() const {return _number_of_cols_image;}
      const real maximumDepthNearMeters() const {return _maximum_depth_near_meters;}
      const real maximumDepthFarMeters() const {return _maximum_depth_far_meters;}
      void setTargetNumberOfPoints(const Count& target_number_of_points_) {_target_number_of_points = target_number_of_points_;}
      void setDetectorThreshold(const int32_t& detector_threshold_);
      void setDetectorThresholdMaximum(const int32_t& detector_threshold_maximum_) {_detector_threshold_maximum = detector_threshold_maximum_;}
      void setDetectorThresholdMinimum(const int32_t& detector_threshold_minimum_) {_detector_threshold_minimum = detector_threshold_minimum_;}
      const int32_t matchingDistanceTrackingThreshold() const {return _matching_distance_tracking_threshold;}
      void setMatchingDistanceTrackingThresholdMaximum(const real& matching_distance_tracking_threshold_maximum_) {_matching_distance_tracking_threshold_maximum = matching_distance_tracking_threshold_maximum_;}
      void setMatchingDistanceTrackingThresholdMinimum(const real& matching_distance_tracking_threshold_minimum_) {_matching_distance_tracking_threshold_minimum = matching_distance_tracking_threshold_minimum_;}
      void setMaximumMatchingDistanceTriangulation(const int32_t& maximum_matching_distance_triangulation_) {_maximum_matching_distance_triangulation = maximum_matching_distance_triangulation_;}
      const Count numberOfAvailablePoints() const {return _number_of_available_points;}

    //ds settings
    protected:

      //ds input properties
      const Count _number_of_rows_image;
      const Count _number_of_cols_image;

      //ds point detection properties
      Count _target_number_of_points    = 700;
      Count _number_of_available_points = 0;

      //ds dynamic thresholds for feature detection
      int32_t _detector_threshold         = 10;
      int32_t _detector_threshold_minimum = 0;
      int32_t _detector_threshold_maximum = 50;

      //ds dynamic thresholds for descriptor matching
      int32_t _matching_distance_tracking_threshold         = 50;
      int32_t _matching_distance_tracking_threshold_maximum = 50;
      int32_t _matching_distance_tracking_threshold_minimum = 25;

      //ds feature density regularization
      const Count _bin_size = 4;
      const Count _number_of_bins_u;
      const Count _number_of_bins_v;
      cv::KeyPoint** _bin_map_left;

      //ds triangulation properties
      int32_t _maximum_matching_distance_triangulation = 50;
      const real _focal_length_pixels;
      const real _principal_point_offset_u_pixels;
      const real _principal_point_offset_v_pixels;
      const real _baseline_pixelsmeters;
      const real _baseline_meters;
      const real _baseline_factor          = 50;
      const real _minimum_disparity_pixels = 1;
      const real _maximum_depth_near_meters;
      const real _maximum_depth_far_meters;

      //ds currently triangulated framepoints stored in a image-like matrix (pixel access)
      FramePointMatrix _framepoints_in_image;

      //ds feature detection
#if CV_MAJOR_VERSION == 2
      cv::FeatureDetector* _keypoint_detector;
#elif CV_MAJOR_VERSION == 3
      cv::Ptr<cv::FastFeatureDetector> _keypoint_detector;
#else
      #error OpenCV version not supported
#endif

      //ds descriptor extraction
#if CV_MAJOR_VERSION == 2
      const cv::DescriptorExtractor* _descriptor_extractor;
#elif CV_MAJOR_VERSION == 3
      cv::Ptr<cv::DescriptorExtractor> _descriptor_extractor;
#else
      #error OpenCV version not supported
#endif

      //ds inner memory buffers (operated on in compute)
      std::vector<cv::KeyPoint> _keypoints_left;
      std::vector<cv::KeyPoint> _keypoints_right;
      cv::Mat _descriptors_left;
      cv::Mat _descriptors_right;
      std::vector<KeypointWithDescriptor> _keypoints_with_descriptors_left;
      std::vector<KeypointWithDescriptor> _keypoints_with_descriptors_right;

      //ds informative only
      CREATE_CHRONOMETER(feature_detection)
      CREATE_CHRONOMETER(descriptor_extraction)
      CREATE_CHRONOMETER(keypoint_pruning)
      CREATE_CHRONOMETER(point_triangulation)
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
