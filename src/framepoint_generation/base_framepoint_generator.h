#pragma once
#include "types/frame.h"

namespace proslam {

  //ds this class computes potential framepoints in a stereo image pair by triangulation
  class BaseFramePointGenerator {

  //ds exported types
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds container holding spatial and appearance information (used in findStereoKeypoints)
    struct KeypointWithDescriptor {
      cv::KeyPoint keypoint;
      cv::Mat descriptor;
      int32_t row; //ds keypoint v coordinate
      int32_t col; //ds keypoint u coordinate
    };

    //ds a 2d array of pointers to framepoints
    typedef FramePoint*** FramePointMatrix;

  //ds object handling
  public:

    //ds the stereo camera setup must be provided
    BaseFramePointGenerator();

    //gg to be called after constructor and parameters are set
    virtual void setup();

    //ds cleanup of dynamic structures
    virtual ~BaseFramePointGenerator();
 
  //ds functionality
  public:

    //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
    virtual void compute(Frame* frame_) = 0;

    //ds detects keypoints and stores them in a vector (called within compute)
    void detectKeypoints(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_);

    //ds extracts the defined descriptors for the given keypoints (called within compute)
    void extractDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_);

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
    void setCameraLeft(const Camera* camera_left_) {_camera_left=camera_left_;}
    const Count& numberOfRowsImage() const {return _number_of_rows_image;}
    const Count& numberOfColsImage() const {return _number_of_cols_image;}
    const real maximumDepthNearMeters() const {return _maximum_depth_near_meters;}
    const real maximumDepthFarMeters() const {return _maximum_depth_far_meters;}
    const Count& targetNumberOfKeypoints() const {return _target_number_of_keypoints;}
    void setTargetNumberOfKeyoints(const Count& target_number_of_keypoints_) {_target_number_of_keypoints = target_number_of_keypoints_;}

    void setDetectorThreshold(const int32_t& detector_threshold_);
    void setDetectorThresholdMinimum(const int32_t& detector_threshold_minimum_) {_detector_threshold_minimum = detector_threshold_minimum_;}
    const int32_t matchingDistanceTrackingThreshold() const {return _matching_distance_tracking_threshold;}
    void setMatchingDistanceTrackingThresholdMaximum(const real& matching_distance_tracking_threshold_maximum_) {_matching_distance_tracking_threshold_maximum = matching_distance_tracking_threshold_maximum_;}
    void setMatchingDistanceTrackingThresholdMinimum(const real& matching_distance_tracking_threshold_minimum_) {_matching_distance_tracking_threshold_minimum = matching_distance_tracking_threshold_minimum_;}
    void setMaximumMatchingDistanceTriangulation(const real& maximum_matching_distance_triangulation_) {_maximum_matching_distance_triangulation = maximum_matching_distance_triangulation_;}
    const Count numberOfAvailablePoints() const {return _number_of_available_points;}

    //ds settings
  protected:

    const Camera* _camera_left;
    
    //ds input properties
    Count _number_of_rows_image;
    Count _number_of_cols_image;

    //ds point detection properties
    Count _target_number_of_keypoints;
    Count _number_of_available_points;

    //ds dynamic thresholds for feature detection
    real _target_number_of_keypoints_tolerance;
    int32_t _detector_threshold;
    int32_t _detector_threshold_minimum;
    real _detector_threshold_step_size;

    //ds dynamic thresholds for descriptor matching
    int32_t _matching_distance_tracking_threshold;
    int32_t _matching_distance_tracking_threshold_maximum;
    int32_t _matching_distance_tracking_threshold_minimum;
    int32_t _matching_distance_tracking_step_size;

    //ds triangulation properties
    int32_t _maximum_matching_distance_triangulation;
    real _focal_length_pixels;
    real _principal_point_offset_u_pixels;
    real _principal_point_offset_v_pixels;
    real _maximum_depth_near_meters;
    real _maximum_depth_far_meters;

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
    cv::Mat _descriptors_left;
    std::vector<KeypointWithDescriptor> _keypoints_with_descriptors_left;

    //ds informative only
    CREATE_CHRONOMETER(feature_detection)
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
