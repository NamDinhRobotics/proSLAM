#include "base_framepoint_generator.h"

namespace proslam {

  //ds the stereo camera setup must be provided
  BaseFramePointGenerator::BaseFramePointGenerator(): _camera_left(0),
                                                      _number_of_rows_image(0),
                                                      _number_of_cols_image(0),
                                                      _target_number_of_keypoints(1000),
                                                      _number_of_available_points(0),
                                                      _detector_threshold(10),
                                                      _detector_threshold_minimum(5),
                                                      _detector_threshold_step_size(10),
                                                      _matching_distance_tracking_threshold(50),
                                                      _matching_distance_tracking_threshold_maximum(50),
                                                      _matching_distance_tracking_threshold_minimum(50),
                                                      _matching_distance_tracking_step_size(0),
                                                      _maximum_matching_distance_triangulation(50),
                                                      _focal_length_pixels(0),
                                                      _principal_point_offset_u_pixels(0),
                                                      _principal_point_offset_v_pixels(0),
                                                      _maximum_depth_near_meters(0),
                                                      _maximum_depth_far_meters(0),
                                                      _framepoints_in_image(0)
#if CV_MAJOR_VERSION == 2
                                                      ,_keypoint_detector(0), _descriptor_extractor(0)
#endif
  {}

  void  BaseFramePointGenerator::setup(){
    std::cerr << "BaseFramePointGenerator::setup|configuring" << std::endl;
    assert(_camera_left);

    _number_of_rows_image            = _camera_left->imageRows();
    _number_of_cols_image            = _camera_left->imageCols();
    _focal_length_pixels             = _camera_left->cameraMatrix()(0,0);
    _principal_point_offset_u_pixels = _camera_left->cameraMatrix()(0,2);
    _principal_point_offset_v_pixels = _camera_left->cameraMatrix()(1,2);

#if CV_MAJOR_VERSION == 2
    _keypoint_detector = new cv::FastFeatureDetector(_detector_threshold);
    _descriptor_extractor = new cv::BriefDescriptorExtractor(DESCRIPTOR_SIZE_BYTES);
#elif CV_MAJOR_VERSION == 3
    _keypoint_detector = cv::FastFeatureDetector::create(_detector_threshold);
    _descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(DESCRIPTOR_SIZE_BYTES);
#else
#error OpenCV version not supported
#endif

    //ds allocate and initialize framepoint map
    _framepoints_in_image = new FramePoint**[_number_of_rows_image];
    for (Index row = 0; row < _number_of_rows_image; ++row) {
      _framepoints_in_image[row] = new FramePoint*[_number_of_cols_image];
      for (Index col = 0; col < _number_of_cols_image; ++col) {
        _framepoints_in_image[row][col] = 0;
      }
    }

    //ds clear buffers
    _keypoints_left.clear();
    _keypoints_with_descriptors_left.clear();
    std::cerr << "BaseFramePointGenerator::setup|configured" << std::endl;
  }

  //ds cleanup of dynamic structures
  BaseFramePointGenerator::~BaseFramePointGenerator() {
    std::cerr << "BaseFramePointGenerator::BaseFramePointGenerator|destroying" << std::endl;

    //ds deallocate dynamic data structures
    for (Index row = 0; row < _number_of_rows_image; ++row) {

      //ds check for unclaimed framepoints and remove them
      for (Index col = 0; col < _number_of_cols_image; ++col) {
        if (_framepoints_in_image[row][col]) {
          delete _framepoints_in_image[row][col];
        }
      }
      delete[] _framepoints_in_image[row];
    }
    delete[] _framepoints_in_image;

    //ds cleanup opencv
#if CV_MAJOR_VERSION == 2
    delete _keypoint_detector;
    delete _descriptor_extractor;
#endif

    std::cerr << "BaseFramePointGenerator::BaseFramePointGenerator|destroyed" << std::endl;
  }


  //ds detects keypoints and stores them in a vector (called within compute)
  void BaseFramePointGenerator::detectKeypoints(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_) {
    CHRONOMETER_START(feature_detection)

    //ds detect new keypoints
    _keypoint_detector->detect(intensity_image_, keypoints_);

    //ds compute point delta
    const real delta = (static_cast<real>(keypoints_.size())-_target_number_of_keypoints)/keypoints_.size();

    //ds check if there's a significant loss of target points
    if (delta < -0.1) {

      //ds compute new threshold
      _detector_threshold += std::max(std::ceil(delta*_detector_threshold_step_size), -_detector_threshold_step_size);

      //ds cap the minimum value
      if (_detector_threshold < _detector_threshold_minimum) {
        _detector_threshold = _detector_threshold_minimum;
      }
      setDetectorThreshold(_detector_threshold);

      //ds increase allowed matching distance if possible
      if (_matching_distance_tracking_threshold < _matching_distance_tracking_threshold_maximum) {
        _matching_distance_tracking_threshold += _matching_distance_tracking_step_size;
      }
    }

    //ds or if there's a significant gain of target points
    else if (delta > 0.1) {

      //ds compute new threshold
      _detector_threshold += std::min(std::ceil(delta*_detector_threshold_step_size), _detector_threshold_step_size);

      //ds raise threshold (uncapped)
      setDetectorThreshold(_detector_threshold);

      //ds lower allowed matching distance if possible
      if (_matching_distance_tracking_threshold > _matching_distance_tracking_threshold_minimum) {
        _matching_distance_tracking_threshold -= _matching_distance_tracking_step_size;
      }
    }

    CHRONOMETER_STOP(feature_detection)
  }

  //ds extracts the defined descriptors for the given keypoints (called within compute)
  void BaseFramePointGenerator::extractDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_) {
    CHRONOMETER_START(descriptor_extraction)
    _descriptor_extractor->compute(intensity_image_, keypoints_, descriptors_);
    CHRONOMETER_STOP(descriptor_extraction)
  }

  void BaseFramePointGenerator::clearFramepointsInImage() {
    for (Index row = 0; row < _number_of_rows_image; ++row) {
      for (Count col = 0; col < _number_of_cols_image; ++col) {
        _framepoints_in_image[row][col] = 0;
      }
    }
  }

  void BaseFramePointGenerator::setDetectorThreshold(const int32_t& detector_threshold_) {
    _detector_threshold = detector_threshold_;

#if CV_MAJOR_VERSION == 2
    _keypoint_detector->setInt("threshold", _detector_threshold);
#elif CV_MAJOR_VERSION == 3
    _keypoint_detector->setThreshold(_detector_threshold);
#else
#error OpenCV version not supported
#endif
  }
}
