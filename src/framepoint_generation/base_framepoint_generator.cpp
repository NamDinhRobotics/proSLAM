#include "base_framepoint_generator.h"

namespace proslam {

  //ds the stereo camera setup must be provided
  BaseFramePointGenerator::BaseFramePointGenerator(): _camera_left(0),
                                                      _number_of_rows_image(0),
                                                      _number_of_cols_image(0),
                                                      _target_number_of_keypoints(1000),
                                                      _number_of_available_points(0),
                                                      _focal_length_pixels(0),
                                                      _principal_point_offset_u_pixels(0),
                                                      _principal_point_offset_v_pixels(0),
                                                      _maximum_depth_near_meters(0),
                                                      _maximum_depth_far_meters(0),
                                                      _framepoints_in_image(0),
#if CV_MAJOR_VERSION == 2
                                                      _descriptor_extractor(0),
#endif
                                                      _keypoint_detector(0),
                                                      _parameters(0) {
    LOG_DEBUG(std::cerr << "BaseFramePointGenerator::BaseFramePointGenerator|constructed" << std::endl)
  }

  void  BaseFramePointGenerator::configure(BaseFramepointGeneratorParameters* parameters_){
    LOG_DEBUG(std::cerr << "BaseFramePointGenerator::configure|configuring" << std::endl)
    _parameters = parameters_;
    assert(_camera_left);

    //ds allocate an configure a caffe detector
    _keypoint_detector = new nnkd::KeypointDetector();
    _keypoint_detector->configure(parameters_->proto_file, parameters_->model_file, false, _camera_left->imageRows(), _camera_left->imageCols(), parameters_->sliding_window_step_size);

    //ds configure generic framepoint generation
    _number_of_rows_image            = _camera_left->imageRows();
    _number_of_cols_image            = _camera_left->imageCols();
    _focal_length_pixels             = _camera_left->cameraMatrix()(0,0);
    _principal_point_offset_u_pixels = _camera_left->cameraMatrix()(0,2);
    _principal_point_offset_v_pixels = _camera_left->cameraMatrix()(1,2);

#if CV_MAJOR_VERSION == 2
    _descriptor_extractor = new cv::BriefDescriptorExtractor(DESCRIPTOR_SIZE_BYTES);
#elif CV_MAJOR_VERSION == 3
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
    _keypoints_with_descriptors_left.clear();
    LOG_DEBUG(std::cerr << "BaseFramePointGenerator::configure|configured" << std::endl)
  }

  //ds cleanup of dynamic structures
  BaseFramePointGenerator::~BaseFramePointGenerator() {
    LOG_DEBUG(std::cerr << "BaseFramePointGenerator::~BaseFramePointGenerator|destroying" << std::endl)

    //ds deallocate dynamic data structures
    for (Index row = 0; row < _number_of_rows_image; ++row) {
      delete[] _framepoints_in_image[row];
    }
    delete[] _framepoints_in_image;

    //ds cleanup opencv
#if CV_MAJOR_VERSION == 2
    delete _keypoint_detector;
    delete _descriptor_extractor;
#endif

    LOG_DEBUG(std::cerr << "BaseFramePointGenerator::~BaseFramePointGenerator|destroyed" << std::endl)
  }


  //ds detects keypoints and stores them in a vector (called within compute)
  void BaseFramePointGenerator::detectKeypoints(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_) {
    CHRONOMETER_START(feature_detection)

    //ds detect new keypoints
    _keypoint_detector->detect(intensity_image_, keypoints_);

//    //ds compute point delta
//    const real delta = (static_cast<real>(keypoints_.size())-_target_number_of_keypoints)/keypoints_.size();
//
//    //ds check if there's a significant loss of target points
//    if (delta < -_parameters->target_number_of_keypoints_tolerance) {
//
//      //ds compute new threshold
//      _parameters->detector_threshold += std::max(std::ceil(delta*_parameters->detector_threshold_step_size), -_parameters->detector_threshold_step_size);
//
//      //ds cap the minimum value
//      if (_parameters->detector_threshold < _parameters->detector_threshold_minimum) {
//        _parameters->detector_threshold = _parameters->detector_threshold_minimum;
//      }
//      setDetectorThreshold(_parameters->detector_threshold);
//
//      //ds increase allowed matching distance if possible
//      if (_parameters->matching_distance_tracking_threshold < _parameters->matching_distance_tracking_threshold_maximum) {
//        _parameters->matching_distance_tracking_threshold += _parameters->matching_distance_tracking_step_size;
//      }
//    }
//
//    //ds or if there's a significant gain of target points
//    else if (delta > _parameters->target_number_of_keypoints_tolerance) {
//
//      //ds compute new threshold
//      _parameters->detector_threshold += std::min(std::ceil(delta*_parameters->detector_threshold_step_size), _parameters->detector_threshold_step_size);
//
//      //ds raise threshold (uncapped)
//      setDetectorThreshold(_parameters->detector_threshold);
//
//      //ds lower allowed matching distance if possible
//      if (_parameters->matching_distance_tracking_threshold > _parameters->matching_distance_tracking_threshold_minimum) {
//        _parameters->matching_distance_tracking_threshold -= _parameters->matching_distance_tracking_step_size;
//      }
//    }

    CHRONOMETER_STOP(feature_detection)
  }

  //ds extracts the defined descriptors for the given keypoints (called within compute)
  void BaseFramePointGenerator::extractDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_) {
    CHRONOMETER_START(descriptor_extraction)
    _descriptor_extractor->compute(intensity_image_, keypoints_, descriptors_);
    CHRONOMETER_STOP(descriptor_extraction)
  }

  void BaseFramePointGenerator::setDetectorThreshold(const int32_t& detector_threshold_) {
    _parameters->detector_threshold = detector_threshold_;
  }
}
