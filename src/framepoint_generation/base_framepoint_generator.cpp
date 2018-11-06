#include "base_framepoint_generator.h"

namespace proslam {

BaseFramePointGenerator::BaseFramePointGenerator(BaseFramePointGeneratorParameters* parameters_): _parameters(parameters_),
                                                                                                  _camera_left(0),
                                                                                                  _number_of_rows_image(0),
                                                                                                  _number_of_cols_image(0),
                                                                                                  _target_number_of_keypoints(1000),
                                                                                                  _target_number_of_keypoints_per_detector(1000),
                                                                                                  _number_of_detected_keypoints(0),
                                                                                                  _number_of_available_points(0),
                                                                                                  _focal_length_pixels(0),
                                                                                                  _principal_point_offset_u_pixels(0),
                                                                                                  _principal_point_offset_v_pixels(0),
                                                                                                  _maximum_depth_near_meters(0),
                                                                                                  _maximum_depth_far_meters(0),
                                                                                                  _detectors(0),
                                                                                                  _detector_thresholds(0),
                                                                                                  _number_of_detectors(0),
                                                                                                  _detector_regions(0),
                                                                                                  _framepoints_in_image(0) {
  LOG_DEBUG(std::cerr << "BaseFramePointGenerator::BaseFramePointGenerator|constructed" << std::endl)
}

void  BaseFramePointGenerator::configure(){
  LOG_DEBUG(std::cerr << "BaseFramePointGenerator::configure|configuring" << std::endl)
  assert(_camera_left);

  _number_of_rows_image            = _camera_left->numberOfImageRows();
  _number_of_cols_image            = _camera_left->numberOfImageCols();
  _focal_length_pixels             = _camera_left->cameraMatrix()(0,0);
  _principal_point_offset_u_pixels = _camera_left->cameraMatrix()(0,2);
  _principal_point_offset_v_pixels = _camera_left->cameraMatrix()(1,2);

  //ds allocate descriptor extractor TODO enable further support and check BIT SIZES
#if CV_MAJOR_VERSION == 2
  if (_parameters->descriptor_type == "BRIEF-256") {
    _descriptor_extractor = new cv::BriefDescriptorExtractor(DESCRIPTOR_SIZE_BYTES);
  } else if (_parameters->descriptor_type == "ORB-256") {
    _descriptor_extractor        = cv::ORB();
    _parameters->descriptor_type = "ORB-256";
  } else {
    LOG_WARNING(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: " << _parameters->descriptor_type
                          << " is not implemented, defaulting to ORB-256" << std::endl)
    _descriptor_extractor        = cv::ORB();
    _parameters->descriptor_type = "ORB-256";
  }
#elif CV_MAJOR_VERSION == 3
  if (_parameters->descriptor_type == "BRIEF-256") {
    #ifdef SRRG_PROSLAM_HAS_OPENCV_CONTRIB
      _descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(DESCRIPTOR_SIZE_BYTES);
    #else
      LOG_WARNING(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: BRIEF-256"
                            << " is not available in current build, defaulting to ORB-256" << std::endl)
      _descriptor_extractor        = cv::ORB::create();
      _parameters->descriptor_type = "ORB-256";
    #endif
  } else if (_parameters->descriptor_type == "ORB-256") {
    _descriptor_extractor = cv::ORB::create();
  } else if (_parameters->descriptor_type == "BRISK-512") {
    _descriptor_extractor = cv::BRISK::create();
  } else if (_parameters->descriptor_type == "FREAK-512") {
    #ifdef SRRG_PROSLAM_HAS_OPENCV_CONTRIB
        _descriptor_extractor = cv::xfeatures2d::FREAK::create();
    #else
        LOG_WARNING(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: FREAK-512"
                              << " is not available in current build, defaulting to ORB-256" << std::endl)
        _descriptor_extractor        = cv::ORB::create();
        _parameters->descriptor_type = "ORB-256";
    #endif
  } else {
    LOG_WARNING(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: " << _parameters->descriptor_type
                          << " is not implemented, defaulting to ORB-256" << std::endl)
    _descriptor_extractor        = cv::ORB::create();
    _parameters->descriptor_type = "ORB-256";
  }
#endif

  //ds log chosen descriptor type and size
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: " << _parameters->descriptor_type
                     << " (memory: " << SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS << "b)" << std::endl)

  //ds allocate and initialize detector grid structure
  _detectors           = new cv::Ptr<cv::FastFeatureDetector>*[_parameters->number_of_detectors_vertical];
  _detector_regions    = new cv::Rect*[_parameters->number_of_detectors_vertical];
  _detector_thresholds = new real*[_parameters->number_of_detectors_vertical];
  const real pixel_rows_per_detector = static_cast<real>(_number_of_rows_image)/_parameters->number_of_detectors_vertical;
  const real pixel_cols_per_detector = static_cast<real>(_number_of_cols_image)/_parameters->number_of_detectors_horizontal;
  for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
    _detectors[r]           = new cv::Ptr<cv::FastFeatureDetector>[_parameters->number_of_detectors_horizontal];
    _detector_regions[r]    = new cv::Rect[_parameters->number_of_detectors_horizontal];
    _detector_thresholds[r] = new real[_parameters->number_of_detectors_horizontal];
    for (uint32_t c = 0; c < _parameters->number_of_detectors_horizontal; ++c) {
#if CV_MAJOR_VERSION == 2
      _detectors[r][c] = new cv::FastFeatureDetector(_parameters->detector_threshold_initial);
#else
      _detectors[r][c] = cv::FastFeatureDetector::create(_parameters->detector_threshold_initial);
#endif
      _detector_regions[r][c] = cv::Rect(std::round(c*pixel_cols_per_detector),
                                         std::round(r*pixel_rows_per_detector),
                                         pixel_cols_per_detector,
                                         pixel_rows_per_detector);
      _detector_thresholds[r][c] = _parameters->detector_threshold_initial;
    }
  }
  _number_of_detectors = _parameters->number_of_detectors_vertical*_parameters->number_of_detectors_horizontal;

  //ds allocate and initialize framepoint map
  _framepoints_in_image = new FramePoint**[_number_of_rows_image];
  for (Index row = 0; row < _number_of_rows_image; ++row) {
    _framepoints_in_image[row] = new FramePoint*[_number_of_cols_image];
    for (Index col = 0; col < _number_of_cols_image; ++col) {
      _framepoints_in_image[row][col] = 0;
    }
  }

  //ds log computed values
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|focal length (pixels): " << _focal_length_pixels << std::endl)

  //ds clear buffers
  _keypoints_with_descriptors_left.clear();
  LOG_DEBUG(std::cerr << "BaseFramePointGenerator::configure|configured" << std::endl)
}

BaseFramePointGenerator::~BaseFramePointGenerator() {
  LOG_DEBUG(std::cerr << "BaseFramePointGenerator::~BaseFramePointGenerator|destroying" << std::endl)

  //ds deallocate dynamic data structures: detectors
  for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
    delete[] _detectors[r];
    delete[] _detector_regions[r];
    delete[] _detector_thresholds[r];
  }
  delete [] _detectors;
  delete [] _detector_regions;
  delete [] _detector_thresholds;

  //ds deallocate framepoint grid
  for (Index row = 0; row < _number_of_rows_image; ++row) {
    delete[] _framepoints_in_image[row];
  }
  delete[] _framepoints_in_image;

  LOG_DEBUG(std::cerr << "BaseFramePointGenerator::~BaseFramePointGenerator|destroyed" << std::endl)
}

void BaseFramePointGenerator::setTargetNumberOfKeyoints(const Count& target_number_of_keypoints_) {
  _target_number_of_keypoints = target_number_of_keypoints_;
  LOG_INFO(std::cerr << "BaseFramePointGenerator::setTargetNumberOfKeyoints|current target number of points: " << _target_number_of_keypoints << std::endl)

  //ds compute target points per detector region
  _target_number_of_keypoints_per_detector = static_cast<real>(_target_number_of_keypoints)/_number_of_detectors;
  LOG_INFO(std::cerr << "BaseFramePointGenerator::setTargetNumberOfKeyoints|current target number of points per image region: " << _target_number_of_keypoints_per_detector << std::endl)
}

void BaseFramePointGenerator::detectKeypoints(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_) {
  CHRONOMETER_START(keypoint_detection)

  //ds detect new keypoints in each image region
  for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
    for (uint32_t c = 0; c < _parameters->number_of_detectors_horizontal; ++c) {

      //ds detect keypoints in current region
      std::vector<cv::KeyPoint> keypoints_per_detector(0);
      _detectors[r][c]->detect(intensity_image_(_detector_regions[r][c]), keypoints_per_detector);

      //ds current threshold for this detector
#if CV_MAJOR_VERSION == 2
      real detector_threshold = _detectors[r][c]->getInt("threshold");
#else
      real detector_threshold = _detectors[r][c]->getThreshold();
#endif

      //ds compute point delta: 100% loss > -1, 100% gain > +1
      const real delta = (static_cast<real>(keypoints_per_detector.size())-_target_number_of_keypoints_per_detector)/_target_number_of_keypoints_per_detector;

      //ds check if there's a significant loss of target points (delta is negative)
      if (delta < -_parameters->target_number_of_keypoints_tolerance) {

        //ds compute new, lower threshold, capped and damped
        const real change = std::max(delta, -_parameters->detector_threshold_maximum_change);

        //ds always lower threshold by at least 1
        detector_threshold += std::min(change*detector_threshold, -1.0);

        //ds check minimum threshold
        if (detector_threshold < _parameters->detector_threshold_minimum) {
          detector_threshold = _parameters->detector_threshold_minimum;
        }
      }

      //ds or if there's a significant gain of target points (delta is positive)
      else if (delta > _parameters->target_number_of_keypoints_tolerance) {

        //ds compute new, higher threshold - capped and damped
        const real change = std::min(delta, _parameters->detector_threshold_maximum_change);

        //ds always increase threshold by at least 1
        detector_threshold += std::max(change*detector_threshold, 1.0);

        //ds check maximum threshold
        if (detector_threshold > _parameters->detector_threshold_maximum) {
          detector_threshold = _parameters->detector_threshold_maximum;
        }
      }

      //ds set treshold (no effect if not changed)
      _detector_thresholds[r][c] = detector_threshold;

      //ds shift keypoint coordinates to whole image region
      const cv::Point2f& offset = _detector_regions[r][c].tl();
      std::for_each(keypoints_per_detector.begin(), keypoints_per_detector.end(), [&offset](cv::KeyPoint& keypoint_) {keypoint_.pt += offset;});

      //ds add to complete vector
      keypoints_.insert(keypoints_.end(), keypoints_per_detector.begin(), keypoints_per_detector.end());
    }
  }
  _number_of_detected_keypoints = keypoints_.size();
  CHRONOMETER_STOP(keypoint_detection)
}

void BaseFramePointGenerator::extractDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_) {
  CHRONOMETER_START(descriptor_extraction)
  _descriptor_extractor->compute(intensity_image_, keypoints_, descriptors_);
  CHRONOMETER_STOP(descriptor_extraction)
}

void BaseFramePointGenerator::adjustDetectorThresholds() {
  for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
    for (uint32_t c = 0; c < _parameters->number_of_detectors_horizontal; ++c) {
#if CV_MAJOR_VERSION == 2
      _detectors[r][c]->setInt("threshold", _detector_thresholds[r][c]);
#else
      _detectors[r][c]->setThreshold(_detector_thresholds[r][c]);
#endif
    }
  }
}
}
