#include "base_framepoint_generator.h"

namespace proslam {

BaseFramePointGenerator::BaseFramePointGenerator(BaseFramePointGeneratorParameters* parameters_): _parameters(parameters_),
                                                                                                  _number_of_rows_image(0),
                                                                                                  _number_of_cols_image(0),
                                                                                                  _target_number_of_keypoints(1000),
                                                                                                  _target_number_of_keypoints_per_detector(1000),
                                                                                                  _number_of_detected_keypoints(0),
                                                                                                  _number_of_available_points(0),
                                                                                                  _focal_length_pixels(0),
                                                                                                  _principal_point_offset_u_pixels(0),
                                                                                                  _principal_point_offset_v_pixels(0),
                                                                                                  _number_of_detectors(0) {
  LOG_INFO(std::cerr << "BaseFramePointGenerator::BaseFramePointGenerator|constructed" << std::endl)
}

void  BaseFramePointGenerator::configure(){
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|configuring" << std::endl)
  assert(_camera_left);

  _number_of_rows_image            = _camera_left->numberOfImageRows();
  _number_of_cols_image            = _camera_left->numberOfImageCols();
  _focal_length_pixels             = _camera_left->cameraMatrix()(0,0);
  _principal_point_offset_u_pixels = _camera_left->cameraMatrix()(0,2);
  _principal_point_offset_v_pixels = _camera_left->cameraMatrix()(1,2);
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|focal length (pixels): " << _focal_length_pixels << std::endl)

  //ds initialize feature matcher
  _feature_matcher_left.configure(_number_of_rows_image, _number_of_cols_image);

  //ds configure tracking window
  _projection_tracking_distance_pixels = _parameters->maximum_projection_tracking_distance_pixels;

  //ds allocate descriptor extractor TODO enable further support and check BIT SIZES
#if CV_MAJOR_VERSION == 2
  if (_parameters->descriptor_type == "BRIEF-256") {
    _descriptor_extractor = new cv::BriefDescriptorExtractor(DESCRIPTOR_SIZE_BYTES);
  } else if (_parameters->descriptor_type == "ORB-256") {
    _descriptor_extractor        = new cv::OrbDescriptorExtractor();
    _parameters->descriptor_type = "ORB-256";
  } else {
    LOG_WARNING(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: " << _parameters->descriptor_type
                          << " is not implemented, defaulting to ORB-256" << std::endl)
    _descriptor_extractor        = new cv::OrbDescriptorExtractor();
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
      _detectors[r][c] = new cv::FastFeatureDetector(_parameters->detector_threshold_minimum);
#else
      _detectors[r][c] = cv::FastFeatureDetector::create(_parameters->detector_threshold_minimum);
#endif
      _detector_regions[r][c] = cv::Rect(std::round(c*pixel_cols_per_detector),
                                         std::round(r*pixel_rows_per_detector),
                                         pixel_cols_per_detector,
                                         pixel_rows_per_detector);
      _detector_thresholds[r][c] = 0;
    }
  }
  _number_of_detectors = _parameters->number_of_detectors_vertical*_parameters->number_of_detectors_horizontal;

  //ds compute binning configuration
  _number_of_cols_bin = std::floor(static_cast<real>(_camera_left->numberOfImageCols())/_parameters->bin_size_pixels)+1;
  _number_of_rows_bin = std::floor(static_cast<real>(_camera_left->numberOfImageRows())/_parameters->bin_size_pixels)+1;

  //ds compute target number of points
  _target_number_of_keypoints = _number_of_cols_bin*_number_of_rows_bin;
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|current target number of points: " << _target_number_of_keypoints << std::endl)

  //ds compute target points per detector region
  _target_number_of_keypoints_per_detector = static_cast<real>(_target_number_of_keypoints)/_number_of_detectors;
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|current target number of points per image region: " << _target_number_of_keypoints_per_detector << std::endl)

  //ds allocate and initialize bin grid
  _bin_map_left = new FramePoint**[_number_of_rows_bin];
  for (Index row = 0; row < _number_of_rows_bin; ++row) {
    _bin_map_left[row] = new FramePoint*[_number_of_cols_bin];
    for (Index col = 0; col < _number_of_cols_bin; ++col) {
      _bin_map_left[row][col] = nullptr;
    }
  }
  LOG_INFO(std::cerr << "BaseTracker::configure|number of horizontal bins: " << _number_of_cols_bin << " size: " << _parameters->bin_size_pixels << std::endl)
  LOG_INFO(std::cerr << "BaseTracker::configure|number of vertical bins: " << _number_of_rows_bin << " size: " << _parameters->bin_size_pixels << std::endl)

  //ds clear buffers
  _keypoints_with_descriptors_left.clear();
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|configured" << std::endl)
}

BaseFramePointGenerator::~BaseFramePointGenerator() {
  LOG_INFO(std::cerr << "BaseFramePointGenerator::~BaseFramePointGenerator|destroying" << std::endl)

  //ds deallocate dynamic data structures: detectors
  if (_detectors && _detector_regions && _detector_thresholds) {
    for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
      delete[] _detectors[r];
      delete[] _detector_regions[r];
      delete[] _detector_thresholds[r];
    }
    delete [] _detectors;
    delete [] _detector_regions;
    delete [] _detector_thresholds;
  }

  //ds free bin map
  for (Count row = 0; row < _number_of_rows_bin; ++row) {
    delete[] _bin_map_left[row];
  }
  delete[] _bin_map_left;
  LOG_INFO(std::cerr << "BaseFramePointGenerator::~BaseFramePointGenerator|destroyed" << std::endl)
}

void BaseFramePointGenerator::detectKeypoints(const cv::Mat& intensity_image_,
                                              std::vector<cv::KeyPoint>& keypoints_,
                                              const bool ignore_minimum_detector_threshold_) {
  CHRONOMETER_START(keypoint_detection)

  //ds detect new keypoints in each image region
  for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
    for (uint32_t c = 0; c < _parameters->number_of_detectors_horizontal; ++c) {

      //ds detect keypoints in current region
      std::vector<cv::KeyPoint> keypoints_per_detector(0);
      _detectors[r][c]->detect(intensity_image_(_detector_regions[r][c]), keypoints_per_detector);

      //ds retrieve currently set threshold for this detector
#if CV_MAJOR_VERSION == 2
      real detector_threshold = _detectors[r][c]->getInt("threshold");
#else
      real detector_threshold = _detectors[r][c]->getThreshold();
#endif

      //ds compute point delta: 100% loss > -1, 100% gain > +1
      const real delta = (static_cast<real>(keypoints_per_detector.size())-_target_number_of_keypoints_per_detector)/_target_number_of_keypoints_per_detector;

      //ds check if there's a significant loss of target points (delta is negative)
      if (delta < -_parameters->target_number_of_keypoints_tolerance) {

        //ds compute new, lower threshold, capped (negative value)
        const real change = std::max(delta, -_parameters->detector_threshold_maximum_change);

        //ds always lower threshold by at least 1
        real detector_threshold_new = detector_threshold+std::min(change*detector_threshold, -1.0);

        //ds minimum feasible threshold is 0
        detector_threshold_new = std::max(detector_threshold_new, 0.0);

        //ds check if we can below the minimum threshold
        if (detector_threshold_new >= _parameters->detector_threshold_minimum || ignore_minimum_detector_threshold_) {
          detector_threshold = detector_threshold_new;
        }
      }

      //ds or if there's a significant gain of target points (delta is positive)
      else if (delta > _parameters->target_number_of_keypoints_tolerance) {

        //ds compute new, higher threshold, capped (positive value)
        const real change = std::min(delta, _parameters->detector_threshold_maximum_change);

        //ds always increase threshold by at least 1
        detector_threshold += std::max(change*detector_threshold, 1.0);

        //ds check maximum threshold
        if (detector_threshold > _parameters->detector_threshold_maximum) {
          detector_threshold = _parameters->detector_threshold_maximum;
        }
      }

      //ds set treshold variable (will be effectively changed by calling adjustDetectorThresholds)
      _detector_thresholds[r][c] += detector_threshold;

      //ds shift keypoint coordinates to whole image region
      const cv::Point2f& offset = _detector_regions[r][c].tl();
      std::for_each(keypoints_per_detector.begin(), keypoints_per_detector.end(), [&offset](cv::KeyPoint& keypoint_) {keypoint_.pt += offset;});

      //ds add to complete vector
      keypoints_.insert(keypoints_.end(), keypoints_per_detector.begin(), keypoints_per_detector.end());
    }
  }
  ++_number_of_detections;
  _number_of_detected_keypoints = keypoints_.size();
  CHRONOMETER_STOP(keypoint_detection)
}

void BaseFramePointGenerator::computeDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_) {
  CHRONOMETER_START(descriptor_extraction)
  _descriptor_extractor->compute(intensity_image_, keypoints_, descriptors_);
  CHRONOMETER_STOP(descriptor_extraction)
}

void BaseFramePointGenerator::adjustDetectorThresholds() {
  for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
    for (uint32_t c = 0; c < _parameters->number_of_detectors_horizontal; ++c) {

      //ds compute average threshold over last detections
      _detector_thresholds[r][c] /= _number_of_detections;

#if CV_MAJOR_VERSION == 2
      _detectors[r][c]->setInt("threshold", std::rint(_detector_thresholds[r][c]));
#else
      _detectors[r][c]->setThreshold(std::rint(_detector_thresholds[r][c]));
#endif

      //ds reset bookkeeping for next detection(s)
      _detector_thresholds[r][c] = 0;
    }
  }
  _number_of_detections = 0;
}

const PointCoordinates BaseFramePointGenerator::getPointInCamera(const cv::Point2f& image_point_previous_,
                                                                 const cv::Point2f& image_point_current_,
                                                                 const TransformMatrix3D& camera_previous_to_current_,
                                                                 const Matrix3& camera_calibration_matrix_) const {
  const Eigen::Matrix<real, 1, 3>& r_1 = camera_previous_to_current_.linear().block<1,3>(0,0);
  const Eigen::Matrix<real, 1, 3>& r_2 = camera_previous_to_current_.linear().block<1,3>(1,0);
  const Eigen::Matrix<real, 1, 3>& r_3 = camera_previous_to_current_.linear().block<1,3>(2,0);

  //ds obtain normalized image coordinates
  const real a_0 = (image_point_previous_.x-camera_calibration_matrix_(0,2))/camera_calibration_matrix_(0,0);
  const real b_0 = (image_point_previous_.y-camera_calibration_matrix_(1,2))/camera_calibration_matrix_(1,1);
  const real a_1 = (image_point_current_.x-camera_calibration_matrix_(0,2))/camera_calibration_matrix_(0,0);
  const real b_1 = (image_point_current_.y-camera_calibration_matrix_(1,2))/camera_calibration_matrix_(1,1);

  //ds initialize homogeneous coordinates in x and y
  const PointCoordinates x_0(a_0, b_0, 1);
  const PointCoordinates x_1(a_1, b_1, 1);

  //ds build constraint matrix
  Eigen::Matrix<real, 3, 2> A;
  A << -r_1*x_0, a_1,
       -r_2*x_0, b_1,
       -r_3*x_0, 1;

  //ds minimize squared error for both image points
  const Vector2 z = A.jacobiSvd(Eigen::ComputeFullU|Eigen::ComputeFullV).solve(camera_previous_to_current_.translation());

  //ds compute candidates
  const PointCoordinates point_in_camera_previous = x_0*z(0);
  const PointCoordinates point_in_camera_current  = x_1*z(1);

  //ds compute midpoint in current frame
  return (point_in_camera_current+camera_previous_to_current_*point_in_camera_previous)/2.0;
}
}
