#include "stereo_triangulator.h"

namespace proslam {

  //ds the stereo camera setup must be provided
  StereoTriangulator::StereoTriangulator(const Camera* camera_left_, const Camera* camera_right_): _number_of_rows_image(camera_left_->imageRows()),
                                                                                                   _number_of_cols_image(camera_left_->imageCols()),
                                                                                                   _number_of_bins_u(std::floor(static_cast<real>(_number_of_cols_image)/_bin_size)),
                                                                                                   _number_of_bins_v(std::floor(static_cast<real>(_number_of_rows_image)/_bin_size)),
                                                                                                   _focal_length_pixels(camera_left_->projectionMatrix()(0,0)),
                                                                                                   _principal_point_offset_u_pixels(camera_left_->projectionMatrix()(0,2)),
                                                                                                   _principal_point_offset_v_pixels(camera_left_->projectionMatrix()(1,2)),
                                                                                                   _baseline_pixelsmeters(camera_right_->projectionMatrix()(0,3)),
                                                                                                   _baseline_meters(-_baseline_pixelsmeters/_focal_length_pixels),
                                                                                                   _maximum_depth_near_meters(_baseline_factor*_baseline_meters),
                                                                                                   _maximum_depth_far_meters(-_baseline_pixelsmeters/_minimum_disparity_pixels),
#if CV_MAJOR_VERSION == 2
                                                                                                   _keypoint_detector(new cv::FastFeatureDetector(_detector_threshold)),
                                                                                                   _descriptor_extractor(new cv::BriefDescriptorExtractor(DESCRIPTOR_SIZE_BYTES)) {
#elif CV_MAJOR_VERSION == 3
                                                                                                   _keypoint_detector(cv::FastFeatureDetector::create(_detector_threshold)),
                                                                                                   _descriptor_extractor(cv::xfeatures2d::BriefDescriptorExtractor::create(DESCRIPTOR_SIZE_BYTES)) {
#else
#error OpenCV version not supported
#endif
    std::cerr << "StereoTriangulator::StereoTriangulator|constructing" << std::endl;
    assert(camera_left_->imageCols() == camera_right_->imageCols());
    assert(camera_left_->imageRows() == camera_right_->imageRows());

    //ds allocate and initialize framepoint map
    _framepoints_in_image = new FramePoint**[_number_of_rows_image];
    for (Index row = 0; row < _number_of_rows_image; ++row) {
      _framepoints_in_image[row] = new FramePoint*[_number_of_cols_image];
      for (Count col = 0; col < _number_of_cols_image; ++col) {
        _framepoints_in_image[row][col] = 0;
      }
    }

    //ds allocate and initialize bin grid
    _bin_map_left = new cv::KeyPoint*[_number_of_bins_v];
    for (Count v = 0; v < _number_of_bins_v; ++v) {
      _bin_map_left[v] = new cv::KeyPoint[_number_of_bins_u];
      for (Count u = 0; u < _number_of_bins_u; ++u) {
        _bin_map_left[v][u].response = 0;
      }
    }

    //ds clear buffers
    _keypoints_left.clear();
    _keypoints_right.clear();
    _keypoints_with_descriptors_left.clear();
    _keypoints_with_descriptors_right.clear();

    //ds info
    std::cerr << "StereoTriangulator::StereoTriangulator|baseline (m): " << _baseline_meters << std::endl;
    std::cerr << "StereoTriangulator::StereoTriangulator|maximum depth tracking close (m): " << _maximum_depth_near_meters << std::endl;
    std::cerr << "StereoTriangulator::StereoTriangulator|maximum depth tracking far (m): " << _maximum_depth_far_meters << std::endl;
    std::cerr << "StereoTriangulator::StereoTriangulator|bin size (pixel): " << _bin_size << std::endl;
    std::cerr << "StereoTriangulator::StereoTriangulator|number of bins u: " << _number_of_bins_u << std::endl;
    std::cerr << "StereoTriangulator::StereoTriangulator|number of bins v: " << _number_of_bins_v << std::endl;
    std::cerr << "StereoTriangulator::StereoTriangulator|total number of bins: " << _number_of_bins_u*_number_of_bins_v << std::endl;
    std::cerr << "StereoTriangulator::StereoTriangulator|constructed" << std::endl;
  }

  //ds cleanup of dynamic structures
  StereoTriangulator::~StereoTriangulator() {
    std::cerr << "StereoTriangulator::StereoTriangulator|destroying" << std::endl;

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
    for (Count v = 0; v < _number_of_bins_v; ++v) {
      delete[] _bin_map_left[v];
    }
    delete[] _bin_map_left;

    //ds cleanup opencv
#if CV_MAJOR_VERSION == 2
    delete _keypoint_detector;
    delete _descriptor_extractor;
#endif

    std::cerr << "StereoTriangulator::StereoTriangulator|destroyed" << std::endl;
  }

  //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
  void StereoTriangulator::compute(Frame* frame_) {

    //ds detect new features
    CHRONOMETER_START(feature_detection)
    detectKeypoints(frame_->intensityImageLeft(), _keypoints_left);
    detectKeypoints(frame_->intensityImageRight(), _keypoints_right);
    CHRONOMETER_STOP(feature_detection)

    //ds keypoint pruning - prune only left side
    CHRONOMETER_START(keypoint_pruning)
    binKeypoints(_keypoints_left, _bin_map_left);
    CHRONOMETER_STOP(keypoint_pruning)

    //ds extract descriptors for detected features
    CHRONOMETER_START(descriptor_extraction)
    extractDescriptors(frame_->intensityImageLeft(), _keypoints_left, _descriptors_left);
    extractDescriptors(frame_->intensityImageRight(), _keypoints_right, _descriptors_right);
    CHRONOMETER_STOP(descriptor_extraction)

    //ds prepare and execute stereo keypoint search
    CHRONOMETER_START(point_triangulation)
    initialize(_keypoints_left, _keypoints_right, _descriptors_left, _descriptors_right);
    findStereoKeypoints(frame_);
    CHRONOMETER_STOP(point_triangulation)

    //ds calibrate feature detector threshold to maintain the target number of tracked points
    calibrateDetectionThresholds();
  }

  //ds detects keypoints and stores them in a vector (called within compute)
  void StereoTriangulator::detectKeypoints(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_) const {
    _keypoint_detector->detect(intensity_image_, keypoints_);
  }

  //ds regularizes the detected keypoints using binning (called within compute)
  void StereoTriangulator::binKeypoints(std::vector<cv::KeyPoint>& keypoints_, cv::KeyPoint** bin_map_) const {

    //ds sort by position in u
    std::sort(keypoints_.begin(), keypoints_.end(), [](const cv::KeyPoint& a_, const cv::KeyPoint& b_) {return a_.pt.x < b_.pt.x;});

    //ds check all keypoints for this grid
    Count u_current = 0;
    for (const cv::KeyPoint& keypoint: keypoints_) {
      const real& keypoint_u = keypoint.pt.x;
      const real& keypoint_v = keypoint.pt.y;

      //ds if the keypoint still enters the current bin
      if (keypoint_u < u_current*_bin_size) {
        assert(u_current < _number_of_bins_u);

        //ds check matching bin range in V
        for (Count v = 0; v < _number_of_bins_v; ++v) {
          if (keypoint_v < v*_bin_size) {

            //ds found matching bin - check if the reponse is higher
            if (keypoint.response > bin_map_[v][u_current].response) {
              bin_map_[v][u_current] = keypoint;
            }
            break;
          }
        }
      } else {
        ++u_current;

        //ds if we reached the end of the grid
        if (u_current == _number_of_bins_u) {
          break;
        }
      }
    }

    //ds collect keypoints from all grids into one single vector - and at the same time prepare the data structure for the next binning
    Index index_keypoint  = 0;
    for (Count v = 0; v < _number_of_bins_v; ++v) {
      for (Count u = 0; u < _number_of_bins_u; ++u) {
        if (bin_map_[v][u].response > 0) {
          keypoints_[index_keypoint] = bin_map_[v][u];
          ++index_keypoint;
          bin_map_[v][u].response = 0;
        }
      }
    }
    keypoints_.resize(index_keypoint);
  }

  //ds extracts the defined descriptors for the given keypoints (called within compute)
  void StereoTriangulator::extractDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_) const {
    _descriptor_extractor->compute(intensity_image_, keypoints_, descriptors_);
  }

  //ds initializes structures for the epipolar stereo keypoint search (called within compute)
  void StereoTriangulator::initialize(const std::vector<cv::KeyPoint>& keypoints_left_,
                                      const std::vector<cv::KeyPoint>& keypoints_right_,
                                      const cv::Mat& descriptors_left_,
                                      const cv::Mat& descriptors_right_) {

    //ds prepare keypoint with descriptors vectors for stereo keypoint search
    _keypoints_with_descriptors_left.resize(keypoints_left_.size());
    _keypoints_with_descriptors_right.resize(keypoints_right_.size());
    if (keypoints_left_.size() <= keypoints_right_.size()) {
      for (Index u = 0; u < keypoints_left_.size(); ++u) {
        _keypoints_with_descriptors_left[u].keypoint    = keypoints_left_[u];
        _keypoints_with_descriptors_left[u].descriptor  = descriptors_left_.row(u);
        _keypoints_with_descriptors_left[u].row         = keypoints_left_[u].pt.y;
        _keypoints_with_descriptors_left[u].col         = keypoints_left_[u].pt.x;
        _keypoints_with_descriptors_right[u].keypoint   = keypoints_right_[u];
        _keypoints_with_descriptors_right[u].descriptor = descriptors_right_.row(u);
        _keypoints_with_descriptors_right[u].row        = keypoints_right_[u].pt.y;
        _keypoints_with_descriptors_right[u].col        = keypoints_right_[u].pt.x;
      }
      for (Index u = keypoints_left_.size(); u < keypoints_right_.size(); ++u) {
        _keypoints_with_descriptors_right[u].keypoint   = keypoints_right_[u];
        _keypoints_with_descriptors_right[u].descriptor = descriptors_right_.row(u);
        _keypoints_with_descriptors_right[u].row        = keypoints_right_[u].pt.y;
        _keypoints_with_descriptors_right[u].col        = keypoints_right_[u].pt.x;
      }
    } else {
      for (Index u = 0; u < keypoints_right_.size(); ++u) {
        _keypoints_with_descriptors_left[u].keypoint    = keypoints_left_[u];
        _keypoints_with_descriptors_left[u].descriptor  = descriptors_left_.row(u);
        _keypoints_with_descriptors_left[u].row         = keypoints_left_[u].pt.y;
        _keypoints_with_descriptors_left[u].col         = keypoints_left_[u].pt.x;
        _keypoints_with_descriptors_right[u].keypoint   = keypoints_right_[u];
        _keypoints_with_descriptors_right[u].descriptor = descriptors_right_.row(u);
        _keypoints_with_descriptors_right[u].row        = keypoints_right_[u].pt.y;
        _keypoints_with_descriptors_right[u].col        = keypoints_right_[u].pt.x;
      }
      for (Index u = keypoints_right_.size(); u < keypoints_left_.size(); ++u) {
        _keypoints_with_descriptors_left[u].keypoint   = keypoints_left_[u];
        _keypoints_with_descriptors_left[u].descriptor = descriptors_left_.row(u);
        _keypoints_with_descriptors_left[u].row        = keypoints_left_[u].pt.y;
        _keypoints_with_descriptors_left[u].col        = keypoints_left_[u].pt.x;
      }
    }
  }

  //ds computes all potential stereo keypoints (exhaustive in matching distance) and stores them as framepoints (called within compute)
  void StereoTriangulator::findStereoKeypoints(Frame* frame_) {

    //ds sort all input vectors by ascending row positions
    std::sort(_keypoints_with_descriptors_left.begin(), _keypoints_with_descriptors_left.end(), [](const KeypointWithDescriptor& a_, const KeypointWithDescriptor& b_){return ((a_.row < b_.row) || (a_.row == b_.row && a_.col < b_.col));});
    std::sort(_keypoints_with_descriptors_right.begin(), _keypoints_with_descriptors_right.end(), [](const KeypointWithDescriptor& a_, const KeypointWithDescriptor& b_){return ((a_.row < b_.row) || (a_.row == b_.row && a_.col < b_.col));});

    //ds running variable
    Index idx_R = 0;

    //ds loop over all left keypoints
    _number_of_available_points = 0;
    for (Index idx_L = 0; idx_L < _keypoints_with_descriptors_left.size(); idx_L++) {
      //stop condition
      if (idx_R == _keypoints_with_descriptors_right.size()) {break;}
      //the right keypoints are on an lower row - skip left
      while (_keypoints_with_descriptors_left[idx_L].row < _keypoints_with_descriptors_right[idx_R].row) {
        idx_L++; if (idx_L == _keypoints_with_descriptors_left.size()) {break;}
      }
      if (idx_L == _keypoints_with_descriptors_left.size()) {break;}
      //the right keypoints are on an upper row - skip right
      while (_keypoints_with_descriptors_left[idx_L].row > _keypoints_with_descriptors_right[idx_R].row) {
        idx_R++; if (idx_R == _keypoints_with_descriptors_right.size()) {break;}
      }
      if (idx_R == _keypoints_with_descriptors_right.size()) {break;}
      //search bookkeeping
      Index idx_RS = idx_R;
      real dist_best = _maximum_matching_distance_triangulation;
      Index idx_best_R = 0;
      //scan epipolar line for current keypoint at idx_L
      while (_keypoints_with_descriptors_left[idx_L].row == _keypoints_with_descriptors_right[idx_RS].row) {
        //zero disparity stop condition
        if (_keypoints_with_descriptors_right[idx_RS].col >= _keypoints_with_descriptors_left[idx_L].col) {break;}
        //compute descriptor distance
        const real dist = cv::norm(_keypoints_with_descriptors_left[idx_L].descriptor, _keypoints_with_descriptors_right[idx_RS].descriptor, DESCRIPTOR_NORM);
        if(dist < dist_best) {
          dist_best = dist;
          idx_best_R = idx_RS;
        }
        idx_RS++; if (idx_RS == _keypoints_with_descriptors_right.size()) {break;}
      }
      //check if something was found
      if (dist_best < _maximum_matching_distance_triangulation) {

        //ds add triangulation map entry
        try {

          const Index& row       = _keypoints_with_descriptors_left[idx_L].row;
          const Index& col_left  = _keypoints_with_descriptors_left[idx_L].col;
          assert(_framepoints_in_image[row][col_left] == 0);

          //ds attempt the triangulation - might throw on failure
          const PointCoordinates camera_coordinates(getCoordinatesInCameraLeft(_keypoints_with_descriptors_left[idx_L].keypoint.pt, _keypoints_with_descriptors_right[idx_best_R].keypoint.pt));

          //ds add to framepoint map
          _framepoints_in_image[row][col_left] = frame_->create(_keypoints_with_descriptors_left[idx_L].keypoint,
                                                                _keypoints_with_descriptors_left[idx_L].descriptor,
                                                                _keypoints_with_descriptors_right[idx_best_R].keypoint,
                                                                _keypoints_with_descriptors_right[idx_best_R].descriptor,
                                                                camera_coordinates);
          ++_number_of_available_points;

          //ds reduce search space
          idx_R = idx_best_R+1;
        } catch (const ExceptionTriangulation& exception) {}
      }
    }
  }

  //ds adjusts the detector and matching thresholds to maintain constant detection (called within compute)
  void StereoTriangulator::calibrateDetectionThresholds() {

    //ds check if there's a significant loss in target points
    if (_number_of_available_points < _target_number_of_points) {

      //ds lower detector threshold if possible to get more points
      if (_detector_threshold > _detector_threshold_minimum) {
        _detector_threshold -= 5;

#if CV_MAJOR_VERSION == 2
        _keypoint_detector->setInt("threshold", _detector_threshold);
#elif CV_MAJOR_VERSION == 3
        _keypoint_detector->setThreshold(_detector_threshold);
#else
#error OpenCV version not supported
#endif
      }

      //ds increase matching threshold if possible to get more matches
      if (_matching_distance_tracking_threshold < _matching_distance_tracking_threshold_maximum) {
        _matching_distance_tracking_threshold += 1;
      }
    }

    //ds of if there is a overflow of points
    else if (_number_of_available_points > _target_number_of_points) {

      //ds raise detector threshold if possible to get less points
      if (_detector_threshold < _detector_threshold_maximum) {
        _detector_threshold += 5;
#if CV_MAJOR_VERSION == 2
        _keypoint_detector->setInt("threshold", _detector_threshold);
#elif CV_MAJOR_VERSION == 3
        _keypoint_detector->setThreshold(_detector_threshold);
#else
#error OpenCV version not supported
#endif
      }

      //ds decrease matching threshold if possible to get less matches
      if (_matching_distance_tracking_threshold > _matching_distance_tracking_threshold_minimum) {
        _matching_distance_tracking_threshold -= 1;
      }
    }
  }

  //ds computes 3D position of a stereo keypoint pair in the keft camera frame (called within findStereoKeypoints)
  const PointCoordinates StereoTriangulator::getCoordinatesInCameraLeft(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) const {

    //ds check for minimal disparity
    if (image_coordinates_left_.x-image_coordinates_right_.x < _minimum_disparity_pixels) {
      throw ExceptionTriangulation("disparity value to low");
    }

    //ds input validation
    assert(image_coordinates_right_.x < image_coordinates_left_.x);
    assert(image_coordinates_right_.y == image_coordinates_left_.y);

    //ds first compute depth (z in camera)
    const real depth_meters = _baseline_pixelsmeters/(image_coordinates_right_.x-image_coordinates_left_.x);
    assert(depth_meters >= 0);
    const real depth_meters_per_pixel = depth_meters/_focal_length_pixels;

    //ds set 3d point
    const PointCoordinates coordinates_in_camera(depth_meters_per_pixel*(image_coordinates_left_.x-_principal_point_offset_u_pixels),
                                                 depth_meters_per_pixel*(image_coordinates_left_.y-_principal_point_offset_v_pixels),
                                                 depth_meters);

    //ds return triangulated point
    return coordinates_in_camera;
  }

  void StereoTriangulator::clearFramepointsInImage() {
    for (Index row = 0; row < _number_of_rows_image; ++row) {
      for (Count col = 0; col < _number_of_cols_image; ++col) {
        _framepoints_in_image[row][col] = 0;
      }
    }
  }

  void StereoTriangulator::setDetectorThreshold(const int32_t& detector_threshold_) {
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
