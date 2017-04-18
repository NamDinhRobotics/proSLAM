#include "base_framepoint_generator.h"

namespace proslam {

  //ds the stereo camera setup must be provided
  BaseFramePointGenerator::BaseFramePointGenerator(): _camera_left(0),
                                                      _number_of_rows_image(0),
                                                      _number_of_cols_image(0),
                                                      _number_of_bins_u(0),
                                                      _number_of_bins_v(0),
                                                      _bin_map_left(0),
                                                      _focal_length_pixels(0),
                                                      _principal_point_offset_u_pixels(0),
                                                      _principal_point_offset_v_pixels(0),
                                                      _maximum_depth_near_meters(0),
                                                      _maximum_depth_far_meters(0),
                                                      _framepoints_in_image(0) {}

  void  BaseFramePointGenerator::setup(){
    assert(_camera_left);
    _number_of_rows_image = _camera_left->imageRows();
    _number_of_cols_image = _camera_left->imageCols();
    _number_of_bins_u = std::floor(static_cast<real>(_number_of_cols_image)/_bin_size);
    _number_of_bins_v = std::floor(static_cast<real>(_number_of_rows_image)/_bin_size);
    _focal_length_pixels = _camera_left->cameraMatrix()(0,0);
    _principal_point_offset_u_pixels = _camera_left->cameraMatrix()(0,2);
    _principal_point_offset_v_pixels = _camera_left->cameraMatrix()(1,2);
    _maximum_depth_near_meters = 6;
    _maximum_depth_far_meters = 0.6;
    
#if CV_MAJOR_VERSION == 2
    _keypoint_detector = new cv::FastFeatureDetector(_detector_threshold);
    _descriptor_extractor = new cv::BriefDescriptorExtractor(DESCRIPTOR_SIZE_BYTES);
#elif CV_MAJOR_VERSION == 3
    _keypoint_detector = cv::FastFeatureDetector::create(_detector_threshold);
    _descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(DESCRIPTOR_SIZE_BYTES);
#else
#error OpenCV version not supported
#endif

    std::cerr << "BaseFramePointGenerator::BaseFramePointGenerator|constructing" << std::endl;

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
    _keypoints_with_descriptors_left.clear();
    std::cerr << "BaseFramePointGenerator::BaseFramePointGenerator|constructed" << std::endl;
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
    for (Count v = 0; v < _number_of_bins_v; ++v) {
      delete[] _bin_map_left[v];
    }
    delete[] _bin_map_left;

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
    _keypoint_detector->detect(intensity_image_, keypoints_);
    CHRONOMETER_STOP(feature_detection)
  }

  //ds regularizes the detected keypoints using binning (called within compute)
  void BaseFramePointGenerator::binKeypoints(std::vector<cv::KeyPoint>& keypoints_, cv::KeyPoint** bin_map_) {
    CHRONOMETER_START(keypoint_pruning)

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
    CHRONOMETER_STOP(keypoint_pruning)
  }

  //ds extracts the defined descriptors for the given keypoints (called within compute)
  void BaseFramePointGenerator::extractDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_) {
    CHRONOMETER_START(descriptor_extraction)
    _descriptor_extractor->compute(intensity_image_, keypoints_, descriptors_);
    CHRONOMETER_STOP(descriptor_extraction)
  }

  //ds adjusts the detector and matching thresholds to maintain constant detection (called within compute)
  void BaseFramePointGenerator::calibrateDetectionThresholds() {

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
