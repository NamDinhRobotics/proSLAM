#include "stereo_grid_detector.h"

namespace gslam {

  gt_real StereoGridDetector::maximum_depth_close   = 0;
  gt_real StereoGridDetector::maximum_depth_far     = 0;

  StereoGridDetector::StereoGridDetector(const Camera* camera_left_,
                                         const Camera* camera_right_): _number_of_rows_image(camera_left_->imageRows()),
                                                                       _number_of_cols_image(camera_left_->imageCols()),
                                                                       _number_of_bins_u(std::floor(static_cast<gt_real>(_number_of_cols_image)/_bin_size)),
                                                                       _number_of_bins_v(std::floor(static_cast<gt_real>(_number_of_rows_image)/_bin_size)),
                                                                       _bin_map_left(0),
//                                                                       _bin_map_right(0),
                                                                       _triangulation_F(camera_left_->projectionMatrix()(0,0)),
                                                                       _triangulation_Finverse(1/_triangulation_F),
                                                                       _triangulation_Pu(camera_left_->projectionMatrix()(0,2)),
                                                                       _triangulation_Pv(camera_left_->projectionMatrix()(1,2)),
                                                                       _triangulation_DuR(camera_right_->projectionMatrix()(0,3)),
                                                                       _triangulation_DuR_flipped(-_triangulation_DuR),
                                                                       _baseline_meters(_triangulation_DuR_flipped/_triangulation_F),
#if CV_MAJOR_VERSION == 2
                                                                       _feature_detector(std::make_shared<cv::FastFeatureDetector>(_detector_threshold)),
                                                                       _descriptor_extractor(std::make_shared<cv::BriefDescriptorExtractor>(DESCRIPTOR_SIZE_BYTES)) {
#elif CV_MAJOR_VERSION == 3
                                                                       _feature_detector(cv::FastFeatureDetector::create(_detector_threshold)),
                                                                       _descriptor_extractor(cv::xfeatures2d::BriefDescriptorExtractor::create(DESCRIPTOR_SIZE_BYTES)) {
#else
  #error OpenCV version not supported
#endif
    std::cerr << "StereoGridDetector::StereoGridDetector|constructing" << std::endl;
    assert(camera_left_->imageCols() == camera_right_->imageCols());
    assert(camera_left_->imageRows() == camera_right_->imageRows());
    maximum_depth_close = _baseline_factor*_baseline_meters;
    maximum_depth_far   = _triangulation_DuR_flipped/_minimum_disparity;

    //ds allocate dynamic datastructures: simple maps
    _keypoint_index_map_left  = new int32_t*[_number_of_rows_image];
    _keypoint_index_map_right = new int32_t*[_number_of_rows_image];
    _triangulation_map        = new TriangulatedPoint*[_number_of_rows_image];
    for (Index row = 0; row < _number_of_rows_image; ++row) {
      _keypoint_index_map_left[row]  = new int32_t[_number_of_cols_image];
      _keypoint_index_map_right[row] = new int32_t[_number_of_cols_image];
      _triangulation_map[row]        = new TriangulatedPoint[_number_of_cols_image];
    }

    //ds allocate dynamic datastructures: bin grid
    _bin_map_left  = new cv::KeyPoint*[_number_of_bins_v];
    for (Count v = 0; v < _number_of_bins_v; ++v) {
      _bin_map_left[v]  = new cv::KeyPoint[_number_of_bins_u];
      for (Count u = 0; u < _number_of_bins_u; ++u) {
        _bin_map_left[v][u].response  = 0;
      }
    }

    //ds initialize data structures
    for (Count row = 0; row < _number_of_rows_image; ++row) {
      for (Count col = 0; col < _number_of_cols_image; ++col) {
        _keypoint_index_map_left[row][col]  = -1;
        _keypoint_index_map_right[row][col] = -1;
        _triangulation_map[row][col].is_set       = false;
        _triangulation_map[row][col].is_available = false;
      }
    }

    std::cerr << "StereoGridDetector::StereoGridDetector|baseline (m): " << _baseline_meters << std::endl;
    std::cerr << "StereoGridDetector::StereoGridDetector|maximum depth tracking close (m): " << maximum_depth_close << std::endl;
    std::cerr << "StereoGridDetector::StereoGridDetector|maximum depth tracking far (m): " << maximum_depth_far << std::endl;
    std::cerr << "StereoGridDetector::StereoGridDetector|bin sixe (pixel): " << _bin_size << std::endl;
    std::cerr << "StereoGridDetector::StereoGridDetector|number of bins u: " << _number_of_bins_u << std::endl;
    std::cerr << "StereoGridDetector::StereoGridDetector|number of bins v: " << _number_of_bins_v << std::endl;
    std::cerr << "StereoGridDetector::StereoGridDetector|total number of bins: " << _number_of_bins_u*_number_of_bins_v << std::endl;
    std::cerr << "StereoGridDetector::StereoGridDetector|constructed" << std::endl;
  }

  StereoGridDetector::~StereoGridDetector() {
    std::cerr << "StereoGridDetector::StereoGridDetector|destroying" << std::endl;

    //ds deallocate dynamic datastructures
    for (Index row = 0; row < _number_of_rows_image; ++row) {
      delete[] _keypoint_index_map_left[row];
      delete[] _keypoint_index_map_right[row];
      delete[] _triangulation_map[row];
    }
    delete[] _keypoint_index_map_left;
    delete[] _keypoint_index_map_right;
    delete[] _triangulation_map;
    for (Count v = 0; v < _number_of_bins_v; ++v) {
      delete[] _bin_map_left[v];
    }
    delete[] _bin_map_left;

    std::cerr << "StereoGridDetector::StereoGridDetector|destroyed" << std::endl;
  }

  const Count StereoGridDetector::triangulate(const Frame* frame_) {

    //ds buffer locally (avoid memory bugs)
    cv::Mat descriptors_left_current;
    cv::Mat descriptors_right_current;

    //ds buffer images
    const cv::Mat& intensity_image_left  = frame_->intensityImage();
    const cv::Mat& intensity_image_right = frame_->intensityImageExtra();

    //ds detect new features
    CHRONOMETER_START(feature_detection)
    std::vector<cv::KeyPoint> keypoints_left;
    _feature_detector->detect(intensity_image_left, keypoints_left);
    std::vector<cv::KeyPoint> keypoints_right;
    _feature_detector->detect(intensity_image_right, keypoints_right);
    CHRONOMETER_STOP(feature_detection)

    //ds keypoint pruning - prune only left and keep all potential epipolar matches on right
    CHRONOMETER_START(keypoint_pruning)
    _binKeypoints(keypoints_left, _bin_map_left);
//    _binKeypoints(keypoints_right, _bin_map_right);
    CHRONOMETER_STOP(keypoint_pruning)

    //ds extract descriptors for detected features
    CHRONOMETER_START(descriptor_extraction)
    _descriptor_extractor->compute(intensity_image_left, keypoints_left, descriptors_left_current);
    _descriptor_extractor->compute(intensity_image_right, keypoints_right, descriptors_right_current);
    CHRONOMETER_STOP(descriptor_extraction)

    //ds detector-driven TRIANGULATION: reset maps
    CHRONOMETER_START(point_triangulation)
    for (uint32_t row = 0; row < _number_of_rows_image; ++row) {
      for (uint32_t col = 0; col < _number_of_cols_image; ++col) {
        _keypoint_index_map_left[row][col]  = -1;
        _keypoint_index_map_right[row][col] = -1;
        _triangulation_map[row][col].is_set       = false;
        _triangulation_map[row][col].is_available = false;
      }
    }

    //ds set keypoint maps
    for (int32_t index_keypoint = 0; index_keypoint < static_cast<int32_t>(keypoints_left.size()); ++index_keypoint) {
      const cv::KeyPoint& keypoint_left = keypoints_left[index_keypoint];
      const uint32_t row = static_cast<uint32_t>(keypoint_left.pt.y);
      const uint32_t col = static_cast<uint32_t>(keypoint_left.pt.x);
      _keypoint_index_map_left[row][col] = index_keypoint;
    }
    for (int32_t index_keypoint = 0; index_keypoint < static_cast<int32_t>(keypoints_right.size()); ++index_keypoint) {
      const cv::KeyPoint& keypoint_right = keypoints_right[index_keypoint];
      const uint32_t row = static_cast<uint32_t>(keypoint_right.pt.y);
      const uint32_t col = static_cast<uint32_t>(keypoint_right.pt.x);
      _keypoint_index_map_right[row][col] = index_keypoint;
    }

    //ds look for triangulation pairs - row wise block optimization
    Count number_of_potential_points = 0;
    for (uint32_t row = 0; row < _number_of_rows_image; ++row) {

      //ds last found on right in this column - blocking the search range to the left
      int32_t col_right_last = 0;

      //ds always starting from the left
      for (uint32_t col_left = 0; col_left < _number_of_cols_image; ++col_left) {

        //ds if we have a keypoint at this location
        if (_keypoint_index_map_left[row][col_left] != -1) {

          //ds exhaustive search
          gt_real matching_distance_best = _maximum_matching_distance_triangulation;
          int32_t col_right_best         = -1;

          //ds check the all potentially matching stereo keypoints in range
          for (int32_t col_right = col_left; col_right > col_right_last; --col_right) {

            //ds if we have a keypoint at this location
            if (_keypoint_index_map_right[row][col_right] != -1) {

              //ds compute the distance
              const gt_real matching_distance = cv::norm(descriptors_left_current.row(_keypoint_index_map_left[row][col_left]),
                                                         descriptors_right_current.row(_keypoint_index_map_right[row][col_right]),
                                                         DESCRIPTOR_NORM);

              if (matching_distance < matching_distance_best) {
                matching_distance_best = matching_distance;
                col_right_best = col_right;
              }
            }
          }

          //ds if we found a match
          if (matching_distance_best < _maximum_matching_distance_triangulation) {

            try {

              //ds directly attempt the triangulation - might throw
              const PointCoordinates camera_coordinates(getCoordinatesInCamera(keypoints_left[_keypoint_index_map_left[row][col_left]].pt,
                                                                               keypoints_right[_keypoint_index_map_right[row][col_right_best]].pt));

              //ds set descriptor map
              _triangulation_map[row][col_left].camera_coordinates_left = camera_coordinates;
              _triangulation_map[row][col_left].keypoint_left    = keypoints_left[_keypoint_index_map_left[row][col_left]];
              _triangulation_map[row][col_left].keypoint_right   = keypoints_right[_keypoint_index_map_right[row][col_right_best]];
              _triangulation_map[row][col_left].descriptor_left  = descriptors_left_current.row(_keypoint_index_map_left[row][col_left]);
              _triangulation_map[row][col_left].descriptor_right = descriptors_right_current.row(_keypoint_index_map_right[row][col_right_best]);
              _triangulation_map[row][col_left].is_set           = true;
              _triangulation_map[row][col_left].is_available     = true;
              ++number_of_potential_points;

              //ds disable further matching and reduce search time
              col_right_last = col_right_best;
            } catch (const std::runtime_error& exception) {}
          }
        }
      }
    }
    CHRONOMETER_STOP(point_triangulation)

    //ds check if there's a significant loss in target points
    if (number_of_potential_points < _target_number_of_points) {

      //ds lower detector threshold if possible to get more points
      if (_detector_threshold > _detector_threshold_minimum) {
        _detector_threshold -= 5;

#if CV_MAJOR_VERSION == 2
        _feature_detector = std::make_shared<cv::FastFeatureDetector>(_detector_threshold);
#elif CV_MAJOR_VERSION == 3
        _feature_detector = cv::FastFeatureDetector::create(_detector_threshold);
#else
  #error OpenCV version not supported
#endif
      }

      //ds increase matching threshold if possible to get more matches
      if (_maximum_tracking_matching_distance < _tracking_matching_distance_threshold_maximum) {
        _maximum_tracking_matching_distance += 1;
      }
    }

    //ds of if there is a overflow of points
    else if (number_of_potential_points > _target_number_of_points) {

      //ds raise detector threshold if possible to get less points
      if (_detector_threshold < _detector_threshold_maximum) {
        _detector_threshold += 5;
#if CV_MAJOR_VERSION == 2
        _feature_detector = std::make_shared<cv::FastFeatureDetector>(_detector_threshold);
#elif CV_MAJOR_VERSION == 3
        _feature_detector = cv::FastFeatureDetector::create(_detector_threshold);
#else
  #error OpenCV version not supported
#endif
      }

      //ds decrease matching threshold if possible to get less matches
      if (_maximum_tracking_matching_distance > _tracking_matching_distance_threshold_minimum) {
        _maximum_tracking_matching_distance -= 1;
      }
    }

    return number_of_potential_points;
  }

  const PointCoordinates StereoGridDetector::getCoordinatesInCamera(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) {

    //ds check for minimal disparity
    if (image_coordinates_left_.x-image_coordinates_right_.x < _minimum_disparity) {
      throw std::runtime_error("disparity value to low");
    }

    //ds input validation
    assert(image_coordinates_right_.x < image_coordinates_left_.x);
    assert(image_coordinates_right_.y == image_coordinates_left_.y);

    //ds first compute depth (z in camera)
    const gt_real depth_meters = _triangulation_DuR_flipped/(image_coordinates_left_.x-image_coordinates_right_.x);
    assert(depth_meters >= 0);

    //ds set 3d point
    const PointCoordinates coordinates_in_camera(_triangulation_Finverse*depth_meters*(image_coordinates_left_.x-_triangulation_Pu),
                                                 _triangulation_Finverse*depth_meters*(image_coordinates_left_.y-_triangulation_Pv),
                                                 depth_meters);

    //ds return triangulated point
    return coordinates_in_camera;
  }

  void StereoGridDetector::_binKeypoints(std::vector<cv::KeyPoint>& keypoints_, cv::KeyPoint** bin_map_) {

    //ds sort by position in u
    std::sort(keypoints_.begin(), keypoints_.end(), [](const cv::KeyPoint& a_, const cv::KeyPoint& b_) {return a_.pt.x < b_.pt.x;});

    //ds check all keypoints for this grid
    Count u_current = 0;
    for (const cv::KeyPoint& keypoint: keypoints_) {
      const gt_real& keypoint_u = keypoint.pt.x;
      const gt_real& keypoint_v = keypoint.pt.y;

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
}
