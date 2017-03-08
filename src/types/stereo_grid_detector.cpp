#include "stereo_grid_detector.h"

#include <fstream>

namespace proslam {

  real StereoGridDetector::maximum_depth_close   = 0;
  real StereoGridDetector::maximum_depth_far     = 0;

  StereoGridDetector::StereoGridDetector(const Camera* camera_left_,
                                         const Camera* camera_right_): _number_of_rows_image(camera_left_->imageRows()),
                                                                       _number_of_cols_image(camera_left_->imageCols()),
                                                                       _number_of_bins_u(std::floor(static_cast<real>(_number_of_cols_image)/_bin_size)),
                                                                       _number_of_bins_v(std::floor(static_cast<real>(_number_of_rows_image)/_bin_size)),
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
        _triangulation_map[row][col].is_available = false;
      }
    }

    _keypoints_left.clear();
    _keypoints_right.clear();
    _keypoints_with_descriptor_left.clear();
    _keypoints_with_descriptor_right.clear();
    _stereo_keypoints.clear();

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

    //ds buffer images
    const cv::Mat& intensity_image_left  = frame_->intensityImage();
    const cv::Mat& intensity_image_right = frame_->intensityImageExtra();

    //ds detect new features
    CHRONOMETER_START(feature_detection)
    _feature_detector->detect(intensity_image_left, _keypoints_left);
    _feature_detector->detect(intensity_image_right, _keypoints_right);
    CHRONOMETER_STOP(feature_detection)

    //ds keypoint pruning - prune only left and keep all potential epipolar matches on right
    CHRONOMETER_START(keypoint_pruning)
    _binKeypoints(_keypoints_left, _bin_map_left);
//    _binKeypoints(keypoints_right, _bin_map_right);
    CHRONOMETER_STOP(keypoint_pruning)

    //ds extract descriptors for detected features
    CHRONOMETER_START(descriptor_extraction)
    _descriptor_extractor->compute(intensity_image_left, _keypoints_left, _descriptors_left);
    _descriptor_extractor->compute(intensity_image_right, _keypoints_right, _descriptors_right);
    CHRONOMETER_STOP(descriptor_extraction)

    //ds detector-driven TRIANGULATION: reset maps
    CHRONOMETER_START(point_triangulation)
    for (uint32_t row = 0; row < _number_of_rows_image; ++row) {
      for (uint32_t col = 0; col < _number_of_cols_image; ++col) {
//        _keypoint_index_map_left[row][col]  = -1;
//        _keypoint_index_map_right[row][col] = -1;
        _triangulation_map[row][col].is_available = false;
      }
    }

//    //ds set keypoint maps
//    for (int32_t index_keypoint = 0; index_keypoint < static_cast<int32_t>(_keypoints_left.size()); ++index_keypoint) {
//      const cv::KeyPoint& keypoint_left = _keypoints_left[index_keypoint];
//      const uint32_t row = static_cast<uint32_t>(keypoint_left.pt.y);
//      const uint32_t col = static_cast<uint32_t>(keypoint_left.pt.x);
//      _keypoint_index_map_left[row][col] = index_keypoint;
//    }
//    for (int32_t index_keypoint = 0; index_keypoint < static_cast<int32_t>(_keypoints_right.size()); ++index_keypoint) {
//      const cv::KeyPoint& keypoint_right = _keypoints_right[index_keypoint];
//      const uint32_t row = static_cast<uint32_t>(keypoint_right.pt.y);
//      const uint32_t col = static_cast<uint32_t>(keypoint_right.pt.x);
//      _keypoint_index_map_right[row][col] = index_keypoint;
//    }

    //ds fuse keypoints buffers
    _keypoints_with_descriptor_left.resize(_keypoints_left.size());
    _keypoints_with_descriptor_right.resize(_keypoints_right.size());
    if (_keypoints_left.size() <= _keypoints_right.size()) {
      for (Index u = 0; u < _keypoints_left.size(); ++u) {
        _keypoints_with_descriptor_left[u].keypoint    = _keypoints_left[u];
        _keypoints_with_descriptor_left[u].descriptor  = _descriptors_left.row(u);
        _keypoints_with_descriptor_left[u].r           = _keypoints_left[u].pt.y;
        _keypoints_with_descriptor_left[u].c           = _keypoints_left[u].pt.x;
        _keypoints_with_descriptor_right[u].keypoint   = _keypoints_right[u];
        _keypoints_with_descriptor_right[u].descriptor = _descriptors_right.row(u);
        _keypoints_with_descriptor_right[u].r          = _keypoints_right[u].pt.y;
        _keypoints_with_descriptor_right[u].c          = _keypoints_right[u].pt.x;
      }
      for (Index u = _keypoints_left.size(); u < _keypoints_right.size(); ++u) {
        _keypoints_with_descriptor_right[u].keypoint   = _keypoints_right[u];
        _keypoints_with_descriptor_right[u].descriptor = _descriptors_right.row(u);
        _keypoints_with_descriptor_right[u].r          = _keypoints_right[u].pt.y;
        _keypoints_with_descriptor_right[u].c          = _keypoints_right[u].pt.x;
      }
      _stereo_keypoints.resize(_keypoints_left.size());
    } else {
      for (Index u = 0; u < _keypoints_right.size(); ++u) {
        _keypoints_with_descriptor_left[u].keypoint    = _keypoints_left[u];
        _keypoints_with_descriptor_left[u].descriptor  = _descriptors_left.row(u);
        _keypoints_with_descriptor_left[u].r           = _keypoints_left[u].pt.y;
        _keypoints_with_descriptor_left[u].c           = _keypoints_left[u].pt.x;
        _keypoints_with_descriptor_right[u].keypoint   = _keypoints_right[u];
        _keypoints_with_descriptor_right[u].descriptor = _descriptors_left.row(u);
      }
      for (Index u = _keypoints_right.size(); u < _keypoints_left.size(); ++u) {
        _keypoints_with_descriptor_left[u].keypoint   = _keypoints_left[u];
        _keypoints_with_descriptor_left[u].descriptor = _descriptors_left.row(u);
        _keypoints_with_descriptor_left[u].r          = _keypoints_left[u].pt.y;
        _keypoints_with_descriptor_left[u].c          = _keypoints_left[u].pt.x;
      }
      _stereo_keypoints.resize(_keypoints_right.size());
    }

//    std::ofstream out("new.txt", std::ofstream::app);

    //ds perform stereo keypoint search algorithm
    //ds sort all input vectors in the order of the expression
    std::sort(_keypoints_with_descriptor_left.begin(), _keypoints_with_descriptor_left.end(), [](const KeypointWithDescriptor& a_, const KeypointWithDescriptor& b_){return ((a_.r < b_.r) || (a_.r == b_.r && a_.c < b_.c));});
    std::sort(_keypoints_with_descriptor_right.begin(), _keypoints_with_descriptor_right.end(), [](const KeypointWithDescriptor& a_, const KeypointWithDescriptor& b_){return ((a_.r < b_.r) || (a_.r == b_.r && a_.c < b_.c));});

    //ds running variable
    Index idx_R = 0;

    //ds loop over all left keypoints
    Count number_of_potential_points = 0;
    for (Index idx_L = 0; idx_L < _keypoints_with_descriptor_left.size(); idx_L++) {
      //stop condition
      if (idx_R == _keypoints_with_descriptor_right.size()) {break;}
      //the right keypoints are on an lower row - skip left
      while (_keypoints_with_descriptor_left[idx_L].r < _keypoints_with_descriptor_right[idx_R].r) {
        idx_L++; if (idx_L == _keypoints_with_descriptor_left.size()) {break;}
      }
      if (idx_L == _keypoints_with_descriptor_left.size()) {break;}
      //the right keypoints are on an upper row - skip right
      while (_keypoints_with_descriptor_left[idx_L].r > _keypoints_with_descriptor_right[idx_R].r) {
        idx_R++; if (idx_R == _keypoints_with_descriptor_right.size()) {break;}
      }
      if (idx_R == _keypoints_with_descriptor_right.size()) {break;}
      //search bookkeeping
      Index idx_RS = idx_R;
      real dist_best = _maximum_matching_distance_triangulation;
      Index idx_best_R = 0;
      //scan epipolar line for current keypoint at idx_L
      while (_keypoints_with_descriptor_left[idx_L].r == _keypoints_with_descriptor_right[idx_RS].r) {
        //zero disparity stop condition
        if (_keypoints_with_descriptor_right[idx_RS].c >= _keypoints_with_descriptor_left[idx_L].c) {break;}
        //compute descriptor distance
        const real dist = cv::norm(_keypoints_with_descriptor_left[idx_L].descriptor, _keypoints_with_descriptor_right[idx_RS].descriptor, DESCRIPTOR_NORM);
        if(dist < dist_best) {
          dist_best = dist;
          idx_best_R = idx_RS;
        }
        idx_RS++; if (idx_RS == _keypoints_with_descriptor_right.size()) {break;}
      }
      //check if something was found
      if (dist_best < _maximum_matching_distance_triangulation) {

        //ds add triangulation map entry
        try {

          const Index& row       = _keypoints_with_descriptor_left[idx_L].r;
          const Index& col_left  = _keypoints_with_descriptor_left[idx_L].c;
          assert(!_triangulation_map[row][col_left].is_available);

          //ds directly attempt the triangulation - might throw
          const PointCoordinates camera_coordinates(getCoordinatesInCamera(_keypoints_with_descriptor_left[idx_L].keypoint.pt,
                                                                           _keypoints_with_descriptor_right[idx_best_R].keypoint.pt));

//          out << "[" << number_of_potential_points << "]"
//              << _keypoints_with_descriptor_left[idx_L].keypoint.pt << "|"
//              << _keypoints_with_descriptor_right[idx_best_R].keypoint.pt
//              << ">" << camera_coordinates.transpose() << std::endl;

          //ds set descriptor map
          _triangulation_map[row][col_left].camera_coordinates_left = camera_coordinates;
          _triangulation_map[row][col_left].keypoint_left    = _keypoints_with_descriptor_left[idx_L].keypoint;
          _triangulation_map[row][col_left].keypoint_right   = _keypoints_with_descriptor_right[idx_best_R].keypoint;
          _triangulation_map[row][col_left].descriptor_left  = _keypoints_with_descriptor_left[idx_L].descriptor;
          _triangulation_map[row][col_left].descriptor_right = _keypoints_with_descriptor_right[idx_best_R].descriptor;
          _triangulation_map[row][col_left].is_available     = true;
          ++number_of_potential_points;

          //ds reduce search space
          idx_R = idx_best_R+1;
        } catch (const ExceptionTriangulation& exception) {}
      }
    }
//    std::cerr << number_of_potential_points << "/" << _keypoints_with_descriptor_left.size() << std::endl;



//    //ds look for triangulation pairs - row wise block optimization
//    Count number_of_potential_points = 0;
//    for (uint32_t row = 0; row < _number_of_rows_image; ++row) {
//
//      //ds last found on right in this column - blocking the search range to the left
//      int32_t col_right_last = 0;
//
//      //ds always starting from the left
//      for (uint32_t col_left = 0; col_left < _number_of_cols_image; ++col_left) {
//
//        //ds if we have a keypoint at this location
//        if (_keypoint_index_map_left[row][col_left] != -1) {
//
//          //ds exhaustive search
//          real matching_distance_best = _maximum_matching_distance_triangulation;
//          int32_t col_right_best         = -1;
//
//          //ds check the all potentially matching stereo keypoints in range
//          for (int32_t col_right = col_left; col_right > col_right_last; --col_right) {
//
//            //ds if we have a keypoint at this location
//            if (_keypoint_index_map_right[row][col_right] != -1) {
//
//              //ds compute the distance
//              const real matching_distance = cv::norm(_descriptors_left.row(_keypoint_index_map_left[row][col_left]),
//                                                      _descriptors_right.row(_keypoint_index_map_right[row][col_right]),
//                                                      DESCRIPTOR_NORM);
//
//              if (matching_distance < matching_distance_best) {
//                matching_distance_best = matching_distance;
//                col_right_best = col_right;
//              }
//            }
//          }
//
//          //ds if we found a match
//          if (matching_distance_best < _maximum_matching_distance_triangulation) {
//
//            try {
//
//              //ds directly attempt the triangulation - might throw
//              const PointCoordinates camera_coordinates(getCoordinatesInCamera(_keypoints_left[_keypoint_index_map_left[row][col_left]].pt,
//                                                                               _keypoints_right[_keypoint_index_map_right[row][col_right_best]].pt));
//
//              out << "[" << number_of_potential_points << "]"
//                  << _keypoints_left[_keypoint_index_map_left[row][col_left]].pt << "|"
//                  << _keypoints_right[_keypoint_index_map_right[row][col_right_best]].pt
//                  << ">" << camera_coordinates.transpose() << std::endl;
//
//              //ds set descriptor map
//              _triangulation_map[row][col_left].camera_coordinates_left = camera_coordinates;
//              _triangulation_map[row][col_left].keypoint_left    = _keypoints_left[_keypoint_index_map_left[row][col_left]];
//              _triangulation_map[row][col_left].keypoint_right   = _keypoints_right[_keypoint_index_map_right[row][col_right_best]];
//              _triangulation_map[row][col_left].descriptor_left  = _descriptors_left.row(_keypoint_index_map_left[row][col_left]);
//              _triangulation_map[row][col_left].descriptor_right = _descriptors_right.row(_keypoint_index_map_right[row][col_right_best]);
//              _triangulation_map[row][col_left].is_available     = true;
//              ++number_of_potential_points;
//
//              //ds disable further matching and reduce search time
//              col_right_last = col_right_best;
//            } catch (const ExceptionTriangulation& exception) {}
//          }
//        }
//      }
//    }
//    std::cerr << number_of_potential_points << "/" << _keypoints_left.size() << std::endl;
    CHRONOMETER_STOP(point_triangulation)

//    out << "-----------------------------------------------------" << std::endl;
//    out.close();
//    getchar();

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

//      //ds increase matching threshold if possible to get more matches
//      if (_maximum_tracking_matching_distance < _tracking_matching_distance_threshold_maximum) {
//        _maximum_tracking_matching_distance += 5;
//      }
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

//      //ds decrease matching threshold if possible to get less matches
//      if (_maximum_tracking_matching_distance > _tracking_matching_distance_threshold_minimum) {
//        _maximum_tracking_matching_distance -= 5;
//      }
    }

    return number_of_potential_points;
  }

  const PointCoordinates StereoGridDetector::getCoordinatesInCamera(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) {

    //ds check for minimal disparity
    if (image_coordinates_left_.x-image_coordinates_right_.x < _minimum_disparity) {
      throw ExceptionTriangulation("disparity value to low");
    }

    //ds input validation
    assert(image_coordinates_right_.x < image_coordinates_left_.x);
    assert(image_coordinates_right_.y == image_coordinates_left_.y);

    //ds first compute depth (z in camera)
    const real depth_meters = _triangulation_DuR_flipped/(image_coordinates_left_.x-image_coordinates_right_.x);
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
}
