#include "stereo_framepoint_generator.h"

namespace proslam {

StereoFramePointGenerator::StereoFramePointGenerator(StereoFramePointGeneratorParameters* parameters_): BaseFramePointGenerator(parameters_),
                                                                                                        _parameters(parameters_) {
  _triangulation_success_ratios.clear();
  _epipolar_search_offsets_pixel.clear();
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::StereoFramePointGenerator|constructed" << std::endl)
}

//ds the stereo camera setup must be provided
void StereoFramePointGenerator::configure(){
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::configure|configuring" << std::endl)
  assert(_camera_right);

  //ds integrate configuration
  _parameters->number_of_cameras = 2;
  BaseFramePointGenerator::configure();
  _triangulation_success_ratios.clear();

  //ds set initial tracking distance (changes at runtime)
  _projection_tracking_distance_pixels = _parameters->maximum_projection_tracking_distance_pixels;

  //ds configure stereo triangulation parameters
  _baseline              = _camera_right->baselineHomogeneous();
  _baseline_pixelsmeters = _baseline(0);
  _baseline_meters       = -_baseline_pixelsmeters/_focal_length_pixels;
  if (_baseline_meters <= 0) {
    throw std::runtime_error("StereoFramePointGenerator::_updateRigidStereoCameraIntrinsics|invalid baseline (m): "
                             "'" + std::to_string(_baseline_meters) + "' verify intrinsic camera parameters");
  }

  _f_x = _camera_right->cameraMatrix()(0,0);
  _f_y = _camera_right->cameraMatrix()(1,1);
  _c_x = _camera_right->cameraMatrix()(0,2);
  _c_y = _camera_right->cameraMatrix()(1,2);
  _b_x = _baseline_pixelsmeters;

  //ds initialize feature matcher
  _feature_matcher_right.configure(_number_of_rows_image, _number_of_cols_image);

  //ds configure epipolar search ranges (minimum 0)
  _epipolar_search_offsets_pixel.push_back(0);
  for (uint32_t u = 1; u <= _parameters->maximum_epipolar_search_offset_pixels; ++u) {
    _epipolar_search_offsets_pixel.push_back(u);
    _epipolar_search_offsets_pixel.push_back(-u);
  }

  //ds clear buffers
  _keypoints_with_descriptors_left.clear();
  _keypoints_with_descriptors_right.clear();

  //ds info
  LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|baseline (m): " << _baseline_meters << std::endl)
  LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|number of epipolar lines considered for stereo matching: " << _epipolar_search_offsets_pixel.size() << std::endl)
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::configure|configured" << std::endl)
}

//ds cleanup of dynamic structures
StereoFramePointGenerator::~StereoFramePointGenerator() {
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::~StereoFramePointGenerator|destroying" << std::endl)
  _triangulation_success_ratios.clear();
  _epipolar_search_offsets_pixel.clear();
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::~StereoFramePointGenerator|destroyed" << std::endl)
}

//ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
void StereoFramePointGenerator::compute(Frame* frame_, Frame* frame_previous_) {

  //ds detect new features to generate frame points from (fixed thresholds)
  detectKeypoints(frame_->intensityImageLeft(), frame_->keypointsLeft());
  detectKeypoints(frame_->intensityImageRight(), frame_->keypointsRight());

  //ds adjust detector thresholds for next frame
  adjustDetectorThresholds();

  //ds overwrite with average
  _number_of_detected_keypoints = (frame_->keypointsLeft().size()+frame_->keypointsRight().size())/2.0;
  frame_->_number_of_detected_keypoints = _number_of_detected_keypoints;

  //ds extract descriptors for detected features
  computeDescriptors(frame_->intensityImageLeft(), frame_->keypointsLeft(), frame_->descriptorsLeft());
  computeDescriptors(frame_->intensityImageRight(), frame_->keypointsRight(), frame_->descriptorsRight());

//TODO refactor and activate
//  //ds initialize matchers for current measurements
//  _feature_matcher_left.setFeatures(frame_->keypointsLeft(), frame_->descriptorsLeft());
//  _feature_matcher_right.setFeatures(frame_->keypointsRight(), frame_->descriptorsRight());

//  //ds if a previous left frame is available track features in the left image before exhaustive stereo matching
//  //ds with or without odometry prior for feature projections
//  if (frame_previous_) {
//    const TransformMatrix3D camera_left_previous_in_current = frame_->worldToCameraLeft()*frame_previous_->cameraLeftToWorld();
//    _setFramepoints(frame_,
//                    frame_previous_,
//                    camera_left_previous_in_current);
//  }

  //ds prepare and execute stereo keypoint search
  CHRONOMETER_START(point_triangulation)
  initialize(frame_);
  findStereoKeypoints(frame_);
  CHRONOMETER_STOP(point_triangulation)
}

//ds initializes structures for the epipolar stereo keypoint search (called within compute)
void StereoFramePointGenerator::initialize(Frame* frame_) {

  //ds prepare keypoint with descriptors vectors for stereo keypoint search
  _keypoints_with_descriptors_left.resize(frame_->keypointsLeft().size());
  _keypoints_with_descriptors_right.resize(frame_->keypointsRight().size());

  //ds if we got more keypoints in the right image
  if (frame_->keypointsLeft().size() <= frame_->keypointsRight().size()) {

    //ds first add all left keypoints plus equally many from the right
    for (Index u = 0; u < frame_->keypointsLeft().size(); ++u) {
      _keypoints_with_descriptors_left[u].keypoint    = frame_->keypointsLeft()[u];
      _keypoints_with_descriptors_left[u].descriptor  = frame_->descriptorsLeft().row(u);
      _keypoints_with_descriptors_right[u].keypoint   = frame_->keypointsRight()[u];
      _keypoints_with_descriptors_right[u].descriptor = frame_->descriptorsRight().row(u);
    }

    //ds add the remaining points from the right image
    for (Index u = frame_->keypointsLeft().size(); u < frame_->keypointsRight().size(); ++u) {
      _keypoints_with_descriptors_right[u].keypoint   = frame_->keypointsRight()[u];
      _keypoints_with_descriptors_right[u].descriptor = frame_->descriptorsRight().row(u);
    }

  //ds if we got more keypoints in the left image
  } else {

    //ds first add all right keypoints plus equally many from the left
    for (Index u = 0; u < frame_->keypointsRight().size(); ++u) {
      _keypoints_with_descriptors_left[u].keypoint    = frame_->keypointsLeft()[u];
      _keypoints_with_descriptors_left[u].descriptor  = frame_->descriptorsLeft().row(u);
      _keypoints_with_descriptors_right[u].keypoint   = frame_->keypointsRight()[u];
      _keypoints_with_descriptors_right[u].descriptor = frame_->descriptorsRight().row(u);
    }

    //ds add the remaining points from the left image
    for (Index u = frame_->keypointsRight().size(); u < frame_->keypointsLeft().size(); ++u) {
      _keypoints_with_descriptors_left[u].keypoint   = frame_->keypointsLeft()[u];
      _keypoints_with_descriptors_left[u].descriptor = frame_->descriptorsLeft().row(u);
    }
  }
}

//ds computes all potential stereo keypoints (exhaustive in matching distance) and stores them as framepoints (called within compute)
void StereoFramePointGenerator::findStereoKeypoints(Frame* frame_) {

  //ds sort all input vectors by ascending row positions
  std::sort(_keypoints_with_descriptors_left.begin(), _keypoints_with_descriptors_left.end(),
            [](const IntensityFeature& a_, const IntensityFeature& b_){return ((a_.keypoint.pt.y < b_.keypoint.pt.y) ||
                                                                                           (a_.keypoint.pt.y == b_.keypoint.pt.y && a_.keypoint.pt.x < b_.keypoint.pt.x));});
  std::sort(_keypoints_with_descriptors_right.begin(), _keypoints_with_descriptors_right.end(),
            [](const IntensityFeature& a_, const IntensityFeature& b_){return ((a_.keypoint.pt.y < b_.keypoint.pt.y) ||
                                                                                           (a_.keypoint.pt.y == b_.keypoint.pt.y && a_.keypoint.pt.x < b_.keypoint.pt.x));});

  //ds number of stereo matches
  _number_of_available_points = 0;

  //ds adjust triangluation distance depending on frame status
  //ds when localizing we must be very careful since the motion model is not initialized yet) - narrow distance TODO adjust for floating point descriptors
  real maximum_matching_distance_triangulation = _parameters->maximum_matching_distance_triangulation;
  if (frame_->status() == Frame::Localizing) {
    maximum_matching_distance_triangulation = std::min(maximum_matching_distance_triangulation, 0.1*SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS);
  }

  //ds running variable
  uint32_t index_R = 0;

  //ds loop over all left keypoints
  for (uint32_t index_L = 0; index_L < _keypoints_with_descriptors_left.size(); index_L++) {

        //ds if there are no more points on the right to match against - stop
        if (index_R == _keypoints_with_descriptors_right.size()) {break;}

        //ds the right keypoints are on an lower row - skip left
        while (_keypoints_with_descriptors_left[index_L].keypoint.pt.y < _keypoints_with_descriptors_right[index_R].keypoint.pt.y) {
          index_L++; if (index_L == _keypoints_with_descriptors_left.size()) {break;}
        }
        if (index_L == _keypoints_with_descriptors_left.size()) {break;}

        //ds the right keypoints are on an upper row - skip right
        while (_keypoints_with_descriptors_left[index_L].keypoint.pt.y > _keypoints_with_descriptors_right[index_R].keypoint.pt.y) {
          index_R++; if (index_R == _keypoints_with_descriptors_right.size()) {break;}
        }
        if (index_R == _keypoints_with_descriptors_right.size()) {break;}

        //ds search bookkeeping
        uint32_t index_search_R = index_R;
        real distance_best      = maximum_matching_distance_triangulation;
        uint32_t index_best_R   = 0;

        //ds scan epipolar line for current keypoint at idx_L - exhaustive
        while (_keypoints_with_descriptors_left[index_L].keypoint.pt.y == _keypoints_with_descriptors_right[index_search_R].keypoint.pt.y) {

          //ds invalid disparity stop condition
          if (_keypoints_with_descriptors_left[index_L].keypoint.pt.x-_keypoints_with_descriptors_right[index_search_R].keypoint.pt.x < _parameters->minimum_disparity_pixels) {break;}

          //ds compute descriptor distance for the stereo match candidates
          const real distance_hamming = cv::norm(_keypoints_with_descriptors_left[index_L].descriptor, _keypoints_with_descriptors_right[index_search_R].descriptor, SRRG_PROSLAM_DESCRIPTOR_NORM);
          if(distance_hamming < distance_best) {
            distance_best = distance_hamming;
            index_best_R  = index_search_R;
          }
          index_search_R++; if (index_search_R == _keypoints_with_descriptors_right.size()) {break;}
        }

        //ds check if something was found
        if (distance_best < maximum_matching_distance_triangulation) {

            //ds attempt the triangulation
            const cv::KeyPoint& keypoint_left  = _keypoints_with_descriptors_left[index_L].keypoint;
            const cv::KeyPoint& keypoint_right = _keypoints_with_descriptors_right[index_best_R].keypoint;
            const PointCoordinates point_in_camera_left(getCoordinatesInCameraLeft(keypoint_left.pt, keypoint_right.pt));

            //ds add to framepoint map
            const uint32_t& r = keypoint_left.pt.y;
            const uint32_t& c = keypoint_left.pt.x;
            _framepoints_in_image[r][c] = frame_->createFramepoint(keypoint_left,
                                                                   _keypoints_with_descriptors_left[index_L].descriptor,
                                                                   keypoint_right,
                                                                   _keypoints_with_descriptors_right[index_best_R].descriptor,
                                                                   point_in_camera_left);

            //ds reduce search space (this eliminates all structurally conflicting matches)
            index_R = index_best_R+1;
        }
  }
  _number_of_available_points = frame_->createdPoints().size();

  //ds check currently achieved stereo points to triangulation ratio
  const real triangulation_succcess_ratio = static_cast<real>(_number_of_available_points)/_keypoints_with_descriptors_left.size();
  if (triangulation_succcess_ratio < 0.25) {

    //ds raise threshold (tolerance)
    LOG_WARNING(std::cerr << "StereoFramePointGenerator::findStereoKeypoints|low triangulation success ratio: " << triangulation_succcess_ratio
              << " (" << _number_of_available_points << "/" << _keypoints_with_descriptors_left.size() << ")" << std::endl)
  }

  //ds update the average
  _triangulation_success_ratios.push_back(triangulation_succcess_ratio);
  _mean_triangulation_success_ratio = (_number_of_triangulations*_mean_triangulation_success_ratio+triangulation_succcess_ratio)/(_number_of_triangulations+1);
  ++_number_of_triangulations;
}

//ds computes 3D position of a stereo keypoint pair in the keft camera frame (called within findStereoKeypoints)
const PointCoordinates StereoFramePointGenerator::getCoordinatesInCameraLeft(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) const {

  //ds check for minimal disparity
  if (image_coordinates_left_.x-image_coordinates_right_.x < _parameters->minimum_disparity_pixels) {
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

const PointCoordinates StereoFramePointGenerator::getPointInLeftCamera(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) const {
  assert(image_coordinates_left_.x >= image_coordinates_right_.x);
  assert(image_coordinates_left_.x-image_coordinates_right_.x >= _parameters->minimum_disparity_pixels);

  //ds point coordinates in camera frame
  PointCoordinates position_in_left_camera(Eigen::Vector3d::Zero());

  //ds triangulate point (assuming non-zero disparity)
  position_in_left_camera.z() = _b_x/(image_coordinates_right_.x-image_coordinates_left_.x);

  //ds compute x in camera (regular)
  position_in_left_camera.x() = 1/_f_x*(image_coordinates_left_.x-_c_x)*position_in_left_camera.z();

  //ds average in case we have an epipolar offset in v
  position_in_left_camera.y() = 1/_f_y*((image_coordinates_left_.y+image_coordinates_right_.y)/2.0-_c_y)*position_in_left_camera.z();
  return position_in_left_camera;
}

void StereoFramePointGenerator::_setFramepoints(Frame* frame_) {

  //ds do not resize framepoints (this function might be called when points are already present, e.g. after tracking)
  FramePointPointerVector& framepoints(frame_->points());
  uint32_t number_of_points               = framepoints.size();
  const uint32_t number_of_stereo_matches_initial = number_of_points;

  //ds prepare for fast stereo matching
  _feature_matcher_left.sortFeatureVector();
  _feature_matcher_right.sortFeatureVector();
  IntensityFeaturePointerVector& features_left(_feature_matcher_left.feature_vector);
  IntensityFeaturePointerVector& features_right(_feature_matcher_right.feature_vector);

  //ds allocate space for the new matches (best case we match all of them)
  framepoints.resize(number_of_stereo_matches_initial+features_left.size());

  //ds start stereo matching for all epipolar offsets
  for (const int32_t& epipolar_offset: _epipolar_search_offsets_pixel) {

    //ds matched features (to not consider them for the next offset search)
    std::set<uint32_t> matched_indices_left;
    std::set<uint32_t> matched_indices_right;

    //ds running variable
    uint32_t index_R = 0;

    //ds loop over all left keypoints
    for (uint32_t index_L = 0; index_L < features_left.size(); index_L++) {

      //ds if there are no more points on the right to match against - stop
      if (index_R == features_right.size()) {break;}

      //ds the right keypoints are on an lower row - skip left
      while (features_left[index_L]->row < features_right[index_R]->row+epipolar_offset) {
        index_L++; if (index_L == features_left.size()) {break;}
      }
      if (index_L == features_left.size()) {break;}
      IntensityFeature* feature_left = features_left[index_L];

      //ds the right keypoints are on an upper row - skip right
      while (feature_left->row > features_right[index_R]->row+epipolar_offset) {
        index_R++; if (index_R == features_right.size()) {break;}
      }
      if (index_R == features_right.size()) {break;}

      //ds search bookkeeping
      uint32_t index_search_R         = index_R;
      double descriptor_distance_best = _parameters->maximum_matching_distance_triangulation;
      uint32_t index_best_R           = 0;

      //ds scan epipolar line for current keypoint at idx_L - exhaustive
      while (feature_left->row == features_right[index_search_R]->row+epipolar_offset) {

        //ds invalid disparity stop condition
        if (feature_left->col-features_right[index_search_R]->col < 0) {break;}

        //ds compute descriptor distance for the stereo match candidates
        const double descriptor_distance = cv::norm(feature_left->descriptor, features_right[index_search_R]->descriptor, SRRG_PROSLAM_DESCRIPTOR_NORM);
        if(descriptor_distance < descriptor_distance_best) {
          descriptor_distance_best = descriptor_distance;
          index_best_R             = index_search_R;
        }
        index_search_R++; if (index_search_R == features_right.size()) {break;}
      }

      //ds check if something was found
      if (descriptor_distance_best < _parameters->maximum_matching_distance_triangulation) {
        IntensityFeature* feature_right = features_right[index_best_R];

        //ds drop invalid disparities
        if (feature_left->col-feature_right->col < _parameters->minimum_disparity_pixels) {
          continue;
        }

        //ds register match without track
        framepoints[number_of_points] = frame_->createFramepoint(feature_left,
                                                                         feature_right,
                                                                         getPointInLeftCamera(feature_left->keypoint.pt, feature_right->keypoint.pt));
        framepoints[number_of_points]->setEpipolarOffset(epipolar_offset);
        ++number_of_points;

        //ds block further matching against features_right[index_best_R] in a search on offset epipolar lines
        matched_indices_left.insert(index_L);
        matched_indices_right.insert(index_best_R);

        //ds reduce search space (this eliminates all structurally conflicting matches)
        index_R = index_best_R+1;
      }
    }

    //ds remove matched indices from candidate pools
    _feature_matcher_left.prune(matched_indices_left);
    _feature_matcher_right.prune(matched_indices_right);
    LOG_DEBUG(std::cerr << "stereoMatchExhaustive|epipolar offset: " << epipolar_offset << " number of unmatched features L: "
              << features_left.size() << " R: " << features_right.size() << std::endl)
  }
  framepoints.resize(number_of_points);
}

void StereoFramePointGenerator::_setFramepoints(Frame* frame_,
                                                Frame* frame_previous_,
                                                const TransformMatrix3D& camera_left_previous_in_current_) {
  const Matrix3& camera_calibration_matrix = _camera_left->cameraMatrix();
  FramePointPointerVector& framepoints(frame_->points());
  FramePointPointerVector& framepoints_previous(frame_previous_->points());
  framepoints.resize(framepoints_previous.size());

  //ds tracked and triangulated features (to not consider them in the exhaustive stereo matching)
  std::set<uint32_t> matched_indices_left;
  std::set<uint32_t> matched_indices_right;
  Count number_of_points = 0;

  //ds for each previous point
  for (FramePoint* point_previous: framepoints_previous) {

    //ds transform the point into the current camera frame
    const Vector3 point_in_camera_left_prediction(camera_left_previous_in_current_*point_previous->cameraCoordinatesLeft());

    //ds project the point into the current left image plane
    const Vector3 point_in_image_left(camera_calibration_matrix*point_in_camera_left_prediction);
    const int32_t col_projection_left = point_in_image_left.x()/point_in_image_left.z();
    const int32_t row_projection_left = point_in_image_left.y()/point_in_image_left.z();

    //ds skip point if not in image plane
    if (col_projection_left < 0 || col_projection_left > _number_of_cols_image ||
        row_projection_left < 0 || row_projection_left > _number_of_rows_image) {
      continue;
    }

    //ds TRACKING obtain matching feature in left image (if any)
    IntensityFeature* feature_left = _feature_matcher_left.getMatchingFeatureInRectangularRegion(row_projection_left,
                                                                                                 col_projection_left,
                                                                                                 point_previous->descriptorLeft(),
                                                                                                 _projection_tracking_distance_pixels,
                                                                                                 _projection_tracking_distance_pixels,
                                                                                                 _parameters->maximum_matching_distance_triangulation);

    //ds if we found a match
    if (feature_left) {

      //ds compute projection offset (i.e. prediction error > optical flow)
      const cv::Point2f projection_error(col_projection_left-feature_left->keypoint.pt.x, row_projection_left-feature_left->keypoint.pt.y);

      //ds project point into the right image - correcting by the prediction error
      const Vector3 point_in_image_right(point_in_image_left+_baseline);
      const int32_t col_projection_right = point_in_image_right.x()/point_in_image_right.z()-projection_error.x;
      const int32_t row_projection_right = point_in_image_right.y()/point_in_image_right.z()-projection_error.y;

      //ds skip point if not in image plane
      if (col_projection_right < 0 || col_projection_right > _number_of_cols_image ||
          row_projection_right < 0 || row_projection_right > _number_of_rows_image) {
        continue;
      }

      //ds TRIANGULATION: obtain matching feature in right image (if any)
      //ds we reduce the vertical matching space to the epipolar range
      //ds we increase the matching tolerance (2x) since we have a strong prior on location
      IntensityFeature* feature_right = _feature_matcher_right.getMatchingFeatureInRectangularRegion(row_projection_right,
                                                                                                     col_projection_right,
                                                                                                     feature_left->descriptor,
                                                                                                     _parameters->maximum_epipolar_search_offset_pixels,
                                                                                                     _projection_tracking_distance_pixels,
                                                                                                     2*_parameters->maximum_matching_distance_triangulation);

      //ds if we found a match
      if (feature_right) {
        assert(feature_left->col >= feature_right->col);

        //ds if disparity is sufficient
        if (feature_left->col-feature_right->col > _parameters->minimum_disparity_pixels) {

          //ds create a stereo match
          FramePoint* framepoint = frame_->createFramepoint(feature_left,
                                                            feature_right,
                                                            getPointInLeftCamera(feature_left->keypoint.pt, feature_right->keypoint.pt),
                                                            point_previous);
          framepoint->setEpipolarOffset(feature_right->row-feature_left->col);

          //ds visualization only
          framepoint->setProjectionEstimateLeft(cv::Point2f(col_projection_right, row_projection_left));
          framepoint->setProjectionEstimateRight(cv::Point2f(point_in_image_right.x()/point_in_image_right.z(), point_in_image_right.y()/point_in_image_right.z()));
          framepoint->setProjectionEstimateRightCorrected(cv::Point2f(col_projection_right, row_projection_right));

          //ds store and move to next slot
          framepoints[number_of_points] = framepoint;
          ++number_of_points;

          //ds block matching in exhaustive matching (later)
          matched_indices_left.insert(feature_left->index_in_vector);
          matched_indices_right.insert(feature_right->index_in_vector);

          //ds remove feature from lattices
          _feature_matcher_left.feature_lattice[feature_left->row][feature_left->col]    = nullptr;
          _feature_matcher_right.feature_lattice[feature_right->row][feature_right->col] = nullptr;
        }
      }
    }
  }
  framepoints.resize(number_of_points);

  //ds remove matched indices from candidate pools
  _feature_matcher_left.prune(matched_indices_left);
  _feature_matcher_right.prune(matched_indices_right);
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::_setFramepoints|tracked and triangulated points: " << number_of_points
                      << "/" << framepoints_previous.size() << std::endl)
}

const real StereoFramePointGenerator::standardDeviationTriangulationSuccessRatio() const {
  real standard_deviation_triangulation_success_ratio = 0;
  for (const double& triangulation_success_ratio: _triangulation_success_ratios) {
    standard_deviation_triangulation_success_ratio += (_mean_triangulation_success_ratio-triangulation_success_ratio)
                                                     *(_mean_triangulation_success_ratio-triangulation_success_ratio);
  }
  standard_deviation_triangulation_success_ratio /= _triangulation_success_ratios.size();
  standard_deviation_triangulation_success_ratio = std::sqrt(standard_deviation_triangulation_success_ratio);
  return standard_deviation_triangulation_success_ratio;
}
}
