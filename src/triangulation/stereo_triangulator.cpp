#include "stereo_triangulator.h"

namespace proslam {

  StereoTriangulator::StereoTriangulator() {
    _camera_right=0;
  }
  
  //ds the stereo camera setup must be provided
  void StereoTriangulator::setup(){

    assert(_camera_right);

    BaseFramePointGenerator::setup();
    _baseline_pixelsmeters = _camera_right->projectionMatrix()(0,3);
    _baseline_meters = -_baseline_pixelsmeters/_focal_length_pixels;
    _maximum_depth_near_meters = _baseline_factor*_baseline_meters;
    _maximum_depth_far_meters = -_baseline_pixelsmeters/_minimum_disparity_pixels,

    //ds clear buffers
    _keypoints_right.clear();
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

}
