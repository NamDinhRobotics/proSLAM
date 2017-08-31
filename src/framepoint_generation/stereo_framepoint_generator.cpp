#include "stereo_framepoint_generator.h"

namespace proslam {

  StereoFramePointGenerator::StereoFramePointGenerator(StereoFramePointGeneratorParameters* parameters_): BaseFramePointGenerator(parameters_),
                                                                                                          _camera_right(0),
                                                                                                          _baseline_pixelsmeters(0),
                                                                                                          _baseline_meters(0),
                                                                                                          _parameters(parameters_) {
    LOG_DEBUG(std::cerr << "StereoFramePointGenerator::StereoFramePointGenerator|construced" << std::endl)
  }

  //ds the stereo camera setup must be provided
  void StereoFramePointGenerator::configure(){
    LOG_DEBUG(std::cerr << "StereoFramePointGenerator::configure|configuring" << std::endl)
    assert(_camera_right);

    //ds integrate configuration
    BaseFramePointGenerator::configure();

    //ds configure current
    _baseline_pixelsmeters     = _camera_right->projectionMatrix()(0,3);
    _baseline_meters           = -_baseline_pixelsmeters/_focal_length_pixels;
    _maximum_depth_near_meters = _parameters->baseline_factor*_baseline_meters;
    _maximum_depth_far_meters  = -_baseline_pixelsmeters/_parameters->minimum_disparity_pixels;
    _keypoints_with_descriptors_left.clear();
    _keypoints_with_descriptors_right.clear();

    //ds compute stereo matching configuration
    _epipolar_line_offsets_pixels.clear();
    _epipolar_line_offsets_pixels.push_back(0);
    for (uint32_t thickness_level = 1; thickness_level <= _parameters->epipolar_line_thickness_pixels; ++thickness_level) {

      //ds always check upper and lower line next to epipolar
      _epipolar_line_offsets_pixels.push_back(thickness_level);
      _epipolar_line_offsets_pixels.push_back(-thickness_level);
    }

    //ds info
    LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|baseline (m): " << _baseline_meters << std::endl)
    LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|maximum depth tracking close (m): " << _maximum_depth_near_meters << std::endl)
    LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|maximum depth tracking far (m): " << _maximum_depth_far_meters << std::endl)
    LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|epipolar offsets to check: " << _epipolar_line_offsets_pixels.size()-1 << std::endl)
    LOG_DEBUG(std::cerr << "StereoFramePointGenerator::configure|configured" << std::endl)
  }

  //ds cleanup of dynamic structures
  StereoFramePointGenerator::~StereoFramePointGenerator() {
    LOG_DEBUG(std::cerr << "StereoFramePointGenerator::~StereoFramePointGenerator|destroying" << std::endl)
    LOG_DEBUG(std::cerr << "StereoFramePointGenerator::~StereoFramePointGenerator|destroyed" << std::endl)
  }

  //ds computes framepoints stored in a image-like matrix (_framepoints_in_image) for provided stereo images
  void StereoFramePointGenerator::compute(Frame* frame_) {

    //ds detect new features to generate frame points from
    detectKeypoints(frame_->intensityImageLeft(), frame_->keypointsLeft());
    detectKeypoints(frame_->intensityImageRight(), frame_->keypointsRight());

    //ds extract descriptors for detected features
    extractDescriptors(frame_->intensityImageLeft(), frame_->keypointsLeft(), frame_->descriptorsLeft());
    extractDescriptors(frame_->intensityImageRight(), frame_->keypointsRight(), frame_->descriptorsRight());

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
        _keypoints_with_descriptors_left[u].available   = true;
        _keypoints_with_descriptors_right[u].keypoint   = frame_->keypointsRight()[u];
        _keypoints_with_descriptors_right[u].descriptor = frame_->descriptorsRight().row(u);
        _keypoints_with_descriptors_right[u].available  = true;
      }

      //ds add the remaining points from the right image
      for (Index u = frame_->keypointsLeft().size(); u < frame_->keypointsRight().size(); ++u) {
        _keypoints_with_descriptors_right[u].keypoint   = frame_->keypointsRight()[u];
        _keypoints_with_descriptors_right[u].descriptor = frame_->descriptorsRight().row(u);
        _keypoints_with_descriptors_right[u].available  = true;
      }

    //ds if we got more keypoints in the left image
    } else {

      //ds first add all right keypoints plus equally many from the left
      for (Index u = 0; u < frame_->keypointsRight().size(); ++u) {
        _keypoints_with_descriptors_left[u].keypoint    = frame_->keypointsLeft()[u];
        _keypoints_with_descriptors_left[u].descriptor  = frame_->descriptorsLeft().row(u);
        _keypoints_with_descriptors_left[u].available   = true;
        _keypoints_with_descriptors_right[u].keypoint   = frame_->keypointsRight()[u];
        _keypoints_with_descriptors_right[u].descriptor = frame_->descriptorsRight().row(u);
        _keypoints_with_descriptors_right[u].available  = true;
      }

      //ds add the remaining points from the left image
      for (Index u = frame_->keypointsRight().size(); u < frame_->keypointsLeft().size(); ++u) {
        _keypoints_with_descriptors_left[u].keypoint   = frame_->keypointsLeft()[u];
        _keypoints_with_descriptors_left[u].descriptor = frame_->descriptorsLeft().row(u);
        _keypoints_with_descriptors_left[u].available  = true;
      }
    }
  }

  //ds computes all potential stereo keypoints (exhaustive in matching distance) and stores them as framepoints (called within compute)
  void StereoFramePointGenerator::findStereoKeypoints(Frame* frame_) {

    //ds sort all input vectors by ascending row positions
    std::sort(_keypoints_with_descriptors_left.begin(), _keypoints_with_descriptors_left.end(),
              [](const KeypointWithDescriptor& a_, const KeypointWithDescriptor& b_){return ((a_.keypoint.pt.y < b_.keypoint.pt.y) ||
                                                                                             (a_.keypoint.pt.y == b_.keypoint.pt.y && a_.keypoint.pt.x < b_.keypoint.pt.x));});
    std::sort(_keypoints_with_descriptors_right.begin(), _keypoints_with_descriptors_right.end(),
              [](const KeypointWithDescriptor& a_, const KeypointWithDescriptor& b_){return ((a_.keypoint.pt.y < b_.keypoint.pt.y) ||
                                                                                             (a_.keypoint.pt.y == b_.keypoint.pt.y && a_.keypoint.pt.x < b_.keypoint.pt.x));});

    //ds number of stereo matches
    _number_of_available_points = 0;

    //ds for each offset (multi-line stereo matching)
    for (const int32_t& offset_pixels: _epipolar_line_offsets_pixels) {

      //ds running variable
      Index index_R = 0;

      //ds loop over all left keypoints
      for (Index idx_L = 0; idx_L < _keypoints_with_descriptors_left.size(); idx_L++) {

        //ds if the point is not yet matched
        if (_keypoints_with_descriptors_left[idx_L].available) {

          //ds if there are no more points on the right to match against - stop
          if (index_R == _keypoints_with_descriptors_right.size()) {break;}
          //the right keypoints are on an lower row - skip left
          while (_keypoints_with_descriptors_left[idx_L].keypoint.pt.y < _keypoints_with_descriptors_right[index_R].keypoint.pt.y+offset_pixels) {
            idx_L++; if (idx_L == _keypoints_with_descriptors_left.size()) {break;}
          }
          if (idx_L == _keypoints_with_descriptors_left.size()) {break;}
          //the right keypoints are on an upper row - skip right
          while (_keypoints_with_descriptors_left[idx_L].keypoint.pt.y > _keypoints_with_descriptors_right[index_R].keypoint.pt.y+offset_pixels) {
            index_R++; if (index_R == _keypoints_with_descriptors_right.size()) {break;}
          }
          if (index_R == _keypoints_with_descriptors_right.size()) {break;}
          //search bookkeeping
          Index index_search_R = index_R;
          real distance_best   = _parameters->maximum_matching_distance_triangulation;
          Index index_best_R   = 0;
          //scan epipolar line for current keypoint at idx_L
          while (_keypoints_with_descriptors_left[idx_L].keypoint.pt.y == _keypoints_with_descriptors_right[index_search_R].keypoint.pt.y+offset_pixels) {
            //zero disparity stop condition
            if (_keypoints_with_descriptors_right[index_search_R].keypoint.pt.x >= _keypoints_with_descriptors_left[idx_L].keypoint.pt.x) {break;}

            //ds if the point is not yet matched
            if (_keypoints_with_descriptors_right[index_search_R].available) {

              //ds compute descriptor distance for the stereo match candidates
              const real distance_hamming = cv::norm(_keypoints_with_descriptors_left[idx_L].descriptor, _keypoints_with_descriptors_right[index_search_R].descriptor, SRRG_PROSLAM_DESCRIPTOR_NORM);
              if(distance_hamming < distance_best) {
                distance_best = distance_hamming;
                index_best_R  = index_search_R;
              }
            }
            index_search_R++; if (index_search_R == _keypoints_with_descriptors_right.size()) {break;}
          }
          //check if something was found
          if (distance_best < _parameters->maximum_matching_distance_triangulation) {

            //ds add triangulation map entry
            try {

              const Index& row       = _keypoints_with_descriptors_left[idx_L].keypoint.pt.y;
              const Index& col_left  = _keypoints_with_descriptors_left[idx_L].keypoint.pt.x;
              assert(_framepoints_in_image[row][col_left] == 0);

              //ds attempt the triangulation - might throw on failure
              const PointCoordinates camera_coordinates(getCoordinatesInCameraLeft(_keypoints_with_descriptors_left[idx_L].keypoint.pt,
                                                                                   _keypoints_with_descriptors_right[index_best_R].keypoint.pt));

              //ds add to framepoint map
              _framepoints_in_image[row][col_left] = frame_->createFramepoint(_keypoints_with_descriptors_left[idx_L].keypoint,
                                                                    _keypoints_with_descriptors_left[idx_L].descriptor,
                                                                    _keypoints_with_descriptors_right[index_best_R].keypoint,
                                                                    _keypoints_with_descriptors_right[index_best_R].descriptor,
                                                                    camera_coordinates);
              ++_number_of_available_points;

              //ds reduce search space
              index_R = index_best_R+1;

              //ds set as matched (required for multi-line stereo matching)
              _keypoints_with_descriptors_left[idx_L].available         = false;
              _keypoints_with_descriptors_right[index_best_R].available = false;
            } catch (const ExceptionTriangulation& exception) {}
          }
        }
      }
    }

    //ds sanity check
    const real triangulation_succcess_ratio = static_cast<real>(_number_of_available_points)/_keypoints_with_descriptors_left.size();
    if (triangulation_succcess_ratio < 0.25) {
      LOG_WARNING(std::cerr << "StereoFramePointGenerator::findStereoKeypoints|low triangulation success ratio: " << triangulation_succcess_ratio
                << " (" << _number_of_available_points << "/" << _keypoints_with_descriptors_left.size() << ")" << std::endl)
    }
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
}
