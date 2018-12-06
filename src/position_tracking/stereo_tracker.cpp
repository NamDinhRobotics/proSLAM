#include "stereo_tracker.h"

namespace proslam {
  using namespace srrg_core;

  //ds the tracker assumes a constant stereo camera configuration
  StereoTracker::StereoTracker(StereoTrackerParameters* parameters_): BaseTracker(parameters_),
                                                                      _parameters(parameters_) {
    LOG_INFO(std::cerr << "StereoTracker::StereoTracker|constructed" << std::endl)
  }

  void StereoTracker::configure() {
    LOG_INFO(std::cerr << "StereoTracker::configure|configuring" << std::endl)
    assert(_camera_right);
    BaseTracker::configure();
    _stereo_framepoint_generator = dynamic_cast<StereoFramePointGenerator*>(_framepoint_generator);
    assert(_stereo_framepoint_generator);
    LOG_INFO(std::cerr << "StereoTracker::configure|configured" << std::endl)
  }

  //ds dynamic cleanup
  StereoTracker::~StereoTracker() {
    LOG_INFO(std::cerr << "StereoTracker::~StereoTracker|destroyed" << std::endl)
  }

  Frame* StereoTracker::_createFrame(){
    Frame* current_frame = _context->createFrame(_context->robotToWorld());
    current_frame->setCameraLeft(_camera_left);
    current_frame->setIntensityImageLeft(_intensity_image_left);
    current_frame->setCameraRight(_camera_right);
    current_frame->setIntensityImageRight(_intensity_image_right);
    current_frame->setRobotToWorld(_context->robotToWorld()); //ds TODO remove
    return current_frame;
  }
  
  //ds creates a new Frame for the given images, retrieves the correspondences relative to the previous Frame, optimizes the current frame pose and updates landmarks
  void StereoTracker::compute() {
    BaseTracker::compute();
  }

  //ds attempts to recover framepoints in the current image using the more precise pose estimate, retrieved after pose optimization
  void StereoTracker::_recoverPoints(Frame* current_frame_) {

    //ds precompute transforms
    const TransformMatrix3D world_to_camera_left  = current_frame_->worldToCameraLeft();
    const CameraMatrix& camera_calibration_matrix = _camera_left->cameraMatrix();
    const Vector3 baseline_homogeneous            = _camera_right->baselineHomogeneous();

    //ds obtain currently active tracking distance
    const real maximum_descriptor_distance = _framepoint_generator->parameters()->matching_distance_tracking_threshold;

    //ds buffers
    const cv::Mat& intensity_image_left  = current_frame_->intensityImageLeft();
    const cv::Mat& intensity_image_right = current_frame_->intensityImageRight();
    std::vector<cv::KeyPoint> keypoint_buffer_left(1);
    std::vector<cv::KeyPoint> keypoint_buffer_right(1);

    //ds recover lost landmarks
    Index index_lost_point_recovered = _number_of_tracked_points;
    current_frame_->points().resize(_number_of_tracked_points+_number_of_lost_points);
    for (FramePoint* point_previous: _lost_points) {

      //ds skip non landmarks for now (TODO parametrize)
      if (!point_previous->landmark()) {
        continue;
      }

      //ds get point into current camera - based on last track
      PointCoordinates point_in_camera_homogeneous(Vector3::Zero());

      //ds if we have a landmark at hand
      if (point_previous->landmark()) {
        point_previous->landmark()->incrementNumberOfRecoveries();

        //ds get point in camera frame based on landmark coordinates
        point_in_camera_homogeneous = world_to_camera_left*point_previous->landmark()->coordinates();
      } else {

        //ds get point in camera frame based on point coordinates
        point_in_camera_homogeneous = world_to_camera_left*point_previous->worldCoordinates();
      }

      //ds obtain point projection on camera image plane
      PointCoordinates point_in_image_left  = camera_calibration_matrix*point_in_camera_homogeneous;
      PointCoordinates point_in_image_right = point_in_image_left+baseline_homogeneous;

      //ds normalize point and update prediction based on landmark position: LEFT
      point_in_image_left  /= point_in_image_left.z();
      point_in_image_right /= point_in_image_right.z();

      //ds check for invalid projections
      if (point_in_image_left.x() < 0 || point_in_image_left.x() > _camera_left->numberOfImageCols()  ||
          point_in_image_right.x() < 0 || point_in_image_right.x() > _camera_left->numberOfImageCols()||
          point_in_image_left.y() < 0 || point_in_image_left.y() > _camera_left->numberOfImageRows()  ) {

        //ds out of FOV
        continue;
      }
      assert(point_in_image_left.y() == point_in_image_right.y());

      //ds set projections - at subpixel accuarcy
      const cv::Point2f projection_left(point_in_image_left.x(), point_in_image_left.y());
      const cv::Point2f projection_right(point_in_image_right.x(), point_in_image_right.y());

      //ds this can be moved outside of the loop if keypoint sizes are constant
      const float regional_border_center = 5*point_previous->keypointLeft().size;
      const cv::Point2f offset_keypoint_half(regional_border_center, regional_border_center);
      const float regional_full_height = regional_border_center+regional_border_center+1;

      //ds if available search range is insufficient
      if (projection_left.x <= regional_border_center+1                                   ||
          projection_left.x >= _camera_left->numberOfImageCols()-regional_border_center-1 ||
          projection_left.y <= regional_border_center+1                                   ||
          projection_left.y >= _camera_left->numberOfImageRows()-regional_border_center-1 ||
          projection_right.x <= regional_border_center+1                                  ||
          projection_right.x >= _camera_left->numberOfImageCols()-regional_border_center-1) {

        //ds skip complete tracking
        continue;
      }

      //ds left search regions
      const cv::Point2f corner_left(projection_left-offset_keypoint_half);
      const cv::Rect_<float> region_of_interest_left(corner_left.x, corner_left.y, regional_full_height, regional_full_height);

      //ds extract descriptors at this position: LEFT
      keypoint_buffer_left[0]    = point_previous->keypointLeft();
      keypoint_buffer_left[0].pt = offset_keypoint_half;
      cv::Mat descriptor_left;
      const cv::Mat roi_left(intensity_image_left(region_of_interest_left));
      _framepoint_generator->descriptorExtractor()->compute(roi_left, keypoint_buffer_left, descriptor_left);

      //ds if no descriptor could be computed
      if (descriptor_left.rows == 0) {
        continue;
      }

      //ds if descriptor distance is to high
      if (cv::norm(point_previous->descriptorLeft(), descriptor_left, SRRG_PROSLAM_DESCRIPTOR_NORM) > maximum_descriptor_distance) {
        continue;
      }
      keypoint_buffer_left[0].pt += corner_left;

      //ds right search region
      const cv::Point2f corner_right(projection_right-offset_keypoint_half);
      const cv::Rect_<float> region_of_interest_right(corner_right.x, corner_right.y, regional_full_height, regional_full_height);

      //ds extract descriptors at this position: RIGHT
      keypoint_buffer_right[0] = point_previous->keypointRight();
      keypoint_buffer_right[0].pt = offset_keypoint_half;
      cv::Mat descriptor_right;
      const cv::Mat roi_right(intensity_image_right(region_of_interest_right));
      _framepoint_generator->descriptorExtractor()->compute(roi_right, keypoint_buffer_right, descriptor_right);

      //ds if no descriptor could be computed
      if (descriptor_right.rows == 0) {
        continue;
      }

      //ds if descriptor distance is to high
      if (cv::norm(point_previous->descriptorRight(), descriptor_right, SRRG_PROSLAM_DESCRIPTOR_NORM) > maximum_descriptor_distance) {
        continue;
      }
      keypoint_buffer_right[0].pt += corner_right;

      //ds skip points with insufficient stereo disparity
      if (keypoint_buffer_left[0].pt.x-keypoint_buffer_right[0].pt.x < _stereo_framepoint_generator->parameters()->minimum_disparity_pixels) {
        continue;
      }

      //ds allocate a new point connected to the previous one
      FramePoint* current_point = current_frame_->createFramepoint(keypoint_buffer_left[0],
                                                                   descriptor_left,
                                                                   keypoint_buffer_right[0],
                                                                   descriptor_right,
                                                                   _stereo_framepoint_generator->getPointInLeftCamera(keypoint_buffer_left[0].pt, keypoint_buffer_right[0].pt),
                                                                   point_previous);

      //ds set the point to the control structure
      current_frame_->points()[index_lost_point_recovered] = current_point;
      ++index_lost_point_recovered;
    }
    _number_of_recovered_points = index_lost_point_recovered-_number_of_tracked_points;
    _number_of_tracked_points = index_lost_point_recovered;
    current_frame_->points().resize(_number_of_tracked_points);
    LOG_DEBUG(std::cerr << "StereoTracker::_recoverPoints|recovered points: " << _number_of_recovered_points << "/" << _number_of_lost_points << std::endl)
  }
}
