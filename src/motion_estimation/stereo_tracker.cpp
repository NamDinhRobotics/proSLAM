#include "stereo_tracker.h"

namespace proslam {
  using namespace srrg_core;

  //ds the tracker assumes a constant stereo camera configuration
  StereoTracker::StereoTracker(): _camera_right(0),
                                  _intensity_image_right(0),
                                  _stereo_framepoint_generator(0),
                                  _parameters(0) {
    LOG_INFO(std::cerr << "StereoTracker::StereoTracker|constructed" << std::endl)
  }

  void StereoTracker::configure(BaseTrackerParameters* parameters_) {
    LOG_INFO(std::cerr << "StereoTracker::setup|configuring" << std::endl)
    _parameters = dynamic_cast<StereoTrackerParameters*>(parameters_);
    assert(_camera_right);
    BaseTracker::configure(parameters_);
    _stereo_framepoint_generator = dynamic_cast<StereoFramePointGenerator*>(_framepoint_generator);
    assert(_stereo_framepoint_generator);
    LOG_INFO(std::cerr << "StereoTracker::setup|configured" << std::endl)
  }

  //ds dynamic cleanup
  StereoTracker::~StereoTracker() {
    LOG_INFO(std::cerr << "StereoTracker::StereoTracker|destroying" << std::endl)
    LOG_INFO(std::cerr << "StereoTracker::StereoTracker|destroyed" << std::endl)
  }

  Frame* StereoTracker::_createFrame(){
    Frame* current_frame = _context->createFrame(_context->robotToWorld(), _framepoint_generator->maximumDepthNearMeters());
    current_frame->setCameraLeft(_camera_left);
    current_frame->setIntensityImageLeft(_intensity_image_left);
    current_frame->setCameraRight(_camera_right);
    current_frame->setIntensityImageRight(_intensity_image_right);
    return current_frame;
  }
  
  //ds creates a new Frame for the given images, retrieves the correspondences relative to the previous Frame, optimizes the current frame pose and updates landmarks
  void StereoTracker::compute() {
    assert(_intensity_image_right);
    BaseTracker::compute();
  }

  //ds attempts to recover framepoints in the current image using the more precise pose estimate, retrieved after pose optimization
  void StereoTracker::_recoverPoints(Frame* current_frame_) {

    //ds precompute transforms
    const TransformMatrix3D world_to_camera_left    = _camera_left->robotToCamera()*current_frame_->worldToRobot();
    const ProjectionMatrix& projection_matrix_left  = _camera_left->projectionMatrix();
    const ProjectionMatrix& projection_matrix_right = _camera_right->projectionMatrix();

    //ds buffers
    const cv::Mat& intensity_image_left  = current_frame_->intensityImageLeft();
    const cv::Mat& intensity_image_right = current_frame_->intensityImageRight();
    std::vector<cv::KeyPoint> keypoint_buffer_left(1);
    std::vector<cv::KeyPoint> keypoint_buffer_right(1);

    //ds recover lost landmarks
    Index index_lost_point_recovered = _number_of_tracked_points;
    current_frame_->activePoints().resize(_number_of_tracked_points+_number_of_lost_points);
    for (FramePoint* point_previous: _lost_points) {

      //ds get point into current camera - based on last track
      Vector4 point_in_camera_homogeneous(Vector4::Ones());

      //ds if we have a landmark at hand
      if (point_previous->landmark()) {
        point_previous->landmark()->incrementNumberOfRecoveries();

        //ds get point in camera frame based on landmark coordinates
        point_in_camera_homogeneous.head<3>() = world_to_camera_left*point_previous->landmark()->coordinates();
      } else {

        //ds get point in camera frame based on point coordinates
        point_in_camera_homogeneous.head<3>() = world_to_camera_left*point_previous->worldCoordinates();
      }

      //ds obtain point projection on camera image plane
      PointCoordinates point_in_image_left  = projection_matrix_left*point_in_camera_homogeneous;
      PointCoordinates point_in_image_right = projection_matrix_right*point_in_camera_homogeneous;

      //ds normalize point and update prediction based on landmark position: LEFT
      point_in_image_left  /= point_in_image_left.z();
      point_in_image_right /= point_in_image_right.z();

      //ds check for invalid projections
      if (point_in_image_left.x() < 0 || point_in_image_left.x() > _number_of_cols_image  ||
          point_in_image_right.x() < 0 || point_in_image_right.x() > _number_of_cols_image||
          point_in_image_left.y() < 0 || point_in_image_left.y() > _number_of_rows_image  ) {

        //ds out of FOV
        continue;
      }
      assert(point_in_image_left.y() == point_in_image_right.y());

      //ds set projections
      const cv::Point2f projection_left(point_in_image_left.x(), point_in_image_left.y());
      const cv::Point2f projection_right(point_in_image_right.x(), point_in_image_right.y());

      //ds this can be moved outside of the loop if keypoint sizes are constant
      const float regional_border_center = 4*point_previous->keypointLeft().size;
      const cv::Point2f offset_keypoint_half(regional_border_center, regional_border_center);
      const float regional_full_height = regional_border_center+regional_border_center+1;

      //ds if available search range is insufficient
      if (projection_left.x <= regional_border_center+1              ||
          projection_left.x >= _number_of_cols_image-regional_border_center-1 ||
          projection_left.y <= regional_border_center+1              ||
          projection_left.y >= _number_of_rows_image-regional_border_center-1 ||
          projection_right.x <= regional_border_center+1             ||
          projection_right.x >= _number_of_cols_image-regional_border_center-1) {

        //ds skip complete tracking
        continue;
      }

      //ds extraction regions
      const cv::Point2f corner_left(projection_left-offset_keypoint_half);
      const cv::Rect_<float> region_of_interest_left(corner_left.x, corner_left.y, regional_full_height, regional_full_height);
      const cv::Point2f corner_right(projection_right-offset_keypoint_half);
      const cv::Rect_<float> region_of_interest_right(corner_right.x, corner_right.y, regional_full_height, regional_full_height);

      //ds extract descriptors at this position: LEFT
      keypoint_buffer_left[0]    = point_previous->keypointLeft();
      keypoint_buffer_left[0].pt = offset_keypoint_half;
      cv::Mat descriptor_left;
      const cv::Mat roi_left(intensity_image_left(region_of_interest_left));
      _framepoint_generator->descriptorExtractor()->compute(roi_left, keypoint_buffer_left, descriptor_left);
      if (descriptor_left.rows == 0) {
        continue;
      }
      keypoint_buffer_left[0].pt += corner_left;

      //ds extract descriptors at this position: RIGHT
      keypoint_buffer_right[0] = point_previous->keypointRight();
      keypoint_buffer_right[0].pt = offset_keypoint_half;
      cv::Mat descriptor_right;
      const cv::Mat roi_right(intensity_image_right(region_of_interest_right));
      _framepoint_generator->descriptorExtractor()->compute(roi_right, keypoint_buffer_right, descriptor_right);
      if (descriptor_right.rows == 0) {
        continue;
      }
      keypoint_buffer_right[0].pt += corner_right;

      if (cv::norm(point_previous->descriptorLeft(), descriptor_left, DESCRIPTOR_NORM) < _framepoint_generator->matchingDistanceTrackingThreshold()  &&
          cv::norm(point_previous->descriptorRight(), descriptor_right, DESCRIPTOR_NORM) < _framepoint_generator->matchingDistanceTrackingThreshold()) {
        try {

          //ds triangulate point
          const PointCoordinates camera_coordinates(_stereo_framepoint_generator->getCoordinatesInCameraLeft(keypoint_buffer_left[0].pt, keypoint_buffer_right[0].pt));

          //ds allocate a new point connected to the previous one
          FramePoint* current_point = current_frame_->create(keypoint_buffer_left[0],
                                                             descriptor_left,
                                                             keypoint_buffer_right[0],
                                                             descriptor_right,
                                                             camera_coordinates,
                                                             point_previous);

          //ds set the point to the control structure
          current_frame_->activePoints()[index_lost_point_recovered] = current_point;
          ++index_lost_point_recovered;
        } catch (const ExceptionTriangulation& /*exception_*/) {}
      }
    }
    _number_of_lost_points_recovered = index_lost_point_recovered-_number_of_tracked_points;
    _number_of_tracked_points = index_lost_point_recovered;
    current_frame_->activePoints().resize(_number_of_tracked_points);
    //    std::cerr << "StereoTracker::recoverPoints|recovered points: " << _number_of_lost_points_recovered << "/" << _number_of_lost_points << std::endl;
  }
}
