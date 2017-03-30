#include "tracker.h"

namespace proslam {
  using namespace srrg_core;

  Tracker::Tracker(const Camera* camera_left_,
                         const Camera* camera_right_): _camera_left(camera_left_),
                                                       _camera_right(camera_right_),
                                                       _camera_rows(camera_left_->imageRows()),
                                                       _camera_cols(camera_left_->imageCols()),
                                                       _framepoint_generator(new StereoTriangulator(_camera_left, _camera_right)),
                                                       _pose_optimizer(new StereoUVAligner()) {
    std::cerr << "Tracker::Tracker|constructing" << std::endl;

    //ds request initial tracking context for the tracker
    _lost_points.clear();

    //ds allocate stereouv aligner for odometry computation
    assert(_pose_optimizer != 0);

    //ds configure aligner
    _pose_optimizer->setMaximumDepthClose(_framepoint_generator->maximumDepthCloseMeters());
    _pose_optimizer->setMaximumDepthFar(_framepoint_generator->maximumDepthFarMeters());

    //ds clear buffers
    _projected_image_coordinates_left.clear();
    std::cerr << "Tracker::Tracker|constructed" << std::endl;
  }

  Tracker::~Tracker() {
    std::cerr << "Tracker::Tracker|destroying" << std::endl;
    //ds TODO clear tracker owned scopes
    delete _framepoint_generator;
    std::cerr << "Tracker::Tracker|destroyed" << std::endl;
  }

  void Tracker::_trackFeatures(Frame* previous_frame_, Frame* current_frame_) {
    assert(previous_frame_ != 0);
    assert(current_frame_ != 0);

    //ds control variables
    current_frame_->points().resize(_number_of_potential_points);
    _number_of_tracked_points          = 0;
    _number_of_tracked_landmarks_close = 0;
    _number_of_tracked_landmarks_far   = 0;
    _number_of_lost_points             = 0;

    //ds retrieve point predictions on current image plane
    _getImageCoordinates(_projected_image_coordinates_left, previous_frame_, current_frame_);

    //ds prepare lost buffer
    _lost_points.resize(previous_frame_->points().size());

    //ds check state
    if (_status_previous == Frame::Localizing) {

      //ds for localization mode we have a more relaxed tracking condition
      _pixel_distance_tracking_threshold = _pixel_distance_tracking_threshold_maximum;
    } else {

      //ds narrow search limit closer to projection when we're in tracking mode
      _pixel_distance_tracking_threshold = _pixel_distance_tracking_threshold_minimum;
    }
    const real _maximum_matching_distance_tracking_point  = _framepoint_generator->matchingDistanceTrackingThreshold();
    const real _maximum_matching_distance_tracking_region = _framepoint_generator->matchingDistanceTrackingThreshold();

    //ds loop over all past points
    for (Index index_point_previous = 0; index_point_previous < previous_frame_->points().size(); ++index_point_previous) {

      //ds compute current projection points
      FramePoint* previous_point = previous_frame_->points()[index_point_previous];
      const ImageCoordinates& projection_left(_projected_image_coordinates_left[index_point_previous]);

      //ds prior grid location
      const int32_t row_projection = std::round(projection_left.y());
      const int32_t col_projection = std::round(projection_left.x());
      const int32_t row_previous = std::round(previous_point->imageCoordinatesLeft().y());
      const int32_t col_previous = std::round(previous_point->imageCoordinatesLeft().x());

      //ds exhaustive search
      int32_t pixel_distance_best = _pixel_distance_tracking_threshold;
      int32_t row_best = -1;
      int32_t col_best = -1;

//ds ------------------------------------------- STAGE 1: POINT VICINITY TRACKING
      //ds compute borders
      const int32_t row_start_point = std::max(row_projection-_range_point_tracking, static_cast<int32_t>(0));
      const int32_t row_end_point   = std::min(row_projection+_range_point_tracking, static_cast<int32_t>(_framepoint_generator->numberOfRowsImage()));
      const int32_t col_start_point = std::max(col_projection-_range_point_tracking, static_cast<int32_t>(0));
      const int32_t col_end_point   = std::min(col_projection+_range_point_tracking, static_cast<int32_t>(_framepoint_generator->numberOfColsImage()));

      //ds locate best match
      for (int32_t row_point = row_start_point; row_point < row_end_point; ++row_point) {
        for (int32_t col_point = col_start_point; col_point < col_end_point; ++col_point) {
          if (_framepoint_generator->framepointsInImage()[row_point][col_point]) {
            const int32_t pixel_distance = std::fabs(row_projection-row_point)+std::fabs(col_projection-col_point);
            const real matching_distance = cv::norm(previous_point->descriptorLeft(),
                                                    _framepoint_generator->framepointsInImage()[row_point][col_point]->descriptorLeft(),
                                                    DESCRIPTOR_NORM);

            if (pixel_distance < pixel_distance_best && matching_distance < _maximum_matching_distance_tracking_point) {
              pixel_distance_best = pixel_distance;
              row_best = row_point;
              col_best = col_point;
            }
          }
        }
      }

      //ds if we found a match
      if (pixel_distance_best < _pixel_distance_tracking_threshold) {

        //ds check if track is consistent
        if ((row_best-row_previous)*(row_best-row_previous)+(col_best-col_previous)*(col_best-col_previous) < _maximum_flow_pixels_squared) {

          //ds allocate a new point connected to the previous one
          FramePoint* current_point = _framepoint_generator->framepointsInImage()[row_best][col_best];
          current_point->setPrevious(previous_point);

          //ds set the point to the control structure
          current_frame_->points()[_number_of_tracked_points] = current_point;
          if (current_point->landmark()) {
            if (current_point->isNear()) {
              ++_number_of_tracked_landmarks_close;
            } else {
              ++_number_of_tracked_landmarks_far;
            }
          }

          const cv::Point2f point_previous(previous_point->imageCoordinatesLeft().x(), previous_point->imageCoordinatesLeft().y());
          const cv::Point2f point_current(current_point->imageCoordinatesLeft().x(), current_point->imageCoordinatesLeft().y());
          ++_number_of_tracked_points;

          //ds disable further matching and reduce search time
          _framepoint_generator->framepointsInImage()[row_best][col_best] = 0;
          continue;
        }
      }

//ds ------------------------------------------- STAGE 2: REGIONAL TRACKING
      pixel_distance_best = _pixel_distance_tracking_threshold;

      //ds compute borders
      const int32_t row_start_region = std::max(row_projection-_pixel_distance_tracking_threshold, static_cast<int32_t>(0));
      const int32_t row_end_region   = std::min(row_projection+_pixel_distance_tracking_threshold, static_cast<int32_t>(_framepoint_generator->numberOfRowsImage()));
      const int32_t col_start_region = std::max(col_projection-_pixel_distance_tracking_threshold, static_cast<int32_t>(0));
      const int32_t col_end_region   = std::min(col_projection+_pixel_distance_tracking_threshold, static_cast<int32_t>(_framepoint_generator->numberOfColsImage()));

      //ds locate best match
      for (int32_t row_region = row_start_region; row_region < row_end_region; ++row_region) {
        for (int32_t col_region = col_start_region; col_region < col_end_region; ++col_region) {
          if (_framepoint_generator->framepointsInImage()[row_region][col_region]) {

            //ds if area has not been already evaluated in previous stage
            if (row_region < row_start_point||
                row_region >= row_end_point ||
                col_region < col_start_point||
                col_region >= col_end_point ) {

              const int32_t pixel_distance = std::fabs(row_projection-row_region)+std::fabs(col_projection-col_region);
              const real matching_distance = cv::norm(previous_point->descriptorLeft(),
                                                      _framepoint_generator->framepointsInImage()[row_region][col_region]->descriptorLeft(),
                                                      DESCRIPTOR_NORM);

              if (pixel_distance < pixel_distance_best && matching_distance < _maximum_matching_distance_tracking_region) {
                pixel_distance_best = pixel_distance;
                row_best = row_region;
                col_best = col_region;
              }
            }
          }
        }
      }

      //ds if we found a match
      if (pixel_distance_best < _pixel_distance_tracking_threshold) {

        //ds check if track is consistent
        if ((row_best-row_previous)*(row_best-row_previous)+(col_best-col_previous)*(col_best-col_previous) < _maximum_flow_pixels_squared) {

          //ds allocate a new point connected to the previous one
          FramePoint* current_point = _framepoint_generator->framepointsInImage()[row_best][col_best];
          current_point->setPrevious(previous_point);

          //ds set the point to the control structure
          current_frame_->points()[_number_of_tracked_points] = current_point;
          if (current_point->landmark()) {
            if (current_point->isNear()) {
              ++_number_of_tracked_landmarks_close;
            } else {
              ++_number_of_tracked_landmarks_far;
            }
          }

          const cv::Point2f point_previous(previous_point->imageCoordinatesLeft().x(), previous_point->imageCoordinatesLeft().y());
          const cv::Point2f point_current(current_point->imageCoordinatesLeft().x(), current_point->imageCoordinatesLeft().y());
          ++_number_of_tracked_points;

          //ds disable further matching and reduce search time
          _framepoint_generator->framepointsInImage()[row_best][col_best] = 0;
          continue;
        }
      }

      //ds no match found
      if (previous_point->landmark()) {
        _lost_points[_number_of_lost_points] = previous_point;
        ++_number_of_lost_points;
      }
    }
    current_frame_->points().resize(_number_of_tracked_points);
    _lost_points.resize(_number_of_lost_points);

//    //ds info
//    std::cerr << "Tracker::trackFeatures|tracks: " << _number_of_tracked_points << "/" << previous_frame_->points().size()
//              << " landmarks close: " << _number_of_tracked_landmarks_close
//              << " landmarks far: " << _number_of_tracked_landmarks_far
//              << std::endl;
    _total_number_of_landmarks_close += _number_of_tracked_landmarks_close;
    _total_number_of_landmarks_far   += _number_of_tracked_landmarks_far;
  }

  void Tracker::_extractFeatures(Frame* frame_) {

    //ds make space for all remaining points
    frame_->points().resize(_number_of_potential_points+_number_of_lost_points_recovered);

    //ds check triangulation map for unmatched points
    Index index_point_new = _number_of_tracked_points;
    for (uint32_t row = 0; row < _framepoint_generator->numberOfRowsImage(); ++row) {
      for (uint32_t col = 0; col < _framepoint_generator->numberOfColsImage(); ++col) {
        if (_framepoint_generator->framepointsInImage()[row][col]) {

          //ds create a new point
          frame_->points()[index_point_new] = _framepoint_generator->framepointsInImage()[row][col];
          ++index_point_new;
          _framepoint_generator->framepointsInImage()[row][col] = 0;
        }
      }
    }
    frame_->points().resize(index_point_new);
//    std::cerr << "Tracker::extractFeatures|new points: " << index_point_new-_number_of_tracked_points << std::endl;
  }

  void Tracker::_getImageCoordinates(std::vector<ImageCoordinates>& projected_image_coordinates_left_,
                                    Frame* previous_frame_,
                                    const Frame* current_frame_) const {
    assert(previous_frame_ != 0);
    assert(current_frame_ != 0);

    //ds preallocation
    projected_image_coordinates_left_.resize(previous_frame_->points().size());
    const TransformMatrix3D robot_previous_to_current = current_frame_->worldToRobot()*previous_frame_->robotToWorld();
    const TransformMatrix3D robot_to_camera           = current_frame_->cameraLeft()->robotToCamera();
    const TransformMatrix3D world_to_camera           = robot_to_camera*current_frame_->worldToRobot();

    //ds compute predictions for all previous frame points
    Count number_of_visible_points = 0;
    for (FramePoint* previous_frame_point: previous_frame_->points()) {
      assert(previous_frame_point->imageCoordinatesLeft().x() >= 0);
      assert(previous_frame_point->imageCoordinatesLeft().x() <= current_frame_->cameraLeft()->imageCols());
      assert(previous_frame_point->imageCoordinatesLeft().y() >= 0);
      assert(previous_frame_point->imageCoordinatesLeft().y() <= current_frame_->cameraLeft()->imageRows());

      //ds get point into current camera - based on last track
      Vector4 point_in_camera_homogeneous(Vector4::Ones());

      //ds if we have a valid landmark at hand
      if (previous_frame_point->landmark() && previous_frame_point->landmark()->areCoordinatesValidated()) {

        //ds get point in camera frame based on landmark coordinates
        point_in_camera_homogeneous.head<3>() = world_to_camera*previous_frame_point->landmark()->coordinates();
      } else {

        //ds reproject based on last track
        point_in_camera_homogeneous.head<3>() = robot_to_camera*(robot_previous_to_current*previous_frame_point->robotCoordinates());
      }

      //ds obtain point projection on camera image plane
      PointCoordinates point_in_image_left  = current_frame_->cameraLeft()->projectionMatrix()*point_in_camera_homogeneous;

      //ds normalize point and update prediction based on landmark position: LEFT
      point_in_image_left  /= point_in_image_left.z();

      //ds check for invalid projections
      if (point_in_image_left.x() < 0 || point_in_image_left.x() > current_frame_->cameraLeft()->imageCols()       ||
          point_in_image_left.y() < 0 || point_in_image_left.y() > current_frame_->cameraLeft()->imageRows()       ) {

        //ds out of FOV
        continue;
      }

      //ds update predictions
      projected_image_coordinates_left_[number_of_visible_points] = point_in_image_left;
      previous_frame_->points()[number_of_visible_points] = previous_frame_point;
      ++number_of_visible_points;
    }
    previous_frame_->points().resize(number_of_visible_points);
    projected_image_coordinates_left_.resize(number_of_visible_points);
  }

  void Tracker::_updateLandmarks(WorldMap* context_) {
    Frame* current_frame = context_->currentFrame();

    //ds start landmark generation/update
    for (FramePoint* point: current_frame->points()) {

      //ds skip point if tracking and not mature enough to be a landmark - for localizing state this is skipped
      if (point->trackLength() < current_frame->minimumTrackLengthForLandmarkCreation()) {
        continue;
      }

      //ds initial update setup
      Landmark* landmark = point->landmark();

      //ds if the point is close to the camera
      if (point->isNear()) {

        //ds if no landmark yet - but point with depth information, create a new landmark using the given depth
        if (!landmark) {
          landmark = context_->createLandmark(current_frame->robotToWorld()*point->robotCoordinates(), point);
          point->setLandmark(landmark);
        }
        landmark->setIsNear(true);
      } else {

        //ds create a new landmark using the vision depth
        if (!landmark) {
          landmark = context_->createLandmark(current_frame->robotToWorld()*point->robotCoordinates(), point);
          point->setLandmark(landmark);
        }
        landmark->setIsNear(false);
      }
      assert(landmark != 0);

      //ds update landmark position
      landmark->update(current_frame->robotToWorld()*point->robotCoordinates(),
                       point->descriptorLeft(),
                       point->descriptorRight(),
                       point->depthMeters());

      //ds add landmarks to currently visible ones
      landmark->setIsCurrentlyTracked(true);
      context_->currentlyTrackedLandmarks().push_back(landmark);
    }
  }

  const TransformMatrix3D Tracker::compute(WorldMap* context_,
                                               const cv::Mat& intensity_image_left_,
                                               const cv::Mat& intensity_image_right_,
                                               const TransformMatrix3D& initial_guess_world_previous_to_current_) {

    //ds reset point configurations
    _number_of_tracked_points        = 0;
    _number_of_lost_points           = 0;
    _number_of_lost_points_recovered = 0;
    for (Landmark* landmark: context_->currentlyTrackedLandmarks()) {
      landmark->setIsCurrentlyTracked(false);
    }
    context_->currentlyTrackedLandmarks().clear();

    //ds retrieve odometry prior - might be identity
    TransformMatrix3D robot_to_world_current = context_->robotToWorldPrevious();
    if (context_->currentFrame()) {
      robot_to_world_current = initial_guess_world_previous_to_current_*robot_to_world_current;
    }
    context_->setRobotToWorldPrevious(robot_to_world_current);

    //ds create new frame
    Frame* current_frame = context_->createFrame(robot_to_world_current, _framepoint_generator->maximumDepthCloseMeters());
    current_frame->setCameraLeft(_camera_left);
    current_frame->setIntensityImageLeft(intensity_image_left_);
    current_frame->setCameraRight(_camera_right);
    current_frame->setIntensityImageRight(intensity_image_right_);

    //ds compute full sensory prior for the current frame
    _framepoint_generator->compute(current_frame);
    _number_of_potential_points = _framepoint_generator->numberOfAvailablePoints();

    //ds if available - attempt to track the points from the previous frame
    if (current_frame->previous()) {
      CHRONOMETER_START(tracking)
      _trackFeatures(current_frame->previous(), current_frame);
      CHRONOMETER_STOP(tracking)
    }

    //ds updated odometry change
    TransformMatrix3D world_previous_to_current(TransformMatrix3D::Identity());

    //ds check tracker status
    switch(_status) {

      //ds track lost - localizing
      case Frame::Localizing: {
        std::cerr << "Tracker::addImage|STATE: LOCALIZING" << std::endl;

        //ds if we have a previous frame
        if (current_frame->previous()) {

          //ds solve pose on frame points only
          CHRONOMETER_START(pose_optimization)
          _pose_optimizer->init(current_frame, current_frame->robotToWorld());
          _pose_optimizer->setWeightFramepoint(1);
          _pose_optimizer->converge();
          CHRONOMETER_STOP(pose_optimization)

          //ds if the pose computation is acceptable
          if (_pose_optimizer->numberOfInliers() > 2*_minimum_number_of_landmarks_to_track) {

            //ds solver deltas
            world_previous_to_current      = _pose_optimizer->robotToWorld()*current_frame->previous()->robotToWorld().inverse();
            const real delta_angular       = WorldMap::toOrientationRodrigues(world_previous_to_current.linear()).norm();
            const real delta_translational = world_previous_to_current.translation().norm();

            //ds if the posit result is significant enough
            if (delta_angular > 0.001 || delta_translational > 0.01) {

              //ds update tracker
              current_frame->setRobotToWorld(_pose_optimizer->robotToWorld());
              std::cerr << "Tracker::addImage|WARNING: using posit on frame points (experimental) inliers: " << _pose_optimizer->numberOfInliers()
                        << " outliers: " << _pose_optimizer->numberOfOutliers() << " average error: " << _pose_optimizer->totalError()/_pose_optimizer->numberOfInliers() <<  std::endl;
            } else {

              //ds keep previous solution
              current_frame->setRobotToWorld(current_frame->previous()->robotToWorld());
              world_previous_to_current = TransformMatrix3D::Identity();
            }

            //ds update previous
            context_->setRobotToWorldPrevious(current_frame->robotToWorld());
          }
        }

        //ds check if we can switch the state
        const Count number_of_good_points = current_frame->countPoints(current_frame->minimumTrackLengthForLandmarkCreation());
        if (number_of_good_points > _minimum_number_of_landmarks_to_track) {
          _updateLandmarks(context_);
          _status_previous = _status;
          _status = Frame::Tracking;
        }
        break;
      }

      //ds on the track
      case Frame::Tracking: {

        //ds compute far to close landmark ratio TODO simplify or get better logic: currently the idea is to give more weight to framepoints in case we have almost only far landmarks
        const real weight_framepoint = 1-(_number_of_tracked_landmarks_far+10*_number_of_tracked_landmarks_close)/static_cast<real>(_number_of_tracked_points);
        assert(weight_framepoint <= 1);

        //ds call pose solver
        CHRONOMETER_START(pose_optimization)
        _pose_optimizer->init(current_frame, current_frame->robotToWorld());
        _pose_optimizer->setWeightFramepoint(std::max(weight_framepoint, static_cast<real>(0.1)));
        _pose_optimizer->converge();
        CHRONOMETER_STOP(pose_optimization)

        //ds solver deltas
        Count number_of_inliers    = _pose_optimizer->numberOfInliers();
        world_previous_to_current  = _pose_optimizer->robotToWorld()*current_frame->previous()->robotToWorld().inverse();
        real delta_angular         = WorldMap::toOrientationRodrigues(world_previous_to_current.linear()).norm();
        real delta_translational   = world_previous_to_current.translation().norm();

        //ds if we don't have enough inliers - trigger fallback posit on last position
        if (number_of_inliers < _minimum_number_of_landmarks_to_track) {

          //ds reset state - also purging points to fully reinitialize the tracking
          std::cerr << "LOST TRACK due to invalid position optimization" << std::endl;
          _status_previous = Frame::Localizing;
          _status          = Frame::Localizing;
          current_frame->setStatus(_status);
          current_frame->releasePoints();
          _framepoint_generator->clearFramepointsInImage();
          context_->currentlyTrackedLandmarks().clear();

          //ds keep previous solution
          current_frame->setRobotToWorld(current_frame->previous()->robotToWorld());
          world_previous_to_current = TransformMatrix3D::Identity();
          context_->setRobotToWorldPrevious(current_frame->robotToWorld());
          return world_previous_to_current;
        }

        //ds if the posit result is significant enough
        if (delta_angular > 0.001 || delta_translational > 0.01) {

          //ds update robot pose with posit result
          current_frame->setRobotToWorld(_pose_optimizer->robotToWorld());
        } else {

          //ds keep previous solution
          current_frame->setRobotToWorld(current_frame->previous()->robotToWorld());
          world_previous_to_current = TransformMatrix3D::Identity();
        }

        //ds visualization only (we need to clear and push_back in order to not crash the gui since its decoupled - otherwise we could use resize)
        context_->currentlyTrackedLandmarks().reserve(_number_of_tracked_landmarks_far+_number_of_tracked_landmarks_close);

        //ds update current frame points
        _number_of_tracked_points = 0;
        for (Index index_point = 0; index_point < current_frame->points().size(); index_point++) {
          assert(current_frame->points()[index_point]->previous());

          //ds buffer current landmark
          Landmark* landmark = current_frame->points()[index_point]->landmark();

          //ds points without landmarks are always kept
          if (landmark == 0) {
            current_frame->points()[_number_of_tracked_points] = current_frame->points()[index_point];
            ++_number_of_tracked_points;
          }

          //ds keep the point if it has been skipped (due to insufficient maturity or invalidity) or is an inlier
          else if (_pose_optimizer->errors()[index_point] == -1 || _pose_optimizer->inliers()[index_point]) {
            current_frame->points()[_number_of_tracked_points] = current_frame->points()[index_point];
            ++_number_of_tracked_points;
          }
        }
        assert(context_->currentlyTrackedLandmarks().size() <= _number_of_tracked_landmarks_far+_number_of_tracked_landmarks_close);

        //ds update point container
        current_frame->points().resize(_number_of_tracked_points);
        assert(_number_of_tracked_points >= number_of_inliers);

        //ds recover lost points based on updated pose
        CHRONOMETER_START(point_recovery)
        _recoverPoints(context_);
        CHRONOMETER_STOP(point_recovery)

        //ds update tracks
        context_->setRobotToWorldPrevious(current_frame->robotToWorld());
        CHRONOMETER_START(landmark_optimization)
        _updateLandmarks(context_);
        CHRONOMETER_STOP(landmark_optimization)
        _status_previous = _status;
        _status          = Frame::Tracking;
        break;
      }

      default: {
        assert(false);
        break;
      }
    }

    assert(current_frame != 0);
    CHRONOMETER_START(track_creation)
    _extractFeatures(current_frame);
    CHRONOMETER_STOP(track_creation)
    current_frame->setStatus(_status);

    //ds done
    _total_number_of_tracked_points += _number_of_tracked_points;
    return world_previous_to_current;
  }

  void Tracker::_recoverPoints(WorldMap* context_) {
    Frame* current_frame = context_->currentFrame();

    //ds precompute transforms
    const TransformMatrix3D world_to_camera_left       = _camera_left->robotToCamera()*current_frame->worldToRobot();
    const ProjectionMatrix& projection_matrix_left     = _camera_left->projectionMatrix();
    const ProjectionMatrix& projection_matrix_right    = _camera_right->projectionMatrix();
    std::vector<cv::KeyPoint> keypoint_buffer_left(1);
    std::vector<cv::KeyPoint> keypoint_buffer_right(1);

    //ds recover lost landmarks
    Index index_lost_point_recovered = _number_of_tracked_points;
    current_frame->points().resize(_number_of_tracked_points+_number_of_lost_points);
    for (FramePoint* point_previous: _lost_points) {

      //ds get point into current camera - based on last track
      Vector4 point_in_camera_homogeneous(Vector4::Ones());

      //ds if we have a landmark at hand
      if (point_previous->landmark()) {

        //ds get point in camera frame based on landmark coordinates
        point_in_camera_homogeneous.head<3>() = world_to_camera_left*point_previous->landmark()->coordinates();
      } else {

        //ds currently only landmark recovery supported
        continue;
      }

      //ds obtain point projection on camera image plane
      PointCoordinates point_in_image_left  = projection_matrix_left*point_in_camera_homogeneous;
      PointCoordinates point_in_image_right = projection_matrix_right*point_in_camera_homogeneous;

      //ds normalize point and update prediction based on landmark position: LEFT
      point_in_image_left  /= point_in_image_left.z();
      point_in_image_right /= point_in_image_right.z();

      //ds check for invalid projections
      if (point_in_image_left.x() < 0 || point_in_image_left.x() > _camera_cols  ||
          point_in_image_right.x() < 0 || point_in_image_right.x() > _camera_cols||
          point_in_image_left.y() < 0 || point_in_image_left.y() > _camera_rows  ) {

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
          projection_left.x >= _camera_cols-regional_border_center-1 ||
          projection_left.y <= regional_border_center+1              ||
          projection_left.y >= _camera_rows-regional_border_center-1 ||
          projection_right.x <= regional_border_center+1             ||
          projection_right.x >= _camera_cols-regional_border_center-1) {

        //ds skip complete tracking
        continue;
      }

      //ds extraction regions
      const cv::Point2f corner_left(projection_left-offset_keypoint_half);
      const cv::Rect_<float> region_of_interest_left(corner_left.x, corner_left.y, regional_full_height, regional_full_height);
      const cv::Point2f corner_right(projection_right-offset_keypoint_half);
      const cv::Rect_<float> region_of_interest_right(corner_right.x, corner_right.y, regional_full_height, regional_full_height);

      //ds extract descriptors at this position: LEFT
      keypoint_buffer_left[0] = point_previous->keypointLeft();
      keypoint_buffer_left[0].pt = offset_keypoint_half;
      cv::Mat descriptor_left;
      _framepoint_generator->descriptorExtractor()->compute(current_frame->intensityImageLeft()(region_of_interest_left), keypoint_buffer_left, descriptor_left);
      if (descriptor_left.rows == 0) {
        continue;
      }
      keypoint_buffer_left[0].pt += corner_left;

      //ds extract descriptors at this position: RIGHT
      keypoint_buffer_right[0] = point_previous->keypointRight();
      keypoint_buffer_right[0].pt = offset_keypoint_half;
      cv::Mat descriptor_right;
      _framepoint_generator->descriptorExtractor()->compute(current_frame->intensityImageRight()(region_of_interest_right), keypoint_buffer_right, descriptor_right);
      if (descriptor_right.rows == 0) {
        continue;
      }
      keypoint_buffer_right[0].pt += corner_right;

      if (cv::norm(point_previous->descriptorLeft(), descriptor_left, DESCRIPTOR_NORM) < _framepoint_generator->matchingDistanceTrackingThreshold()      &&
          cv::norm(point_previous->descriptorRight(), descriptor_right, DESCRIPTOR_NORM) < _framepoint_generator->matchingDistanceTrackingThreshold()) {
        try {

          //ds triangulate point
          const PointCoordinates camera_coordinates(_framepoint_generator->getCoordinatesInCameraLeft(keypoint_buffer_left[0].pt, keypoint_buffer_right[0].pt));

          //ds allocate a new point connected to the previous one
          FramePoint* current_point = current_frame->create(keypoint_buffer_left[0],
                                                                    descriptor_left,
                                                                    keypoint_buffer_right[0],
                                                                    descriptor_right,
                                                                    camera_coordinates,
                                                                    point_previous);
          current_point->setRobotCoordinates(current_frame->cameraLeft()->cameraToRobot()*camera_coordinates);

          //ds set the point to the control structure
          current_frame->points()[index_lost_point_recovered] = current_point;
          ++index_lost_point_recovered;
        } catch (const ExceptionTriangulation& /*exception_*/) {}
      }
    }
    _number_of_lost_points_recovered = index_lost_point_recovered-_number_of_tracked_points;
    _number_of_tracked_points = index_lost_point_recovered;
    current_frame->points().resize(_number_of_tracked_points);
//    std::cerr << "Tracker::recoverPoints|recovered points: " << _number_of_lost_points_recovered << "/" << _number_of_lost_points << std::endl;
  }
}
