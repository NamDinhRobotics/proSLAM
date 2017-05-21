#include "base_tracker.h"

namespace proslam {
  using namespace srrg_core;

  BaseTracker::BaseTracker(): _camera_left(0),
                              _number_of_rows_image(0),
                              _number_of_cols_image(0),
                              _intensity_image_left(0),
                              _context(0),
                              _pose_optimizer(0),
                              _framepoint_generator(0),
                              _pixel_distance_tracking_threshold(4*4),
                              _number_of_rows_bin(0),
                              _number_of_cols_bin(0),
                              _bin_map_left(0),
                              _enable_keypoint_binning(false),
                              _parameters(0),
                              _has_odometry(false) {
    std::cerr << "BaseTracker::BaseTracker|constructed" << std::endl;
  }

  void BaseTracker::configure(BaseTrackerParameters* parameters_) {
    std::cerr << "BaseTracker::setup|configuring" << std::endl;
    _parameters = parameters_;
    assert(_camera_left);
    assert(_pose_optimizer);
    _number_of_rows_image = _camera_left->imageRows();
    _number_of_cols_image = _camera_left->imageCols();
    _motion_previous_to_current_robot.setIdentity();
    _pose_optimizer->setMaximumDepthNearMeters(_framepoint_generator->maximumDepthNearMeters());
    _pose_optimizer->setMaximumDepthFarMeters(_framepoint_generator->maximumDepthFarMeters());

    //ds clear buffers
    _lost_points.clear();
    _projected_image_coordinates_left.clear();

    //ds binning configuration
    _number_of_cols_bin = std::floor(static_cast<real>(_number_of_cols_image)/_parameters->bin_size_pixels)+1;
    _number_of_rows_bin = std::floor(static_cast<real>(_number_of_rows_image)/_parameters->bin_size_pixels)+1;
    _framepoint_generator->setTargetNumberOfKeyoints(_parameters->ratio_keypoints_to_bins*_number_of_cols_bin*_number_of_rows_bin);

    //ds allocate and initialize bin grid
    _bin_map_left = new FramePoint**[_number_of_rows_bin];
    for (Index row = 0; row < _number_of_rows_bin; ++row) {
      _bin_map_left[row] = new FramePoint*[_number_of_cols_bin];
      for (Index col = 0; col < _number_of_cols_bin; ++col) {
        _bin_map_left[row][col] = 0;
      }
    }

    //ds print tracker configuration (with dynamic type of parameters)
    std::cerr << "BaseTracker::setup|configured" << std::endl;
  }

  //ds dynamic cleanup
  BaseTracker::~BaseTracker() {
    std::cerr << "BaseTracker::BaseTracker|destroying" << std::endl;
    
    //ds clear buffers
    _lost_points.clear();
    _projected_image_coordinates_left.clear();

    //ds free bin map
    for (Count row = 0; row < _number_of_rows_bin; ++row) {
      delete[] _bin_map_left[row];
    }
    delete[] _bin_map_left;

    //ds free dynamics
    delete _framepoint_generator;
    delete _pose_optimizer;
    std::cerr << "BaseTracker::BaseTracker|destroyed" << std::endl;
  }

  //ds creates a new Frame for the given images, retrieves the correspondences relative to the previous Frame, optimizes the current frame pose and updates landmarks
  void BaseTracker::compute() {
    assert(_camera_left);
    assert(_context);
    assert(_intensity_image_left);
    
    //ds reset point configurations
    _number_of_tracked_points        = 0;
    _number_of_lost_points           = 0;
    _number_of_lost_points_recovered = 0;
    for (Landmark* landmark: _context->currentlyTrackedLandmarks()) {
      landmark->setIsCurrentlyTracked(false);
    }
    _context->currentlyTrackedLandmarks().clear();

    //gg if we have an odometry we use it as initial guess
    if (_has_odometry) {
      if (!_context->currentFrame()){
        _previous_odometry = _odometry;
      }
      TransformMatrix3D odom_delta = _previous_odometry.inverse()*_odometry;
      _motion_previous_to_current_robot = odom_delta;
      _previous_odometry = _odometry;
    }

    //ds retrieve estimate by applying the constant velocity motion model
    if (_context->currentFrame()) {
      _context->setRobotToWorld(_context->robotToWorld()*_motion_previous_to_current_robot);
    }

    //ds create new frame
    Frame* current_frame = _createFrame();
 
    //ds compute full sensory prior for the current frame
    _framepoint_generator->compute(current_frame);
    _number_of_potential_points = _framepoint_generator->numberOfAvailablePoints();

    //ds if available - attempt to track the points from the previous frame
    if (current_frame->previous()) {

      //ds enable binning by default
      CHRONOMETER_START(tracking);
      _trackFramepoints(current_frame->previous(), current_frame);
      CHRONOMETER_STOP(tracking);
    }

    //ds check tracker status
    switch(_status) {

      //ds track lost - localizing
      case Frame::Localizing: {
        std::cerr << "BaseTracker::compute|STATE: LOCALIZING" << std::endl;

        //ds if we have a previous frame
        if (current_frame->previous()) {

          //ds solve pose on frame points only
          CHRONOMETER_START(pose_optimization);
          _pose_optimizer->initialize(current_frame, current_frame->robotToWorld());
          _pose_optimizer->setWeightFramepoint(1);
          _pose_optimizer->converge();
          CHRONOMETER_STOP(pose_optimization);

          //ds if the pose computation is acceptable
          if (_pose_optimizer->numberOfInliers() > _parameters->minimum_number_of_landmarks_to_track) {

            //ds compute resulting motion
            _motion_previous_to_current_robot = current_frame->previous()->worldToRobot()*_pose_optimizer->robotToWorld();
            const real delta_angular          = WorldMap::toOrientationRodrigues(_motion_previous_to_current_robot.linear()).norm();
            const real delta_translational    = _motion_previous_to_current_robot.translation().norm();

              //ds if the posit result is significant enough
            if (delta_angular > _parameters->minimum_delta_angular_for_movement || delta_translational > _parameters->minimum_delta_translational_for_movement) {

              //ds update tracker
              current_frame->setRobotToWorld(_pose_optimizer->robotToWorld());
              std::cerr << "BaseTracker::compute|WARNING: using posit on frame points (experimental) inliers: " << _pose_optimizer->numberOfInliers()
                        << " outliers: " << _pose_optimizer->numberOfOutliers() << " average error: " << _pose_optimizer->totalError()/_pose_optimizer->numberOfInliers() <<  std::endl;
            } else {

              //ds keep previous solution
              current_frame->setRobotToWorld(current_frame->previous()->robotToWorld());
              _motion_previous_to_current_robot = TransformMatrix3D::Identity();
            }

            //ds update context position
            _context->setRobotToWorld(current_frame->robotToWorld());
          }
        }

        //ds check if we can switch the state
        const Count number_of_good_points = current_frame->countPoints(current_frame->minimumTrackLengthForLandmarkCreation());
        if (number_of_good_points > _parameters->minimum_number_of_landmarks_to_track) {

          //ds trigger landmark creation and framepoint update
          _updateLandmarks(_context, current_frame);
          _status_previous = _status;
          _status = Frame::Tracking;
        } else {

          //ds just trigger framepoint updates
          current_frame->updatePoints();
        }
        break;
      }

      //ds on the track
      case Frame::Tracking: {

        //ds compute ratio between landmarks and tracked points
        const real percentage_landmarks = static_cast<real>(_number_of_tracked_landmarks_far+_number_of_tracked_landmarks_close)/_number_of_tracked_points;
        if (percentage_landmarks < 0.1) {
          std::cerr << "BaseTracker::compute|WARNING: low percentage of tracked landmarks over framepoints: " << percentage_landmarks
                    << " (" << _number_of_tracked_landmarks_far+_number_of_tracked_landmarks_close << "/" << _number_of_tracked_points << ")" << std::endl;
        }

        //ds compute ratio between close and far landmarks
        const real percentage_of_close_landmarks = _number_of_tracked_landmarks_close/static_cast<real>(_number_of_tracked_landmarks_far+_number_of_tracked_landmarks_close);
        if (percentage_of_close_landmarks < 0.1) {
          std::cerr << "BaseTracker::compute|WARNING: low percentage of close landmarks available: " << percentage_of_close_landmarks
                    << " (" << _number_of_tracked_landmarks_far << "/" << _number_of_tracked_landmarks_far+_number_of_tracked_landmarks_close << ")" << std::endl;
        }

        //ds derive framepoint weight for current optimization
        const real weight_framepoint = percentage_of_close_landmarks*(1-percentage_landmarks);
        assert(weight_framepoint <= 1);

//        //ds compute far to close landmark ratio TODO simplify or get better logic: currently the idea is to give more weight to framepoints in case we have almost only far landmarks
//        const real weight_framepoint = 1-(_number_of_tracked_landmarks_far+2*_number_of_tracked_landmarks_close)/static_cast<real>(_number_of_tracked_points);
//        assert(weight_framepoint <= 1);

        //ds call pose solver
        CHRONOMETER_START(pose_optimization)
        _pose_optimizer->initialize(current_frame, current_frame->robotToWorld());
        _pose_optimizer->setWeightFramepoint(std::max(weight_framepoint, static_cast<real>(0.01)));
        _pose_optimizer->converge();
        CHRONOMETER_STOP(pose_optimization)

        //ds solver deltas
        const Count& number_of_inliers = _pose_optimizer->numberOfInliers();

        _motion_previous_to_current_robot = current_frame->previous()->worldToRobot()*_pose_optimizer->robotToWorld();
        const real delta_angular          = WorldMap::toOrientationRodrigues(_motion_previous_to_current_robot.linear()).norm();
        const real delta_translational    = _motion_previous_to_current_robot.translation().norm();

        //ds if we have enough inliers in the pose optimization
        if (number_of_inliers > _parameters->minimum_number_of_landmarks_to_track) {

          //ds if the posit result is significant enough
          if (delta_angular > _parameters->minimum_delta_angular_for_movement || delta_translational > _parameters->minimum_delta_translational_for_movement) {

            //ds update robot pose with posit result
            current_frame->setRobotToWorld(_pose_optimizer->robotToWorld());
          } else {

            //ds keep previous solution
            current_frame->setRobotToWorld(current_frame->previous()->robotToWorld());
            _motion_previous_to_current_robot = TransformMatrix3D::Identity();
          }

          //ds visualization only (we need to clear and push_back in order to not crash the gui since its decoupled - otherwise we could use resize)
          _context->currentlyTrackedLandmarks().reserve(_number_of_tracked_landmarks_far+_number_of_tracked_landmarks_close);

          //ds prune current frame points
          _pruneFramepoints(current_frame);
          assert(_context->currentlyTrackedLandmarks().size() <= _number_of_tracked_landmarks_far+_number_of_tracked_landmarks_close);
          assert(_number_of_tracked_points >= number_of_inliers);

          //ds recover lost points based on updated pose
          CHRONOMETER_START(point_recovery)
          _recoverPoints(current_frame);
          CHRONOMETER_STOP(point_recovery)

          //ds update tracks
          _context->setRobotToWorld(current_frame->robotToWorld());
          CHRONOMETER_START(landmark_optimization)
          _updateLandmarks(_context, current_frame);
          CHRONOMETER_STOP(landmark_optimization)
          _status_previous = _status;
          _status          = Frame::Tracking;
        } else {

          //ds reset state
          std::printf("BaseTracker::compute|WARNING: LOST TRACK due to invalid position optimization at frame [%06lu]\n", current_frame->identifier());
          _status_previous = Frame::Localizing;
          _status          = Frame::Localizing;
          current_frame->setStatus(_status);

          //ds stick to previous solution - simulating a fresh start
          current_frame->setRobotToWorld(current_frame->previous()->robotToWorld());
          _motion_previous_to_current_robot = TransformMatrix3D::Identity();
          _number_of_lost_points_recovered = 0;
          _number_of_tracked_points        = 0;

          //ds reset frame in world context, triggering a restart of the pipeline
          _context->breakTrack(current_frame);

          //ds release all framepoints currently in the bin - enabling a full fresh addition in the add framepoints phase
          for (Index row_bin = 0; row_bin < _number_of_rows_bin; ++row_bin) {
            for (Index col_bin = 0; col_bin < _number_of_cols_bin; ++col_bin) {
              delete _bin_map_left[row_bin][col_bin];
              _bin_map_left[row_bin][col_bin] = 0;
            }
          }
        }
        break;
      }

      default: {
        assert(false);
        break;
      }
    }

    //ds add new framepoints
    assert(current_frame != 0);
    CHRONOMETER_START(track_creation)
    _addNewFramepoints(current_frame);
    CHRONOMETER_STOP(track_creation)
    current_frame->setStatus(_status);

    //ds done
    _total_number_of_tracked_points += _number_of_tracked_points;
  }

  //ds retrieves framepoint correspondences between previous and current frame
  void BaseTracker::_trackFramepoints(Frame* previous_frame_, Frame* current_frame_) {
    assert(previous_frame_);
    assert(current_frame_);
    _enable_keypoint_binning = true;

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
      _pixel_distance_tracking_threshold = _parameters->maximum_threshold_distance_tracking_pixels;
    } else {

      //ds narrow search limit closer to projection when we're in tracking mode
      _pixel_distance_tracking_threshold = _parameters->minimum_threshold_distance_tracking_pixels;
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
      const int32_t row_previous   = std::round(previous_point->imageCoordinatesLeft().y());
      const int32_t col_previous   = std::round(previous_point->imageCoordinatesLeft().x());

      //ds exhaustive search
      int32_t pixel_distance_best = _pixel_distance_tracking_threshold;
      int32_t row_best = -1;
      int32_t col_best = -1;

      //ds ------------------------------------------- STAGE 1: POINT VICINITY TRACKING
      //ds compute borders
      const int32_t row_start_point = std::max(row_projection-_parameters->range_point_tracking, static_cast<int32_t>(0));
      const int32_t row_end_point   = std::min(row_projection+_parameters->range_point_tracking, static_cast<int32_t>(_framepoint_generator->numberOfRowsImage()));
      const int32_t col_start_point = std::max(col_projection-_parameters->range_point_tracking, static_cast<int32_t>(0));
      const int32_t col_end_point   = std::min(col_projection+_parameters->range_point_tracking, static_cast<int32_t>(_framepoint_generator->numberOfColsImage()));

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
        if ((row_best-row_previous)*(row_best-row_previous)+(col_best-col_previous)*(col_best-col_previous) < _parameters->maximum_distance_tracking_pixels) {
          _addTrack(previous_point, current_frame_, row_best, col_best);
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
        if ((row_best-row_previous)*(row_best-row_previous)+(col_best-col_previous)*(col_best-col_previous) < _parameters->maximum_distance_tracking_pixels) {
          _addTrack(previous_point, current_frame_, row_best, col_best);
          continue;
        }
      }

      //ds no match found - if landmark - and not too many recoveries
      if (previous_point->landmark() && previous_point->landmark()->numberOfRecoveries() < _parameters->maximum_number_of_landmark_recoveries) {
        _lost_points[_number_of_lost_points] = previous_point;
        ++_number_of_lost_points;
      }
    }
    current_frame_->points().resize(_number_of_tracked_points);
    _lost_points.resize(_number_of_lost_points);

    //    //ds info
    //    std::cerr << "BaseTracker::trackFeatures|tracks: " << _number_of_tracked_points << "/" << previous_frame_->points().size()
    //              << " landmarks close: " << _number_of_tracked_landmarks_close
    //              << " landmarks far: " << _number_of_tracked_landmarks_far
    //              << std::endl;
    _total_number_of_landmarks_close += _number_of_tracked_landmarks_close;
    _total_number_of_landmarks_far   += _number_of_tracked_landmarks_far;
  }

  void BaseTracker::_addTrack(FramePoint* framepoint_previous_, Frame* current_frame_, const Count& row_, const Count& col_) {

    //ds allocate a new point connected to the previous one
    FramePoint* current_point = _framepoint_generator->framepointsInImage()[row_][col_];
    current_point->setPrevious(framepoint_previous_);

    //ds set the point to the control structure
    current_frame_->points()[_number_of_tracked_points] = current_point;
    if (current_point->landmark()) {
      if (current_point->isNear()) {
        ++_number_of_tracked_landmarks_close;
      } else {
        ++_number_of_tracked_landmarks_far;
      }
    }
    ++_number_of_tracked_points;

    //ds determine bin index of the current point
    const Count row_bin = std::floor(static_cast<real>(row_)/_parameters->bin_size_pixels);
    const Count col_bin = std::floor(static_cast<real>(col_)/_parameters->bin_size_pixels);

    //ds occupy corresponding bin
    _bin_map_left[row_bin][col_bin] = current_point;

    //ds disable further matching and reduce search time
    _framepoint_generator->framepointsInImage()[row_][col_] = 0;
  }

  //ds adds new framepoints to the provided frame (picked from the pool of the _framepoint_generator)
  void BaseTracker::_addNewFramepoints(Frame* frame_) {

    //ds make space for all remaining points
    frame_->points().resize(_number_of_potential_points+_number_of_lost_points_recovered);
    Index index_point_new = _number_of_tracked_points;

    //ds buffer current pose
    const TransformMatrix3D& frame_to_world = frame_->robotToWorld();

    //ds if binning is desired
    if (_enable_keypoint_binning) {

      //ds check triangulation map for unmatched points and fill them into the bin map
      for (Index row = 0; row < _framepoint_generator->numberOfRowsImage(); ++row) {
        for (Index col = 0; col < _framepoint_generator->numberOfColsImage(); ++col) {
          if (_framepoint_generator->framepointsInImage()[row][col]) {

            //ds determine bin index of the current point
            const Index row_bin = std::floor(static_cast<real>(row)/_parameters->bin_size_pixels);
            const Index col_bin = std::floor(static_cast<real>(col)/_parameters->bin_size_pixels);
            assert(row_bin < _number_of_rows_bin);
            assert(col_bin < _number_of_cols_bin);

            //ds if the bin is empty
            if (!_bin_map_left[row_bin][col_bin]) {

              //ds set the current point
              _bin_map_left[row_bin][col_bin] = _framepoint_generator->framepointsInImage()[row][col];
            }

            //ds or if the bin is not occupied by a tracked point and we have a point with higher keypoint response
            else if (!_bin_map_left[row_bin][col_bin]->previous() &&
                     _framepoint_generator->framepointsInImage()[row][col]->keypointLeft().response > _bin_map_left[row_bin][col_bin]->keypointLeft().response) {

              //ds drop the previous framepoint sitting in this bin
              delete _bin_map_left[row_bin][col_bin];

              //ds set the current point
              _bin_map_left[row_bin][col_bin] = _framepoint_generator->framepointsInImage()[row][col];
            } else {

              //ds drop the framepoint
              delete _framepoint_generator->framepointsInImage()[row][col];
            }

            //ds always free point from input grid
            _framepoint_generator->framepointsInImage()[row][col] = 0;
          }
        }
      }

      //ds collect new framepoints from bin map and clear it simultaneously - TODO this could be done faster using bookkeeping
      for (Index row_bin = 0; row_bin < _number_of_rows_bin; ++row_bin) {
        for (Index col_bin = 0; col_bin < _number_of_cols_bin; ++col_bin) {
          if (_bin_map_left[row_bin][col_bin]) {

            //ds if not occupied by a tracked point
            if (!_bin_map_left[row_bin][col_bin]->previous()) {

              //ds assign the new point
              frame_->points()[index_point_new] = _bin_map_left[row_bin][col_bin];

              //ds update framepoint world position using the current pose estimate
              frame_->points()[index_point_new]->setWorldCoordinates(frame_to_world*frame_->points()[index_point_new]->robotCoordinates());
              ++index_point_new;
            }

            //ds empty bin
            _bin_map_left[row_bin][col_bin] = 0;
          }
        }
      }
    } else {

      //ds check triangulation map for unmatched points and add them directly
      for (Index row = 0; row < _framepoint_generator->numberOfRowsImage(); ++row) {
        for (Index col = 0; col < _framepoint_generator->numberOfColsImage(); ++col) {
          if (_framepoint_generator->framepointsInImage()[row][col]) {

            //ds assign the new point
            frame_->points()[index_point_new] = _framepoint_generator->framepointsInImage()[row][col];

            //ds update framepoint world position using the current pose estimate
            frame_->points()[index_point_new]->setWorldCoordinates(frame_to_world*frame_->points()[index_point_new]->robotCoordinates());
            ++index_point_new;

            //ds free point from input grid
            _framepoint_generator->framepointsInImage()[row][col] = 0;
          }
        }
      }
    }
    frame_->points().resize(index_point_new);
//    std::cerr << "BaseTracker::extractFeatures|new points: " << index_point_new-_number_of_tracked_points << " (tracked: " << _number_of_tracked_points << ")" << std::endl;
  }

  //ds retrieves framepoint projections as image coordinates in a vector (at the same time removing points with invalid projections)
  void BaseTracker::_getImageCoordinates(std::vector<ImageCoordinates>& projected_image_coordinates_left_, Frame* previous_frame_, const Frame* current_frame_) const {
    assert(previous_frame_ != 0);
    assert(current_frame_ != 0);

    //ds preallocation
    projected_image_coordinates_left_.resize(previous_frame_->points().size());
    const TransformMatrix3D world_to_camera = current_frame_->cameraLeft()->robotToCamera()*current_frame_->worldToRobot();

    //ds compute predictions for all previous frame points
    Count number_of_visible_points = 0;
    for (FramePoint* previous_frame_point: previous_frame_->points()) {
      assert(previous_frame_point->imageCoordinatesLeft().x() >= 0);
      assert(previous_frame_point->imageCoordinatesLeft().x() <= _number_of_cols_image);
      assert(previous_frame_point->imageCoordinatesLeft().y() >= 0);
      assert(previous_frame_point->imageCoordinatesLeft().y() <= _number_of_rows_image);

      //ds get point into current camera - based on last track
      Vector3 point_in_camera;

      //ds if we have a valid landmark at hand
      if (previous_frame_point->landmark() && previous_frame_point->landmark()->areCoordinatesValidated()) {

        //ds get point in camera frame based on landmark coordinates
        point_in_camera = world_to_camera*previous_frame_point->landmark()->coordinates();
      } else {

        //ds reproject based on last track
        point_in_camera = world_to_camera*previous_frame_point->worldCoordinates();
      }

      //ds obtain point projection on camera image plane
      PointCoordinates point_in_image_left = current_frame_->cameraLeft()->cameraMatrix()*point_in_camera;

      //ds normalize point and update prediction based on landmark position: LEFT
      point_in_image_left /= point_in_image_left.z();

      //ds check for invalid projections
      if (point_in_image_left.x() < 0 || point_in_image_left.x() > _number_of_cols_image||
          point_in_image_left.y() < 0 || point_in_image_left.y() > _number_of_rows_image) {

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

  //ds prunes invalid framespoints after pose optimization
  void BaseTracker::_pruneFramepoints(Frame* frame_) {

    //ds update current frame points
    _number_of_tracked_points = 0;
    for (Index index_point = 0; index_point < frame_->points().size(); index_point++) {
      assert(frame_->points()[index_point]->previous());

      //ds buffer current landmark
      Landmark* landmark = frame_->points()[index_point]->landmark();

      //ds points without landmarks are always kept
      if (landmark == 0) {
        frame_->points()[_number_of_tracked_points] = frame_->points()[index_point];
        ++_number_of_tracked_points;
      }

      //ds keep the point if it has been skipped (due to insufficient maturity or invalidity) or is an inlier
      else if (_pose_optimizer->errors()[index_point] == -1 || _pose_optimizer->inliers()[index_point]) {

        //ds reset recovery count
        landmark->setNumberOfRecoveries(0);

        //ds add the point
        frame_->points()[_number_of_tracked_points] = frame_->points()[index_point];
        ++_number_of_tracked_points;
      }
    }
    frame_->points().resize(_number_of_tracked_points);
  }

  //ds updates existing or creates new landmarks for framepoints of the provided frame
  void BaseTracker::_updateLandmarks(WorldMap* context_, Frame* frame_) {

    //ds buffer current pose
    const TransformMatrix3D& frame_to_world = frame_->robotToWorld();

    //ds start landmark generation/update
    for (FramePoint* point: frame_->points()) {
      point->setWorldCoordinates(frame_to_world*point->robotCoordinates());

      //ds skip point if tracking and not mature enough to be a landmark - for localizing state this is skipped
      if (point->trackLength() < frame_->minimumTrackLengthForLandmarkCreation()) {
        continue;
      }

      //ds initial update setup
      Landmark* landmark = point->landmark();

      //ds if there's no landmark yet
      if (!landmark) {

        //ds create a landmark and associate it with the current framepoint
        landmark = context_->createLandmark(point);
        assert(landmark);
        point->setLandmark(landmark);
      }

      //ds check if the current landmark position is near or far to the camera
      if (point->isNear()) {
        landmark->setIsNear(true);
      } else {
        landmark->setIsNear(false);
      }

      //ds update landmark position based on current point
      landmark->update(point);

      //ds add landmarks to currently visible ones
      landmark->setIsCurrentlyTracked(true);
      context_->currentlyTrackedLandmarks().push_back(landmark);
    }
  }
}
