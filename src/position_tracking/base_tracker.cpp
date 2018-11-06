#include "base_tracker.h"
#include "aligners/stereouv_aligner.h"

namespace proslam {
using namespace srrg_core;

BaseTracker::BaseTracker(BaseTrackerParameters* parameters_): _parameters(parameters_),
                                                              _camera_left(0),
                                                              _projection_tracking_distance_pixels(parameters_->maximum_projection_tracking_distance_pixels),
                                                              _intensity_image_left(0),
                                                              _context(0),
                                                              _pose_optimizer(0),
                                                              _framepoint_generator(0),
                                                              _number_of_rows_bin(0),
                                                              _number_of_cols_bin(0),
                                                              _bin_map_left(0),
                                                              _has_odometry(false) {
  LOG_DEBUG(std::cerr << "BaseTracker::BaseTracker|constructed" << std::endl)
}

void BaseTracker::configure() {
  LOG_DEBUG(std::cerr << "BaseTracker::configure|configuring" << std::endl)
  assert(_camera_left);
  assert(_pose_optimizer);
  _previous_to_current_camera.setIdentity();
  _pose_optimizer->setMaximumDepthNearMeters(_framepoint_generator->maximumDepthNearMeters());
  _pose_optimizer->setMaximumDepthFarMeters(_framepoint_generator->maximumDepthFarMeters());

  //ds clear buffers
  _lost_points.clear();

  //ds binning configuration
  _number_of_cols_bin = std::floor(static_cast<real>(_camera_left->numberOfImageCols())/_parameters->bin_size_pixels)+1;
  _number_of_rows_bin = std::floor(static_cast<real>(_camera_left->numberOfImageRows())/_parameters->bin_size_pixels)+1;
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
  LOG_DEBUG(std::cerr << "BaseTracker::configure|number of horizontal bins: " << _number_of_cols_bin << " size: " << _parameters->bin_size_pixels << std::endl)
  LOG_DEBUG(std::cerr << "BaseTracker::configure|number of vertical bins: " << _number_of_rows_bin << " size: " << _parameters->bin_size_pixels << std::endl)
  LOG_DEBUG(std::cerr << "BaseTracker::configure|configured" << std::endl)
}

//ds dynamic cleanup
BaseTracker::~BaseTracker() {
  LOG_DEBUG(std::cerr << "BaseTracker::~BaseTracker|destroying" << std::endl)

  //ds clear buffers
  _lost_points.clear();

  //ds free bin map
  for (Count row = 0; row < _number_of_rows_bin; ++row) {
    delete[] _bin_map_left[row];
  }
  delete[] _bin_map_left;

  //ds free dynamics
  delete _framepoint_generator;
  delete _pose_optimizer;
  LOG_DEBUG(std::cerr << "BaseTracker::~BaseTracker|destroyed" << std::endl)
}

void BaseTracker::compute() {
  assert(_camera_left);
  assert(_context);
  assert(_intensity_image_left);

  //ds reset point configurations
  _number_of_tracked_points   = 0;
  _number_of_lost_points      = 0;
  _number_of_recovered_points = 0;
  _context->currentlyTrackedLandmarks().clear();

  //ds relative camera motion guess
  TransformMatrix3D previous_to_current = TransformMatrix3D::Identity();

  //ds check if initial guess can be refined with a motion model or other input
  switch(_parameters->motion_model) {

    //ds use previous motion as initial guess for current motion
    case Parameters::MotionModel::CONSTANT_VELOCITY: {
      previous_to_current = _previous_to_current_camera;
      break;
    }

    //ds camera odometry as motion guess
    case Parameters::MotionModel::CAMERA_ODOMETRY: {
      if (!_context->currentFrame()){
        _previous_odometry = _odometry;
      }
      TransformMatrix3D odom_delta = _odometry.inverse()*_previous_odometry;
      _previous_odometry           = _odometry;
      previous_to_current          = odom_delta;
      break;
    }

    //ds no guess (identity)
    case Parameters::MotionModel::NONE: {
      previous_to_current = TransformMatrix3D::Identity();
      break;
    }

    //ds invalid setting
    default: {
      throw std::runtime_error("motion model undefined");
    }
  }

  //ds create new frame
  Frame* current_frame = _createFrame();
  current_frame->setStatus(_status);

  //ds compute all framepoints in the current frame
  _framepoint_generator->compute(current_frame);
  _number_of_potential_points = _framepoint_generator->numberOfAvailablePoints();

  //ds if available - attempt to track the points from the previous frame
  Frame* previous_frame = current_frame->previous();
  if (previous_frame) {

    //ds see if we're tracking by appearance (expecting large photometric displacements between images)
    //ds always do it for the first 2 frames (repeats after track is lost)
    bool track_by_appearance = (_status == Frame::Localizing);

    //ds search point tracks
    CHRONOMETER_START(tracking);
    _track(previous_frame, current_frame, previous_to_current, track_by_appearance);
    CHRONOMETER_STOP(tracking);
  }

  //ds check previous tracker status
  switch(_status) {

    //ds localization mode - always running on tracking by appearance with maximum window size
    case Frame::Localizing: {
      LOG_INFO(std::cerr << "BaseTracker::compute|STATE: LOCALIZING|current tracks: "
                         << _number_of_tracked_points << "/" << _number_of_potential_points << "/" << _framepoint_generator->numberOfDetectedKeypoints()
                         << " at image: " << current_frame->identifier() << std::endl)

      //ds if we got not enough tracks to evaluate for position tracking
      if (_number_of_tracked_points < _parameters->minimum_number_of_landmarks_to_track) {

        //ds reset state and stick to previous solution
        LOG_WARNING(std::printf("BaseTracker::compute|skipping position tracking due to insufficient number of tracks: %lu\n", _number_of_tracked_points))
        _previous_to_current_camera = TransformMatrix3D::Identity();
        break;
      }

      //ds if we have a previous frame
      if (previous_frame) {

        //ds solve pose on frame points only
        CHRONOMETER_START(pose_optimization);
        _pose_optimizer->setEnableWeightsTranslation(false);
        _pose_optimizer->initialize(previous_frame, current_frame, previous_to_current);
        _pose_optimizer->converge();
        CHRONOMETER_STOP(pose_optimization);

        //ds if the pose computation is acceptable
        if (_pose_optimizer->numberOfInliers() > _parameters->minimum_number_of_landmarks_to_track) {

          //ds compute resulting motion
          _previous_to_current_camera = _pose_optimizer->previousToCurrent();
          const real delta_angular          = WorldMap::toOrientationRodrigues(_previous_to_current_camera.linear()).norm();
          const real delta_translational    = _previous_to_current_camera.translation().norm();

            //ds if the posit result is significant enough
          if (delta_angular > _parameters->minimum_delta_angular_for_movement || delta_translational > _parameters->minimum_delta_translational_for_movement) {

            //ds update tracker - TODO purge this transform chaos
            const TransformMatrix3D world_to_camera_current = _previous_to_current_camera*previous_frame->worldToCameraLeft();
            const TransformMatrix3D world_to_robot_current  = _camera_left->cameraToRobot()*world_to_camera_current;
            current_frame->setRobotToWorld(world_to_robot_current.inverse());
            LOG_WARNING(std::cerr << "BaseTracker::compute|using posit on frame points (experimental) inliers: " << _pose_optimizer->numberOfInliers()
                      << " outliers: " << _pose_optimizer->numberOfOutliers() << " average error: " << _pose_optimizer->totalError()/_pose_optimizer->numberOfInliers() <<  std::endl)
          } else {

            //ds keep previous solution
            current_frame->setRobotToWorld(previous_frame->robotToWorld());
            _previous_to_current_camera = TransformMatrix3D::Identity();
          }

          //ds update context position
          _context->setRobotToWorld(current_frame->robotToWorld());
        }
      }

      //ds check if we can switch the state
      const Count number_of_good_points = current_frame->countPoints(_parameters->minimum_track_length_for_landmark_creation);
      if (number_of_good_points > _parameters->minimum_number_of_landmarks_to_track) {

        //ds trigger landmark creation and framepoint update
        _updatePoints(_context, current_frame);
        _status_previous = _status;
        _status = Frame::Tracking;
      } else {

        //ds just trigger framepoint updates
        current_frame->updateActivePoints();
      }
      break;
    }

    //ds on the track
    case Frame::Tracking: {
      _registerRecursive(previous_frame, current_frame, previous_to_current);
      break;
    }
    default: {
      throw std::runtime_error("invalid tracker state");
      break;
    }
  }

  //ds add new framepoints
  assert(current_frame != 0);
  CHRONOMETER_START(track_creation)
  _addPoints(current_frame);
  CHRONOMETER_STOP(track_creation)
  current_frame->setStatus(_status);
  current_frame->setProjectionTrackingDistancePixels(_projection_tracking_distance_pixels);

  //ds update bookkeeping
  _number_of_tracked_landmarks_previous = _context->currentlyTrackedLandmarks().size();
  _total_number_of_tracked_points      += _number_of_tracked_points;

  //ds update stats
  _mean_number_of_keypoints   = (_mean_number_of_keypoints*(_context->frames().size()-1)+current_frame->_number_of_detected_keypoints)/_context->frames().size();
  _mean_number_of_framepoints = (_mean_number_of_framepoints*(_context->frames().size()-1)+current_frame->points().size())/_context->frames().size();
}

//ds retrieves framepoint correspondences between previous and current frame
void BaseTracker::_track(const Frame* previous_frame_,
                         Frame* current_frame_,
                         const TransformMatrix3D& previous_to_current_,
                         const bool& track_by_appearance_) {
  assert(previous_frame_);
  assert(current_frame_);

  //ds control variables
  current_frame_->points().resize(_number_of_potential_points);
  _number_of_tracked_points    = 0;
  _number_of_tracked_landmarks = 0;
  _number_of_lost_points       = 0;

  //ds retrieve point predictions on current image plane and candidates to track for
  ImageCoordinatesVector projected_image_coordinates_left(0);
  FramePointPointerVector previous_points(0);
  _project(projected_image_coordinates_left,
           previous_points,
           previous_frame_,
           current_frame_,
           previous_to_current_);

  //ds prepare lost buffer
  _lost_points.resize(previous_points.size());

  //ds check state for current framepoint tracking configuration
  if (track_by_appearance_) {

    //ds disable binning
    _enable_keypoint_binning = false;

    //ds maximimize tracking window
    _projection_tracking_distance_pixels = _parameters->maximum_projection_tracking_distance_pixels;
  } else {

    //ds enable binning if set (otherwise remains false)
    _enable_keypoint_binning = _parameters->enable_keypoint_binning;
  }

  //ds set resulting absolute distance (which is the square)
  const int32_t projection_tracking_distance_absolute_pixels = _projection_tracking_distance_pixels*_projection_tracking_distance_pixels;

  //ds adjust pose optimization kernel to current precision
  _pose_optimizer->parameters()->maximum_error_kernel = projection_tracking_distance_absolute_pixels;

  //ds get a fresh track candidate buffer
  TrackCandidateMap track_candidates;

  //ds readability
  const real maximum_distance_appearance = _framepoint_generator->parameters()->matching_distance_tracking_threshold;
  FramePointMatrix current_points        = _framepoint_generator->framepointsInImage();

  //ds loop over all past points
  for (Index index_point_previous = 0; index_point_previous < previous_points.size(); ++index_point_previous) {

    //ds compute current projection points
    FramePoint* previous_point = previous_points[index_point_previous];
    const ImageCoordinates& projection_left(projected_image_coordinates_left[index_point_previous]);

    //ds prior grid location
    const int32_t row_projection = std::round(projection_left.y());
    const int32_t col_projection = std::round(projection_left.x());
    const int32_t row_previous   = std::round(previous_point->imageCoordinatesLeft().y());
    const int32_t col_previous   = std::round(previous_point->imageCoordinatesLeft().x());

    //ds exhaustive search
    int32_t pixel_distance_best = projection_tracking_distance_absolute_pixels;
    real matching_distance_best = maximum_distance_appearance;
    int32_t row_best = -1;
    int32_t col_best = -1;

    //ds ------------------------------------------- STAGE 1: POINT VICINITY TRACKING
    //ds compute borders
    const int32_t row_start_point = std::max(row_projection-_parameters->range_point_tracking, static_cast<int32_t>(0));
    const int32_t row_end_point   = std::min(row_projection+_parameters->range_point_tracking, static_cast<int32_t>(_framepoint_generator->numberOfRowsImage()));
    const int32_t col_start_point = std::max(col_projection-_parameters->range_point_tracking, static_cast<int32_t>(0));
    const int32_t col_end_point   = std::min(col_projection+_parameters->range_point_tracking, static_cast<int32_t>(_framepoint_generator->numberOfColsImage()));

    //ds check state
    if (track_by_appearance_) {

      //ds locate best match in appearance
      for (int32_t row_point = row_start_point; row_point < row_end_point; ++row_point) {
        for (int32_t col_point = col_start_point; col_point < col_end_point; ++col_point) {
          if (current_points[row_point][col_point]) {
            const int32_t pixel_distance = std::fabs(row_projection-row_point)+std::fabs(col_projection-col_point);
            const real matching_distance = cv::norm(previous_point->descriptorLeft(),
                                                    current_points[row_point][col_point]->descriptorLeft(),
                                                    SRRG_PROSLAM_DESCRIPTOR_NORM);

            if (matching_distance < matching_distance_best) {
              pixel_distance_best    = pixel_distance;
              matching_distance_best = matching_distance;
              row_best = row_point;
              col_best = col_point;
            }
          }
        }
      }
    } else {

      //ds locate best match in motion
      for (int32_t row_point = row_start_point; row_point < row_end_point; ++row_point) {
        for (int32_t col_point = col_start_point; col_point < col_end_point; ++col_point) {
          if (current_points[row_point][col_point]) {
            const int32_t pixel_distance = std::fabs(row_projection-row_point)+std::fabs(col_projection-col_point);
            const real matching_distance = cv::norm(previous_point->descriptorLeft(),
                                                    current_points[row_point][col_point]->descriptorLeft(),
                                                    SRRG_PROSLAM_DESCRIPTOR_NORM);

            if (pixel_distance < pixel_distance_best && matching_distance < maximum_distance_appearance) {
              pixel_distance_best    = pixel_distance;
              matching_distance_best = matching_distance;
              row_best = row_point;
              col_best = col_point;
            }
          }
        }
      }
    }

    //ds if we found
    if (row_best != -1) {

      //ds check if track is consistent
      if ((row_best-row_previous)*(row_best-row_previous)+(col_best-col_previous)*(col_best-col_previous) < _parameters->maximum_distance_tracking_pixels) {
        FramePoint* current_point = current_points[row_best][col_best];
        try {
          track_candidates.at(current_point->identifier()).push_back(TrackCandidate(previous_point, row_best, col_best, pixel_distance_best, matching_distance_best));
        } catch (const std::out_of_range& /**/) {
          track_candidates.insert(std::make_pair(current_point->identifier(), TrackCandidateVector(1, TrackCandidate(previous_point, row_best, col_best, pixel_distance_best, matching_distance_best))));
        }
        continue;
      }
    }

    //ds ------------------------------------------- STAGE 2: REGIONAL TRACKING
    pixel_distance_best    = projection_tracking_distance_absolute_pixels;
    matching_distance_best = maximum_distance_appearance;
    row_best               = -1;
    col_best               = -1;

    //ds compute borders
    const int32_t row_start_region = std::max(row_projection-projection_tracking_distance_absolute_pixels, static_cast<int32_t>(0));
    const int32_t row_end_region   = std::min(row_projection+projection_tracking_distance_absolute_pixels, static_cast<int32_t>(_framepoint_generator->numberOfRowsImage()));
    const int32_t col_start_region = std::max(col_projection-projection_tracking_distance_absolute_pixels, static_cast<int32_t>(0));
    const int32_t col_end_region   = std::min(col_projection+projection_tracking_distance_absolute_pixels, static_cast<int32_t>(_framepoint_generator->numberOfColsImage()));

    //ds check state
    if (track_by_appearance_) {

      //ds locate best match in appearance
      for (int32_t row_region = row_start_region; row_region < row_end_region; ++row_region) {
        for (int32_t col_region = col_start_region; col_region < col_end_region; ++col_region) {
          if (current_points[row_region][col_region]) {

            //ds if area has not been already evaluated in previous stage
            if (row_region < row_start_point||
                row_region >= row_end_point ||
                col_region < col_start_point||
                col_region >= col_end_point ) {

              const int32_t pixel_distance = std::fabs(row_projection-row_region)+std::fabs(col_projection-col_region);
              const real matching_distance = cv::norm(previous_point->descriptorLeft(),
                                                      current_points[row_region][col_region]->descriptorLeft(),
                                                      SRRG_PROSLAM_DESCRIPTOR_NORM);

              if (matching_distance < matching_distance_best) {
                pixel_distance_best    = pixel_distance;
                matching_distance_best = matching_distance;
                row_best = row_region;
                col_best = col_region;
              }
            }
          }
        }
      }
    } else {

      //ds locate best match in motion
      for (int32_t row_region = row_start_region; row_region < row_end_region; ++row_region) {
        for (int32_t col_region = col_start_region; col_region < col_end_region; ++col_region) {
          if (current_points[row_region][col_region]) {

            //ds if area has not been already evaluated in previous stage
            if (row_region < row_start_point||
                row_region >= row_end_point ||
                col_region < col_start_point||
                col_region >= col_end_point ) {

              const int32_t pixel_distance = std::fabs(row_projection-row_region)+std::fabs(col_projection-col_region);
              const real matching_distance = cv::norm(previous_point->descriptorLeft(),
                                                      current_points[row_region][col_region]->descriptorLeft(),
                                                      SRRG_PROSLAM_DESCRIPTOR_NORM);

              if (pixel_distance < pixel_distance_best && matching_distance < maximum_distance_appearance) {
                pixel_distance_best    = pixel_distance;
                matching_distance_best = matching_distance;
                row_best = row_region;
                col_best = col_region;
              }
            }
          }
        }
      }
    }

    //ds if we found a match
    if (row_best != -1) {

      //ds check if track is consistent
      if ((row_best-row_previous)*(row_best-row_previous)+(col_best-col_previous)*(col_best-col_previous) < _parameters->maximum_distance_tracking_pixels) {
        FramePoint* current_point = current_points[row_best][col_best];
        try {
          track_candidates.at(current_point->identifier()).push_back(TrackCandidate(previous_point, row_best, col_best, pixel_distance_best, matching_distance_best));
        } catch (const std::out_of_range& /**/) {
          track_candidates.insert(std::make_pair(current_point->identifier(), TrackCandidateVector(1, TrackCandidate(previous_point, row_best, col_best, pixel_distance_best, matching_distance_best))));
        }
        continue;
      }
    }

    //ds no match found - if landmark - and not too many recoveries
    if (previous_point->landmark() && previous_point->landmark()->numberOfRecoveries() < _parameters->maximum_number_of_landmark_recoveries) {
      _lost_points[_number_of_lost_points] = previous_point;
      ++_number_of_lost_points;
    }
  }

  //ds evaluate track candidates and pick best for each current framepoint
  for (TrackCandidateMapElement candidates_selection: track_candidates) {
    TrackCandidateVector& candidates = candidates_selection.second;

    //ds if there are multiple candidates
    if (candidates.size() > 1) {

      //ds find best in appearance
      if (track_by_appearance_) {

        //ds best distance so far - appearance
        TrackCandidate& candidate_best = candidates.front();
        real distance_best             = candidates.front().descriptor_distance;

        //ds loop over the remaining candidates
        for (uint32_t u = 1; u < candidates.size(); ++u) {
          if (candidates[u].descriptor_distance < distance_best) {
            distance_best  = candidates[u].descriptor_distance;
            candidate_best = candidates[u];
          }
        }

        //ds save best
        _addTrack(candidate_best, current_frame_);

      //ds find best in pixel distance
      } else {

        //ds best distance so far - projection
        TrackCandidate& candidate_best = candidates.front();
        real distance_best             = candidates.front().pixel_distance;

        //ds loop over the remaining candidates
        for (uint32_t u = 1; u < candidates.size(); ++u) {
          if (candidates[u].pixel_distance < distance_best) {
            distance_best  = candidates[u].pixel_distance;
            candidate_best = candidates[u];
          }
        }

        //ds save best
        _addTrack(candidate_best, current_frame_);
      }
    } else {

      //ds take the only candidate
      TrackCandidate& candidate = candidates.front();
      _addTrack(candidate, current_frame_);
    }
  }

//  std::cerr << " tracks: " << _number_of_tracked_points << "/" << previous_frame_->points().size()
//            << " : " << static_cast<real>(_number_of_tracked_points)/previous_frame_->points().size()
//            << " of which landmarks: " << _number_of_tracked_landmarks  << "/" << _number_of_tracked_landmarks_previous
//            << " : " << static_cast<real>(_number_of_tracked_landmarks)/_number_of_tracked_landmarks_previous << std::endl;
//  getchar();
  _tracking_ratio = static_cast<real>(_number_of_tracked_points)/previous_frame_->points().size();
  if (_tracking_ratio < 0.1) {
    LOG_WARNING(std::cerr << "BaseTracker::_trackFramepoints|low point tracking ratio: " << _tracking_ratio
              << " (" << _number_of_tracked_points << "/" << previous_frame_->points().size() << ")" << std::endl)
  }

  //ds stats
  _mean_tracking_ratio = (_number_of_tracked_frames*_mean_tracking_ratio+_tracking_ratio)/(1+_number_of_tracked_frames);
  ++_number_of_tracked_frames;

  //ds update frame with current points
  current_frame_->points().resize(_number_of_tracked_points);
  _lost_points.resize(_number_of_lost_points);
  _total_number_of_landmarks += _number_of_tracked_landmarks;
}

void BaseTracker::_registerRecursive(Frame* frame_previous_,
                                     Frame* frame_current_,
                                     TransformMatrix3D previous_to_current_,
                                     const Count& recursion_) {
  assert(_number_of_tracked_landmarks_previous != 0);

  //ds if we're below the target - raise tracking window
  if (_tracking_ratio < 0.1) {

    //ds if we still can increase the tracking window size
    if (_projection_tracking_distance_pixels < _parameters->maximum_projection_tracking_distance_pixels) {
      ++_projection_tracking_distance_pixels;
    }
  } else {

    //ds if we still can reduce the tracking window size
    if (_projection_tracking_distance_pixels > _parameters->minimum_projection_tracking_distance_pixels) {
      --_projection_tracking_distance_pixels;
    }
  }

  //ds current number of tracked landmarks
  real relative_number_of_tracked_landmarks_to_previous = static_cast<real>(_number_of_tracked_landmarks)/_number_of_tracked_landmarks_previous;

  //ds if we got not enough tracks to evaluate for position tracking: robustness
  if (_number_of_tracked_landmarks == 0 || relative_number_of_tracked_landmarks_to_previous < 0.1) {

    //ds if we have recursions left (currently only two)
    if (recursion_ < 2) {

      //ds fallback to no motion model
      previous_to_current_ = TransformMatrix3D::Identity();

      //ds attempt tracking by appearance (maximum window size)
      _track(frame_previous_, frame_current_, previous_to_current_, true);
      _registerRecursive(frame_previous_, frame_current_, previous_to_current_, recursion_+1);
      ++_number_of_recursive_registrations;
      return;
    } else {

      //ds failed
      breakTrack(frame_current_);
      return;
    }
  }

  //ds compute ratio between landmarks and tracked points: informative only
  const real percentage_landmarks = static_cast<real>(_number_of_tracked_landmarks)/_number_of_tracked_points;
  if (percentage_landmarks < 0.1) {
    LOG_WARNING(std::cerr << "BaseTracker::compute|low percentage of tracked landmarks over framepoints: " << percentage_landmarks
              << " (landmarks/framepoints: " << _number_of_tracked_landmarks << "/" << _number_of_tracked_points << ")" << std::endl)
  }

  //ds call pose solver
  CHRONOMETER_START(pose_optimization)
  _pose_optimizer->setEnableWeightsTranslation(true);
  _pose_optimizer->initialize(frame_previous_, frame_current_, previous_to_current_);
  _pose_optimizer->converge();
  CHRONOMETER_STOP(pose_optimization)

  //ds solver deltas
  const Count number_of_inliers = _pose_optimizer->numberOfInliers();
  const real average_error      = _pose_optimizer->totalError()/number_of_inliers;

  //ds if we have enough inliers in the pose optimization
  if (number_of_inliers > _parameters->minimum_number_of_landmarks_to_track) {
    _previous_to_current_camera = _pose_optimizer->previousToCurrent();
    const real delta_angular          = WorldMap::toOrientationRodrigues(_previous_to_current_camera.linear()).norm();
    const real delta_translational    = _previous_to_current_camera.translation().norm();

    //ds if the posit result is significant enough
    if (delta_angular > _parameters->minimum_delta_angular_for_movement || delta_translational > _parameters->minimum_delta_translational_for_movement) {

      //ds update tracker - TODO purge this transform chaos
      const TransformMatrix3D world_to_camera_current = _previous_to_current_camera*frame_previous_->worldToCameraLeft();
      const TransformMatrix3D world_to_robot_current  = _camera_left->cameraToRobot()*world_to_camera_current;
      frame_current_->setRobotToWorld(world_to_robot_current.inverse());
    } else {

      //ds keep previous solution
      frame_current_->setRobotToWorld(frame_previous_->robotToWorld());
      _previous_to_current_camera = TransformMatrix3D::Identity();
    }

    //ds visualization only (we need to clear and push_back in order to not crash the gui since its decoupled - otherwise we could use resize)
    _context->currentlyTrackedLandmarks().reserve(_number_of_tracked_landmarks);

    //ds prune current frame points
    _prunePoints(frame_current_);
    assert(_context->currentlyTrackedLandmarks().size() <= _number_of_tracked_landmarks);
    assert(_number_of_tracked_points >= number_of_inliers);

    //ds recover lost points based on refined pose
    if (_parameters->enable_landmark_recovery) {
      CHRONOMETER_START(point_recovery)
      _recoverPoints(frame_current_);
      CHRONOMETER_STOP(point_recovery)
    }

    //ds update tracks
    _context->setRobotToWorld(frame_current_->robotToWorld());
    CHRONOMETER_START(landmark_optimization)
    _updatePoints(_context, frame_current_);
    CHRONOMETER_STOP(landmark_optimization)

    //ds tracking
    _status_previous = _status;
    _status          = Frame::Tracking;
  } else {
    LOG_WARNING(std::cerr << frame_current_->identifier() << "|BaseTracker::_registerRecursive|inliers: " << number_of_inliers
                          << " average error: " << average_error << std::endl)

    //ds if we have recursions left (currently only two)
    if (recursion_ < 2) {

      //ds if we still can increase the tracking window size
      if (_projection_tracking_distance_pixels < _parameters->maximum_projection_tracking_distance_pixels) {
        ++_projection_tracking_distance_pixels;
      }

      //ds attempt new tracking with the increased window size
      _track(frame_previous_, frame_current_, previous_to_current_);
      _registerRecursive(frame_previous_, frame_current_, previous_to_current_, recursion_+1);
      ++_number_of_recursive_registrations;
    } else {

      //ds failed
      breakTrack(frame_current_);
    }
  }
}

void BaseTracker::_addTrack(TrackCandidate& track_candidate_, Frame* current_frame_) {

  //ds allocate a new point connected to the previous one
  FramePoint* current_point = _framepoint_generator->framepointsInImage()[track_candidate_.row][track_candidate_.col];
  current_point->setPrevious(track_candidate_.framepoint);

  //ds set the point to the control structure
  current_frame_->points()[_number_of_tracked_points] = current_point;
  if (current_point->landmark()) {
    ++_number_of_tracked_landmarks;
  }
  ++_number_of_tracked_points;
}

//! @breaks the track at the current frame
void BaseTracker::breakTrack(Frame* frame_) {

  //ds reset state
  LOG_WARNING(std::printf("BaseTracker::breakTrack|LOST TRACK at frame [%06lu] with tracks: %lu\n", frame_->identifier(), _number_of_tracked_points))
  _status_previous = Frame::Localizing;
  _status          = Frame::Localizing;
  _enable_keypoint_binning = false;

  //ds stick to previous solution - simulating a fresh start
  frame_->setRobotToWorld(frame_->previous()->robotToWorld());
  _previous_to_current_camera = TransformMatrix3D::Identity();
  _number_of_recovered_points = 0;
  _number_of_tracked_points   = 0;

  //ds reset frame in world context, triggering a restart of the pipeline
  _context->breakTrack(frame_);

  //ds release all framepoints currently in the bin - enabling a full fresh addition in the add framepoints phase
  for (Index row_bin = 0; row_bin < _number_of_rows_bin; ++row_bin) {
    for (Index col_bin = 0; col_bin < _number_of_cols_bin; ++col_bin) {
      _bin_map_left[row_bin][col_bin] = 0;
    }
  }
}

//ds adds new framepoints to the provided frame (picked from the pool of the _framepoint_generator)
void BaseTracker::_addPoints(Frame* frame_) {
  const TransformMatrix3D& robot_to_world = frame_->robotToWorld();
  FramePointMatrix current_points         = _framepoint_generator->framepointsInImage();

  //ds clear tracked points in the framepoint matrix ("masking")
  for (FramePoint* frame_point: frame_->points()) {

    //ds determine bin index of the current point
    const Count row_bin = std::rint(static_cast<real>(frame_point->row)/_parameters->bin_size_pixels);
    const Count col_bin = std::rint(static_cast<real>(frame_point->col)/_parameters->bin_size_pixels);

    //ds occupy corresponding bin for keypoint binning
    _bin_map_left[row_bin][col_bin] = frame_point;

    //ds free point from framepoint matrix
    current_points[frame_point->row][frame_point->col] = 0;
  }

  //ds make space for all remaining points
  frame_->points().resize(_number_of_potential_points+_number_of_recovered_points);
  Index index_point_new = _number_of_tracked_points;

  //ds if binning is enabled
  if (_enable_keypoint_binning) {

    //ds check triangulation map for unmatched points and fill them into the bin map
    for (Index row = 0; row < _framepoint_generator->numberOfRowsImage(); ++row) {
      for (Index col = 0; col < _framepoint_generator->numberOfColsImage(); ++col) {
        if (current_points[row][col]) {
          FramePoint* frame_point = current_points[row][col];

          //ds determine bin index of the current point
          const Index row_bin = std::rint(static_cast<real>(row)/_parameters->bin_size_pixels);
          const Index col_bin = std::rint(static_cast<real>(col)/_parameters->bin_size_pixels);
          assert(row_bin < _number_of_rows_bin);
          assert(col_bin < _number_of_cols_bin);

          //ds if the bin is empty
          if (!_bin_map_left[row_bin][col_bin]) {

            //ds set the current point
            _bin_map_left[row_bin][col_bin] = frame_point;
          }

          //ds or if the bin is not occupied by a tracked point and we have a point with higher keypoint response
          else if (!_bin_map_left[row_bin][col_bin]->previous() && frame_point->keypointLeft().response > _bin_map_left[row_bin][col_bin]->keypointLeft().response) {

            //ds set the current point
            _bin_map_left[row_bin][col_bin] = frame_point;
          }

          //ds always free point from input grid
          current_points[row][col] = 0;
        }
      }
    }

    //ds collect new framepoints from bin map and clear it simultaneously - TODO this could be done much faster using bookkeeping
    for (Index row_bin = 0; row_bin < _number_of_rows_bin; ++row_bin) {
      for (Index col_bin = 0; col_bin < _number_of_cols_bin; ++col_bin) {
        if (_bin_map_left[row_bin][col_bin]) {

          //ds if not occupied by a tracked point
          if (!_bin_map_left[row_bin][col_bin]->previous()) {

            //ds assign the new point
            frame_->points()[index_point_new] = _bin_map_left[row_bin][col_bin];

            //ds update framepoint world position using the current pose estimate
            frame_->points()[index_point_new]->setWorldCoordinates(robot_to_world*frame_->points()[index_point_new]->robotCoordinates());
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
        if (current_points[row][col]) {

          //ds assign the new point
          frame_->points()[index_point_new] = current_points[row][col];

          //ds update framepoint world position using the current pose estimate
          frame_->points()[index_point_new]->setWorldCoordinates(robot_to_world*frame_->points()[index_point_new]->robotCoordinates());
          ++index_point_new;

          //ds free point from input grid
          current_points[row][col] = 0;
        }
      }
    }
  }
  frame_->points().resize(index_point_new);
}

//ds retrieves framepoint projections as image coordinates in a vector (at the same time removing points with invalid projections)
void BaseTracker::_project(ImageCoordinatesVector& projected_image_coordinates_left_,
                           FramePointPointerVector& previous_points_,
                           const Frame* previous_frame_,
                           const Frame* current_frame_,
                           const TransformMatrix3D& previous_to_current_) const {
  const Matrix3& camera_calibration_matrix = current_frame_->cameraLeft()->cameraMatrix();
  ASSERT(previous_frame_ != 0, "previous frame not set")
  ASSERT(current_frame_ != 0,"previous frame not set")

  //ds preallocation of output structures
  projected_image_coordinates_left_.resize(previous_frame_->points().size());
  previous_points_.resize(previous_frame_->points().size());

  //ds compute projections for all previous frame points into the image plane of the current image
  Count number_of_valid_projected_points = 0;
  for (FramePoint* previous_point: previous_frame_->points()) {
    assert(previous_point->imageCoordinatesLeft().x() >= 0);
    assert(previous_point->imageCoordinatesLeft().x() <= _camera_left->numberOfImageCols());
    assert(previous_point->imageCoordinatesLeft().y() >= 0);
    assert(previous_point->imageCoordinatesLeft().y() <= _camera_left->numberOfImageRows());

    //ds get point into current camera - based on last track
    ImageCoordinates point_in_image_left;

    //ds if we have a valid landmark at hand - prefer the position estimate over the raw measurement
    if (previous_point->landmark()) {
      point_in_image_left = camera_calibration_matrix*previous_to_current_*previous_point->cameraCoordinatesLeftLandmark();
    } else {
      point_in_image_left = camera_calibration_matrix*previous_to_current_*previous_point->cameraCoordinatesLeft();
    }

    //ds normalize point and update prediction based on landmark position: LEFT
    point_in_image_left /= point_in_image_left.z();

    //ds check for invalid projections (out of FOV) and skip if so
    if (point_in_image_left.x() < 0 || point_in_image_left.x() > _camera_left->numberOfImageCols()||
        point_in_image_left.y() < 0 || point_in_image_left.y() > _camera_left->numberOfImageRows()) {
      continue;
    }

    //ds update result containers
    projected_image_coordinates_left_[number_of_valid_projected_points] = point_in_image_left;
    previous_points_[number_of_valid_projected_points]                  = previous_point;
    ++number_of_valid_projected_points;
  }

  //ds trim containers
  previous_points_.resize(number_of_valid_projected_points);
  projected_image_coordinates_left_.resize(number_of_valid_projected_points);
}

//ds prunes invalid points after pose optimization
void BaseTracker::_prunePoints(Frame* frame_) {

  //ds filter invalid points of pose optimization
  _number_of_tracked_points = 0;
  for (Index index_point = 0; index_point < frame_->points().size(); index_point++) {
    assert(frame_->points()[index_point]->previous());

    //ds keep points which were not suppressed in the optimization
    if (_pose_optimizer->errors()[index_point] != -1) {

      //ds if we have a new point keep it
      //if (!frame_->points()[index_point]->landmark()) {
        frame_->points()[_number_of_tracked_points] = frame_->points()[index_point];
        ++_number_of_tracked_points;

      //ds keep landmarks only if they were inliers
      //} else if (_pose_optimizer->inliers()[index_point]) {
      //  frame_->points()[_number_of_tracked_points] = frame_->points()[index_point];
      //  ++_number_of_tracked_points;
      //}
    }
  }
//  std::cerr << "dropped points REGISTRATION: " << frame_->points().size()-_number_of_tracked_points << "/" << frame_->points().size() << std::endl;
  frame_->points().resize(_number_of_tracked_points);
}

//ds updates existing or creates new landmarks for framepoints of the provided frame
void BaseTracker::_updatePoints(WorldMap* context_, Frame* frame_) {

  //ds buffer current pose
  const TransformMatrix3D& robot_to_world = frame_->robotToWorld();

  //ds start landmark generation/update
  for (FramePoint* point: frame_->points()) {
    point->setWorldCoordinates(robot_to_world*point->robotCoordinates());

    //ds skip point if tracking and not mature enough to be a landmark - for localizing state this is skipped
    if (point->trackLength() < _parameters->minimum_track_length_for_landmark_creation) {
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

    //ds sanity check TODO check with g2o and purge
    try {
      if (landmark != context_->landmarks().at(landmark->identifier())) {
        LOG_DEBUG(std::cerr << "BaseTracker::_updatePoints|skipping invalid landmark with ID: " << landmark->identifier()
                            << " (suspected memory corruption)" << std::endl)
        point->setLandmark(nullptr);
        continue;
      }
    } catch (const std::out_of_range& /*ex*/) {
      LOG_DEBUG(std::cerr << "BaseTracker::_updatePoints|skipping invalid landmark with ID: " << landmark->identifier()
                          << " (suspected memory corruption)" << std::endl)
      point->setLandmark(nullptr);
      continue;
    }

    //ds update landmark position based on current point (triggered as we linked the landmark to the point)
    landmark->update(point);
    point->setCameraCoordinatesLeftLandmark(frame_->worldToCameraLeft()*landmark->coordinates());

    //ds VISUALIZATION ONLY: add landmarks to currently visible ones
    landmark->setIsCurrentlyTracked(true);
    context_->currentlyTrackedLandmarks().push_back(landmark);
  }
}
}
