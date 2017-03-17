#include "tracker.h"

namespace proslam {
  using namespace srrg_core;

  Tracker::Tracker(const Camera* camera_left_,
                         const Camera* camera_right_): _camera_left(camera_left_),
                                                       _camera_right(camera_right_),
                                                       _camera_rows(camera_left_->imageRows()),
                                                       _camera_cols(camera_left_->imageCols()),
                                                       _preprocessor(new StereoTriangulator(_camera_left, _camera_right)),
                                                       _aligner(static_cast<StereoUVAligner*>(AlignerFactory::create(AlignerType6_4::stereouv))) {
    std::cerr << "Tracker::Tracker|constructing" << std::endl;

    //ds request initial tracking context for the tracker
    _lost_points.clear();

    //ds allocate stereouv aligner for odometry computation
    assert(_aligner != 0);

    //ds configure aligner
    _aligner->setMaximumDepthClose(StereoTriangulator::maximum_depth_close);
    _aligner->setMaximumDepthFar(StereoTriangulator::maximum_depth_far);

    //ds clear buffers
    _projected_image_coordinates_left.clear();
    std::cerr << "Tracker::Tracker|constructed" << std::endl;
  }

  Tracker::~Tracker() {
    std::cerr << "Tracker::Tracker|destroying" << std::endl;
    //ds TODO clear tracker owned scopes
    delete _preprocessor;
    std::cerr << "Tracker::Tracker|destroyed" << std::endl;
  }

  void Tracker::trackFeatures(Frame* previous_frame_, Frame* current_frame_) {
    assert(previous_frame_ != 0);
    assert(current_frame_ != 0);

    //ds control variables
    current_frame_->points().resize(_number_of_potential_points);
    _number_of_tracked_points          = 0;
    _number_of_tracked_landmarks_close = 0;
    _number_of_tracked_landmarks_far   = 0;
    _number_of_lost_points             = 0;

    //ds retrieve point predictions on current image plane
    getImageCoordinates(_projected_image_coordinates_left, previous_frame_, current_frame_);

    //ds prepare lost buffer
    _lost_points.resize(previous_frame_->points().size());

    //ds check state
    if (_status_previous == Frame::Localizing) {

      //ds for localization mode we have a more relaxed tracking condition
      _pixel_distance_tracking = _pixel_distance_tracking_maximum;
    } else {

      //ds narrow search limit closer to projection when we're in tracking mode
      _pixel_distance_tracking = _pixel_distance_tracking_minimum;
    }
    const real _maximum_matching_distance_tracking_point  = _preprocessor->maximumTrackingMatchingDistance();
    const real _maximum_matching_distance_tracking_region = _preprocessor->maximumTrackingMatchingDistance();

    //ds loop over all past points
    for (Index index_point_previous = 0; index_point_previous < previous_frame_->points().size(); ++index_point_previous) {

      //ds compute current projection points
      FramePoint* previous_point = previous_frame_->points()[index_point_previous];
      const ImageCoordinates& projection_left(_projected_image_coordinates_left[index_point_previous]);

      //ds prior grid location
      const int32_t row_projection = std::round(projection_left.y());
      const int32_t col_projection = std::round(projection_left.x());
      const int32_t row_previous = std::round(previous_point->imageCoordinates().y());
      const int32_t col_previous = std::round(previous_point->imageCoordinates().x());

      //ds exhaustive search
      int32_t pixel_distance_best = _pixel_distance_tracking;
      int32_t row_best = -1;
      int32_t col_best = -1;

//ds ------------------------------------------- STAGE 1: POINT VICINITY TRACKING
      //ds compute borders
      const int32_t row_start_point = std::max(row_projection-_range_point_tracking, static_cast<int32_t>(0));
      const int32_t row_end_point   = std::min(row_projection+_range_point_tracking, static_cast<int32_t>(_preprocessor->numberOfRowsImage()));
      const int32_t col_start_point = std::max(col_projection-_range_point_tracking, static_cast<int32_t>(0));
      const int32_t col_end_point   = std::min(col_projection+_range_point_tracking, static_cast<int32_t>(_preprocessor->numberOfColsImage()));

      //ds locate best match
      for (int32_t row_point = row_start_point; row_point < row_end_point; ++row_point) {
        for (int32_t col_point = col_start_point; col_point < col_end_point; ++col_point) {
          if (_preprocessor->framepointMap()[row_point][col_point].is_available) {
            const int32_t pixel_distance = std::fabs(row_projection-row_point)+std::fabs(col_projection-col_point);
            const real matching_distance = cv::norm(previous_point->descriptor(),
                                                       _preprocessor->framepointMap()[row_point][col_point].descriptor_left,
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
      if (pixel_distance_best < _pixel_distance_tracking) {

        //ds check if track is consistent
        if ((row_best-row_previous)*(row_best-row_previous)+(col_best-col_previous)*(col_best-col_previous) < _maximum_flow_pixels_squared) {

          //ds allocate a new point connected to the previous one
          FramePoint* current_point = current_frame_->createNewPoint(_preprocessor->framepointMap()[row_best][col_best].keypoint_left,
                                                                    _preprocessor->framepointMap()[row_best][col_best].descriptor_left,
                                                                    _preprocessor->framepointMap()[row_best][col_best].keypoint_right,
                                                                    _preprocessor->framepointMap()[row_best][col_best].descriptor_right,
                                                                    _preprocessor->framepointMap()[row_best][col_best].camera_coordinates_left.z(),
                                                                    current_frame_->camera()->cameraToRobot()*_preprocessor->framepointMap()[row_best][col_best].camera_coordinates_left,
                                                                    previous_point);
          //ds set the point to the control structure
          current_frame_->points()[_number_of_tracked_points] = current_point;
          if (current_point->landmark()) {
            if (current_point->hasDepth()) {
              ++_number_of_tracked_landmarks_close;
            } else {
              ++_number_of_tracked_landmarks_far;
            }
          }

          const cv::Point2f point_previous(previous_point->imageCoordinates().x(), previous_point->imageCoordinates().y());
          const cv::Point2f point_current(current_point->imageCoordinates().x(), current_point->imageCoordinates().y());
          ++_number_of_tracked_points;

          //ds disable further matching and reduce search time
          _preprocessor->framepointMap()[row_best][col_best].is_available = false;
          continue;
        }
      }

//ds ------------------------------------------- STAGE 2: REGIONAL TRACKING
      pixel_distance_best = _pixel_distance_tracking;

      //ds compute borders
      const int32_t row_start_region = std::max(row_projection-_pixel_distance_tracking, static_cast<int32_t>(0));
      const int32_t row_end_region   = std::min(row_projection+_pixel_distance_tracking, static_cast<int32_t>(_preprocessor->numberOfRowsImage()));
      const int32_t col_start_region = std::max(col_projection-_pixel_distance_tracking, static_cast<int32_t>(0));
      const int32_t col_end_region   = std::min(col_projection+_pixel_distance_tracking, static_cast<int32_t>(_preprocessor->numberOfColsImage()));

      //ds locate best match
      for (int32_t row_region = row_start_region; row_region < row_end_region; ++row_region) {
        for (int32_t col_region = col_start_region; col_region < col_end_region; ++col_region) {
          if (_preprocessor->framepointMap()[row_region][col_region].is_available) {

            //ds if area has not been already evaluated in previous stage
            if (row_region < row_start_point||
                row_region >= row_end_point ||
                col_region < col_start_point||
                col_region >= col_end_point ) {

              const int32_t pixel_distance = std::fabs(row_projection-row_region)+std::fabs(col_projection-col_region);
              const real matching_distance = cv::norm(previous_point->descriptor(),
                                                         _preprocessor->framepointMap()[row_region][col_region].descriptor_left,
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
      if (pixel_distance_best < _pixel_distance_tracking) {

        //ds check if track is consistent
        if ((row_best-row_previous)*(row_best-row_previous)+(col_best-col_previous)*(col_best-col_previous) < _maximum_flow_pixels_squared) {

          //ds allocate a new point connected to the previous one
          FramePoint* current_point = current_frame_->createNewPoint(_preprocessor->framepointMap()[row_best][col_best].keypoint_left,
                                                                     _preprocessor->framepointMap()[row_best][col_best].descriptor_left,
                                                                     _preprocessor->framepointMap()[row_best][col_best].keypoint_right,
                                                                     _preprocessor->framepointMap()[row_best][col_best].descriptor_right,
                                                                     _preprocessor->framepointMap()[row_best][col_best].camera_coordinates_left.z(),
                                                                     current_frame_->camera()->cameraToRobot()*_preprocessor->framepointMap()[row_best][col_best].camera_coordinates_left,
                                                                     previous_point);
          //ds set the point to the control structure
          current_frame_->points()[_number_of_tracked_points] = current_point;
          if (current_point->landmark()) {
            if (current_point->hasDepth()) {
              ++_number_of_tracked_landmarks_close;
            } else {
              ++_number_of_tracked_landmarks_far;
            }
          }

          const cv::Point2f point_previous(previous_point->imageCoordinates().x(), previous_point->imageCoordinates().y());
          const cv::Point2f point_current(current_point->imageCoordinates().x(), current_point->imageCoordinates().y());
          ++_number_of_tracked_points;

          //ds disable further matching and reduce search time
          _preprocessor->framepointMap()[row_best][col_best].is_available = false;
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

  void Tracker::extractFeatures(Frame* frame_) {

    //ds make space for all remaining points
    frame_->points().resize(_number_of_potential_points+_number_of_lost_points_recovered);

    //ds check triangulation map for unmatched points TODO this can be done faster
    Index index_point_new = _number_of_tracked_points;
    for (uint32_t row = 0; row < _preprocessor->numberOfRowsImage(); ++row) {
      for (uint32_t col = 0; col < _preprocessor->numberOfColsImage(); ++col) {
        if (_preprocessor->framepointMap()[row][col].is_available) {

          //ds create a new point
          frame_->points()[index_point_new] = frame_->createNewPoint(_preprocessor->framepointMap()[row][col].keypoint_left,
                                                                     _preprocessor->framepointMap()[row][col].descriptor_left,
                                                                     _preprocessor->framepointMap()[row][col].keypoint_right,
                                                                     _preprocessor->framepointMap()[row][col].descriptor_right,
                                                                     _preprocessor->framepointMap()[row][col].camera_coordinates_left.z(),
                                                                     frame_->camera()->cameraToRobot()*_preprocessor->framepointMap()[row][col].camera_coordinates_left);
          ++index_point_new;
          _preprocessor->framepointMap()[row][col].is_available = false;
        }
      }
    }
    frame_->points().resize(index_point_new);
//    std::cerr << "Tracker::extractFeatures|new points: " << index_point_new-_number_of_tracked_points << std::endl;
  }

  void Tracker::getImageCoordinates(std::vector<ImageCoordinates>& projected_image_coordinates_left_,
                                       Frame* previous_frame_,
                                       const Frame* current_frame_) const {
    assert(previous_frame_ != 0);
    assert(current_frame_ != 0);

    //ds preallocation
    projected_image_coordinates_left_.resize(previous_frame_->points().size());
    const TransformMatrix3D robot_previous_to_current = current_frame_->worldToRobot()*previous_frame_->robotToWorld();
    const TransformMatrix3D robot_to_camera           = current_frame_->camera()->robotToCamera();
    const TransformMatrix3D world_to_camera           = robot_to_camera*current_frame_->worldToRobot();

    //ds compute predictions for all previous frame points
    Count number_of_visible_points = 0;
    for (FramePoint* previous_frame_point: previous_frame_->points()) {
      assert(previous_frame_point->imageCoordinates().x() >= 0);
      assert(previous_frame_point->imageCoordinates().x() <= current_frame_->camera()->imageCols());
      assert(previous_frame_point->imageCoordinates().y() >= 0);
      assert(previous_frame_point->imageCoordinates().y() <= current_frame_->camera()->imageRows());

      //ds get point into current camera - based on last track
      Vector4 point_in_camera_homogeneous(Vector4::Ones());

      //ds if we have a valid landmark at hand
      if (previous_frame_point->landmark() && previous_frame_point->landmark()->isValidated()) {

        //ds get point in camera frame based on landmark coordinates
        point_in_camera_homogeneous.head<3>() = world_to_camera*previous_frame_point->landmark()->coordinates();
      } else {

        //ds reproject based on last track
        point_in_camera_homogeneous.head<3>() = robot_to_camera*(robot_previous_to_current*previous_frame_point->robotCoordinates());
      }

      //ds obtain point projection on camera image plane
      PointCoordinates point_in_image_left  = current_frame_->camera()->projectionMatrix()*point_in_camera_homogeneous;

      //ds normalize point and update prediction based on landmark position: LEFT
      point_in_image_left  /= point_in_image_left.z();

      //ds check for invalid projections
      if (point_in_image_left.x() < 0 || point_in_image_left.x() > current_frame_->camera()->imageCols()       ||
          point_in_image_left.y() < 0 || point_in_image_left.y() > current_frame_->camera()->imageRows()       ) {

        //ds out of FOV
        continue;
      }

      //ds update predictions
      projected_image_coordinates_left_[number_of_visible_points]  = point_in_image_left;
      previous_frame_->points()[number_of_visible_points] = previous_frame_point;
      ++number_of_visible_points;
    }
    previous_frame_->points().resize(number_of_visible_points);
    projected_image_coordinates_left_.resize(number_of_visible_points);
  }

  void Tracker::updateLandmarks(WorldMap* context_) {
    Frame* current_frame = context_->currentFrame();

//    //ds precomputations
    const TransformMatrix3D world_to_camera_left(current_frame->camera()->robotToCamera()*current_frame->worldToRobot());

    //ds start landmark generation/update
    for (FramePoint* point: current_frame->points()) {

      //ds skip point if tracking and not mature enough to be a landmark - for localizing state this is skipped
      if (point->age() < Frame::minimum_landmark_age) {
        continue;
      }

      //ds initial update setup
      Landmark* landmark = point->landmark();

      //ds initial guess
      real depth_meters = point->depth();

      //ds default information value
      Matrix3 omega(Matrix3::Identity());

      //ds check depth situation: regular depth
      if (point->hasDepth()) {

        //ds if no landmark yet - but point with depth information, create a new landmark using the given depth
        if (!landmark) {
          landmark = context_->createLandmark(current_frame->robotToWorld()*point->robotCoordinates());
          point->setLandmark(landmark);
        }

        //ds depth must be within limits here
        assert(depth_meters > 0);
        assert(depth_meters < StereoTriangulator::maximum_depth_far);

        //ds adjust omega to include full depth information
        landmark->setIsByVision(false);
        omega *= (StereoTriangulator::maximum_depth_far-depth_meters)/StereoTriangulator::maximum_depth_far;
      }

      //ds depth obtained by vision
      else {

        //ds create a new landmark using the vision depth
        if (!landmark) {
          landmark = context_->createLandmark(current_frame->robotToWorld()*point->robotCoordinates());
          point->setLandmark(landmark);
        }

        //ds adjust omega to include full depth information
        landmark->setIsByVision(true);
        omega *= (StereoTriangulator::maximum_depth_close-depth_meters)/StereoTriangulator::maximum_depth_close;
      }

      assert(depth_meters > 0);
      assert(landmark != 0);

      //ds update landmark position
      landmark->update(current_frame->robotToWorld()*point->robotCoordinates(),
                       point->descriptor(),
                       point->descriptorExtra(),
                       depth_meters);
    }
  }

  const TransformMatrix3D Tracker::addImage(WorldMap* context_,
                                               const cv::Mat& intensity_image_left_,
                                               const cv::Mat& intensity_image_right_,
                                               const TransformMatrix3D& initial_guess_world_previous_to_current_) {

    //ds reset point configurations
    _number_of_potential_points      = 0;
    _number_of_tracked_points        = 0;
    _number_of_lost_points           = 0;
    _number_of_lost_points_recovered = 0;

    //ds retrieve odometry prior - might be identity
    TransformMatrix3D robot_to_world_current = context_->robotToWorldPrevious();
    if (context_->currentFrame()) {
      robot_to_world_current = initial_guess_world_previous_to_current_*robot_to_world_current;
    }
    context_->setRobotToWorldPrevious(robot_to_world_current);

    //ds create new frame (memory lock expected)
    Frame* current_frame = context_->createFrame(robot_to_world_current, StereoTriangulator::maximum_depth_close);
    current_frame->setCamera(_camera_left);
    current_frame->setIntensityImage(intensity_image_left_);
    current_frame->setCameraExtra(_camera_right);
    current_frame->setIntensityImageExtra(intensity_image_right_);

    //ds compute full sensory prior for the current frame
    _number_of_potential_points = _preprocessor->triangulate(current_frame);

    //ds if available - attempt to track the points from the previous frame
    if (current_frame->previous()) {
      CHRONOMETER_START(tracking)
      trackFeatures(current_frame->previous(), current_frame);
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
          context_->landmarks().clearActive();
          _aligner->init(current_frame, current_frame->robotToWorld());
          _aligner->setWeightFramepoint(1);
          _aligner->converge();
          CHRONOMETER_STOP(pose_optimization)

          //ds if the pose computation is acceptable
          if (_aligner->numberOfInliers() > 2*_minimum_number_of_landmarks_to_track) {

            //ds solver deltas
            world_previous_to_current      = _aligner->robotToWorld()*current_frame->previous()->robotToWorld().inverse();
            const real delta_angular       = WorldMap::toOrientationRodrigues(world_previous_to_current.linear()).norm();
            const real delta_translational = world_previous_to_current.translation().norm();

            //ds if the posit result is significant enough
            if (delta_angular > 0.001 || delta_translational > 0.01) {

              //ds update tracker
              current_frame->setRobotToWorld(_aligner->robotToWorld());
              std::cerr << "Tracker::addImage|WARNING: using posit on frame points (experimental) inliers: " << _aligner->numberOfInliers()
                        << " outliers: " << _aligner->numberOfOutliers() << " average error: " << _aligner->totalError()/_aligner->numberOfInliers() <<  std::endl;
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
        const Count number_of_good_points = current_frame->countPoints(Frame::minimum_landmark_age);
        if (number_of_good_points > _minimum_number_of_landmarks_to_track) {
          updateLandmarks(context_);
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
        context_->landmarks().clearActive();
        _aligner->init(current_frame, current_frame->robotToWorld());
        _aligner->setWeightFramepoint(std::max(weight_framepoint, static_cast<real>(0.1)));
        _aligner->converge();
        CHRONOMETER_STOP(pose_optimization)

        //ds solver deltas
        Count number_of_inliers    = _aligner->numberOfInliers();
        world_previous_to_current  = _aligner->robotToWorld()*current_frame->previous()->robotToWorld().inverse();
        real delta_angular         = WorldMap::toOrientationRodrigues(world_previous_to_current.linear()).norm();
        real delta_translational   = world_previous_to_current.translation().norm();

        //ds if we don't have enough inliers - trigger fallback posit
        if (number_of_inliers < _minimum_number_of_landmarks_to_track) {

          //ds trigger identity movement posit
          CHRONOMETER_START(pose_optimization)
          context_->landmarks().clearActive();
          _aligner->init(current_frame, current_frame->previous()->robotToWorld());
          _aligner->setWeightFramepoint(std::max(weight_framepoint, static_cast<real>(0.1)));
          _aligner->converge();
          CHRONOMETER_STOP(pose_optimization)

          //ds update stats
          number_of_inliers         = _aligner->numberOfInliers();
          world_previous_to_current = _aligner->robotToWorld()*current_frame->previous()->robotToWorld().inverse();
          delta_angular             = WorldMap::toOrientationRodrigues(world_previous_to_current.linear()).norm();
          delta_translational       = world_previous_to_current.translation().norm();

          //ds check if still insufficient
          if (number_of_inliers < _minimum_number_of_landmarks_to_track) {

            //ds reset state
            std::cerr << "LOST TRACK due to invalid position optimization" << std::endl;
            _status_previous = Frame::Localizing;
            _status          = Frame::Localizing;
            current_frame->setStatus(_status);
            current_frame->points().clear();

            //ds keep previous solution
            current_frame->setRobotToWorld(current_frame->previous()->robotToWorld());
            world_previous_to_current = TransformMatrix3D::Identity();
            context_->setRobotToWorldPrevious(current_frame->robotToWorld());
            return world_previous_to_current;
          }

          std::cerr << "Tracker::addImage|WARNING: using posit on identiy motion model (experimental) inliers: " << _aligner->numberOfInliers()
                    << " outliers: " << _aligner->numberOfOutliers() << " average error: " << _aligner->totalError()/_aligner->numberOfInliers() <<  std::endl;
        }

        //ds if the posit result is significant enough
        if (delta_angular > 0.001 || delta_translational > 0.01) {

          //ds update robot pose with posit result
          current_frame->setRobotToWorld(_aligner->robotToWorld());
        } else {

          //ds keep previous solution
          current_frame->setRobotToWorld(current_frame->previous()->robotToWorld());
          world_previous_to_current = TransformMatrix3D::Identity();
        }

        //ds update current frame points
        _number_of_tracked_points = 0;
        for (Index index_point = 0; index_point < current_frame->points().size(); index_point++) {

          //ds points without landmarks are always kept
          if (!current_frame->points()[index_point]->landmark()) {
            current_frame->points()[_number_of_tracked_points] = current_frame->points()[index_point];
            ++_number_of_tracked_points;
          }

          //ds keep the point if it has been skipped (due to insufficient maturity or invalidity) or is an inlier
          else if (_aligner->errors()[index_point] == -1 || _aligner->inliers()[index_point]) {
            current_frame->points()[_number_of_tracked_points] = current_frame->points()[index_point];
            if (current_frame->points()[_number_of_tracked_points]->landmark() != 0) {
              current_frame->points()[_number_of_tracked_points]->landmark()->setIsActive(true);
            }
            ++_number_of_tracked_points;
          }
        }

        //ds update point container
        current_frame->points().resize(_number_of_tracked_points);
        assert(_number_of_tracked_points >= number_of_inliers);

        //ds recover lost points based on updated pose
        CHRONOMETER_START(point_recovery)
        recoverPoints(context_);
        CHRONOMETER_STOP(point_recovery)

        //ds update tracks
        context_->setRobotToWorldPrevious(current_frame->robotToWorld());
        CHRONOMETER_START(landmark_optimization)
        updateLandmarks(context_);
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
    extractFeatures(current_frame);
    CHRONOMETER_STOP(track_creation)
    current_frame->setStatus(_status);

    //ds done
    _total_number_of_tracked_points += _number_of_tracked_points;
    return world_previous_to_current;
  }

  void Tracker::recoverPoints(WorldMap* context_) {
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
      const float regional_border_center = 4*point_previous->keypoint().size;
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
      keypoint_buffer_left[0] = point_previous->keypoint();
      keypoint_buffer_left[0].pt = offset_keypoint_half;
      cv::Mat descriptor_left;
      _preprocessor->descriptorExtractor()->compute(current_frame->intensityImage()(region_of_interest_left), keypoint_buffer_left, descriptor_left);
      if (descriptor_left.rows == 0) {
        continue;
      }
      keypoint_buffer_left[0].pt += corner_left;

      //ds extract descriptors at this position: RIGHT
      keypoint_buffer_right[0] = point_previous->keypointExtra();
      keypoint_buffer_right[0].pt = offset_keypoint_half;
      cv::Mat descriptor_right;
      _preprocessor->descriptorExtractor()->compute(current_frame->intensityImageExtra()(region_of_interest_right), keypoint_buffer_right, descriptor_right);
      if (descriptor_right.rows == 0) {
        continue;
      }
      keypoint_buffer_right[0].pt += corner_right;

      if (cv::norm(point_previous->descriptor(), descriptor_left, DESCRIPTOR_NORM) < _preprocessor->maximumTrackingMatchingDistance()      &&
          cv::norm(point_previous->descriptorExtra(), descriptor_right, DESCRIPTOR_NORM) < _preprocessor->maximumTrackingMatchingDistance()) {
        try{

          //ds triangulate point
          const PointCoordinates camera_coordinates(_preprocessor->getCoordinatesInCamera(keypoint_buffer_left[0].pt, keypoint_buffer_right[0].pt));

          //ds allocate a new point connected to the previous one
          FramePoint* current_point = current_frame->createNewPoint(keypoint_buffer_left[0],
                                                                    descriptor_left,
                                                                    keypoint_buffer_right[0],
                                                                    descriptor_right,
                                                                    camera_coordinates.z(),
                                                                    current_frame->camera()->cameraToRobot()*camera_coordinates,
                                                                    point_previous);
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
