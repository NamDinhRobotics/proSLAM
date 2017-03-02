#include "gt_tracker.h"

namespace gslam {

  Tracker::Tracker(WorldContext* context_): _optical_flow_win_size(21, 21),
                                            _optical_flow_termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03),
                                            _aligner(static_cast<UVDAligner*>(AlignerFactory::create(AlignerType6_3::uvd))){
    _context                    = context_;
    _status                     = Frame::Localizing;
    _min_feature_distance       = 5; //5
    _min_num_landmarks_to_track = 5;
    _min_reprojection_inliers   = 10; //ds 30 to test map merging on orazio dataset without odometry
    _max_depth_meters           = 5; //5 //3

    //ds allocate UVD aligner for odometry computation
    assert(_aligner != 0);

    //ds request initial tracking context for the tracker
    _context->createNewTrackingContext();

    //ds starting pose
    const TransformMatrix3D pose_start = TransformMatrix3D::Identity();
    _context->currentTrackingContext()->setRobotToWorldPrevious(pose_start);
    _odometry_motions = std::make_shared<std::list<TransformMatrix3D>>();
    _odometry_motions->clear();
    _odometry_motions->push_back(pose_start);
  }

  Tracker::~Tracker() {
    LOG_INFO("Tracker::Tracker", "destroying");
    //ds TODO clear tracker owned scopes
    LOG_INFO("Tracker::Tracker", "destroyed");
  }

  void Tracker::predictCvPoints(std::vector<cv::Point2f>& predictions, const Frame* previous_frame_, const Frame* current_frame_) const {
    assert(previous_frame_ != 0);
    assert(current_frame_ != 0);

    //ds preallocation
    predictions.resize(previous_frame_->points().size());
    const TransformMatrix3D transform_world_to_camera = current_frame_->camera()->robotToCamera()*current_frame_->worldToRobot();
    const Matrix3 KR = current_frame_->camera()->cameraMatrix()*transform_world_to_camera.linear();
    const Vector3 Kt = current_frame_->camera()->cameraMatrix()*transform_world_to_camera.translation();

    //ds compute predictions for all previous frame points
    for (Index index_point_previous = 0; index_point_previous < predictions.size(); ++index_point_previous) {
      const FramePoint* frame_point = previous_frame_->points()[index_point_previous];
      assert(frame_point->imageCoordinates().x() >= 0);
      assert(frame_point->imageCoordinates().x() <= current_frame_->camera()->imageCols());
      assert(frame_point->imageCoordinates().y() >= 0);
      assert(frame_point->imageCoordinates().y() <= current_frame_->camera()->imageRows());
      assert(frame_point->imageCoordinates().z() == 1);

      //ds default are previous frame coordinates
      predictions[index_point_previous].x = frame_point->imageCoordinates().x();
      predictions[index_point_previous].y = frame_point->imageCoordinates().y();
      if (!frame_point->landmark() || !frame_point->landmark()->isValidated()) {
        continue;
      }

      //ds project point in camera based on landmark coordinates
      const PointCoordinates point_in_camera = KR*frame_point->landmark()->coordinates()+Kt;
      if (point_in_camera.z() <= 0) {
        continue;
      }

      //ds project point onto image plane
      const PointCoordinates point_in_image = point_in_camera/point_in_camera.z();

      //ds check if invalid
      if (point_in_image.x() < 0 || point_in_image.x() > current_frame_->camera()->imageCols()||
          point_in_image.y() < 0 || point_in_image.y() > current_frame_->camera()->imageRows()) {
        continue;
      }
      assert(current_frame_->camera()->isInFieldOfView(point_in_image));

      //ds update prediction based on landmark position
      predictions[index_point_previous].x = point_in_camera.x();
      predictions[index_point_previous].y = point_in_camera.y();
    }
  }

  void Tracker::trackFeatures(const Frame* previous_frame_) const {
    assert(previous_frame_ != 0);
    Frame* current_frame = _context->currentTrackingContext()->currentFrame();
    assert(current_frame != 0);
    assert(previous_frame_ != 0);

    //ds retrieve point predictions on current image plane
    std::vector<cv::Point2f> points_predicted_from_previous_frame;
    predictCvPoints(points_predicted_from_previous_frame, previous_frame_, current_frame);

    //ds wrap into cv context
    std::vector<cv::Point2f> points_in_previous_frame;
    previous_frame_->toCvPoints(points_in_previous_frame);
    std::vector<uchar> status_tracked;
    std::vector<float> error;
    assert(points_in_previous_frame.size() == points_predicted_from_previous_frame.size());

    //ds track point features using LK
    calcOpticalFlowPyrLK(previous_frame_->intensityImage(),
                         current_frame->intensityImage(),
                         points_in_previous_frame,
                         points_predicted_from_previous_frame,
                         status_tracked,
                         error,
                         _optical_flow_win_size,
                         3,
                         _optical_flow_termcrit);

    //ds allocate space for tracked points (best case all tracked -> same size)
    current_frame->points().resize(previous_frame_->points().size());

    //ds control variable, for tracked points
    Index index_point_tracked = 0;

    //ds loop over all previous points
    for (Index index_point_previous = 0; index_point_previous < status_tracked.size(); ++index_point_previous) {

      //ds skip point if not tracked
      if (!status_tracked[index_point_previous]) continue;

      //ds grab previous point
      FramePoint* previous_point = previous_frame_->points()[index_point_previous];
      assert(previous_point != 0);

      //ds set keypoint in current frame
      cv::KeyPoint keypoint_current(previous_point->keypoint());
      keypoint_current.pt = points_predicted_from_previous_frame[index_point_previous];

      //ds validate optical flow tracking: skip point if invalid
      if (keypoint_current.pt.x < 0 || keypoint_current.pt.x > current_frame->camera()->imageCols()||
          keypoint_current.pt.y < 0 || keypoint_current.pt.y > current_frame->camera()->imageRows()) continue;

      assert(keypoint_current.pt.x >= 0);
      assert(keypoint_current.pt.x <= current_frame->camera()->imageCols());
      assert(keypoint_current.pt.y >= 0);
      assert(keypoint_current.pt.y <= current_frame->camera()->imageRows());

      //ds compute keypoint region for descriptor computation
      const gt_real keypoint_size      = 12*keypoint_current.size;
      const gt_real keypoint_size_half = keypoint_size/2;
      const gt_real box_x=std::max(keypoint_current.pt.x-keypoint_size_half, static_cast<gt_real>(0.0));
      const gt_real box_y=std::max(keypoint_current.pt.y-keypoint_size_half, static_cast<gt_real>(0.0));
      cv::KeyPoint keypoint_in_region_of_interest(keypoint_current);
      keypoint_in_region_of_interest.pt.x = keypoint_size_half;
      keypoint_in_region_of_interest.pt.y = keypoint_size_half;
      const cv::Rect region_of_interest(box_x,
                                        box_y,
                                        std::min(keypoint_size+1, current_frame->camera()->imageCols()-box_x),
                                        std::min(keypoint_size+1, current_frame->camera()->imageRows()-box_y));
      std::vector<cv::KeyPoint> keypoint_buffer(1, keypoint_in_region_of_interest);

      //ds compute descriptor in defined region
      cv::Mat descriptor;
      _context->currentTrackingContext()->descriptorExtractor()->compute(current_frame->intensityImage()(region_of_interest), keypoint_buffer, descriptor);

      //ds skip the point if we didn't manage to extract a descriptor (otherwise discard the track)
      if (keypoint_buffer.size() == 0) continue;

      assert(keypoint_buffer.size() == 1);
      assert(descriptor.rows == 1);

      //ds set the point to the control structure
      current_frame->points()[index_point_tracked] = current_frame->createNewPoint(keypoint_current, descriptor, previous_point);
      ++index_point_tracked;
    }

    //ds check if points were tracked
    if (index_point_tracked ){
      current_frame->points().resize(index_point_tracked);
    } else {
      current_frame->points().clear();
    }
  }

  void Tracker::extractFeatures() {

    //ds initialize white feature mask
    Frame* current_frame = _context->currentTrackingContext()->currentFrame();
    _feature_detection_mask.create(current_frame->intensityImage().rows,
                                   current_frame->intensityImage().cols,
                                   CV_8UC1);
    _feature_detection_mask = 1;

    //ds mask the mask
    std::vector<cv::Point2f> current_cv_points;
    current_frame->toCvPoints(current_cv_points);
    const cv::Point2f side(_min_feature_distance, _min_feature_distance);
    for (const cv::Point2f& current_cv_point: current_cv_points){
      cv::rectangle(_feature_detection_mask, current_cv_point-side, current_cv_point+side, cv::Scalar(0), CV_FILLED);
    }

    //ds evaluate masking: check available uncovered region
    int uncovered_region               = cv::countNonZero(_feature_detection_mask);
    int number_of_new_features_to_find = uncovered_region/(_min_feature_distance*_min_feature_distance);
    if (number_of_new_features_to_find <= 0)
      return;
    
    //ds detect new features
    std::vector<cv::KeyPoint> keypoints;

#if CV_MAJOR_VERSION == 2
    _feature_detector = cv::GFTTDetector(number_of_new_features_to_find, 0.01, _min_feature_distance, 5, false, 0.04);
    _feature_detector.detect(current_frame->intensityImage(), keypoints, _feature_detection_mask);
#elif CV_MAJOR_VERSION == 3
    _feature_detector = cv::GFTTDetector::create(number_of_new_features_to_find, 0.01, _min_feature_distance, 5, false, 0.04);
    _feature_detector->detect(current_frame->intensityImage(), keypoints, _feature_detection_mask);
#else
  #error OpenCV version not supported
#endif
    
    //ds extract descriptors for detected features
    cv::Mat descriptors;
    _context->currentTrackingContext()->descriptorExtractor()->compute(current_frame->intensityImage(), keypoints, descriptors);

    //ds add the new points to the old ones
    const Count number_of_features_previous = current_frame->points().size();
    current_frame->points().resize(number_of_features_previous+keypoints.size());
    for (Index index_keypoint = 0; index_keypoint < keypoints.size(); ++index_keypoint) {

      //ds set point to holder
      current_frame->points()[number_of_features_previous+index_keypoint] = current_frame->createNewPoint(keypoints[index_keypoint], descriptors.row(index_keypoint));
    }
  }
 
  void Tracker::updateDepth() const {
    Frame* current_frame                   = _context->currentTrackingContext()->currentFrame();
    const gt_real& depth_conversion_factor = current_frame->camera()->depthConversionFactor();
    const cv::Mat& depth_image             = current_frame->depthImage();

    //ds check depth for all active points
    for (FramePoint* frame_point: current_frame->points()) {
      const int column           = std::nearbyint(frame_point->imageCoordinates().x());
      const int row              = std::nearbyint(frame_point->imageCoordinates().y());
      const uint16_t pixel_depth = depth_image.at<uint16_t>(row, column);

      //ds if depth is not available, skip processing
      if (pixel_depth == 0) {
        continue;
      }

      //ds compute world depth
      const gt_real world_depth_meters = depth_conversion_factor*pixel_depth;
      assert(world_depth_meters > 0.0);

      //ds if acceptable
      if (world_depth_meters < _max_depth_meters) {

        //ds set depth to the point
        frame_point->setDepth(world_depth_meters);
      } else {

        //ds set depth prior to the point
        frame_point->setDepthByVision(world_depth_meters);
      }
    }
  }

  void Tracker::updateLandmarks() const {
    Frame* current_frame                     = _context->currentTrackingContext()->currentFrame();
    const TransformMatrix3D& camera_to_world = current_frame->robotToWorld()*current_frame->camera()->cameraToRobot();
    
    const Matrix3& inverse_camera_matrix = current_frame->camera()->inverseCameraMatrix();
    const Matrix3 A                      = camera_to_world.linear()*inverse_camera_matrix;
    const Vector3 b                      = camera_to_world.translation();

    //ds start landmark generation/update
    for (FramePoint* current_point: current_frame->points()) {

      //ds skip point if tracking and not mature enough to be a landmark - for localizing state this is skipped
      if (_status == Frame::Tracking && current_point->age() < Frame::minimum_landmark_age) {
        continue;
      }

      //ds initial update setup
      Landmark* landmark = current_point->landmark();
      Matrix3 omega(Matrix3::Identity());

      //ds initial guess
      gt_real depth_meters = current_point->depth();

      //ds check depth situation: regular depth
      if (current_point->hasDepth()) {

        //ds if no landmark yet - but point with depth information, create a new landmark using the given depth
        if (!landmark) {
          landmark = _context->currentTrackingContext()->createNewLandmark();
          current_point->setLandmark(landmark);
        }

        //ds depth must be within limits here
        assert(depth_meters > 0.0);
        assert(depth_meters < _max_depth_meters);

        //ds adjust omega to include full depth information
        omega(2,2) = _landmark_optimization_depth_weight/depth_meters;
        landmark->setIsByVision(false);
      }

      //ds depth obtained by vision
      else if (current_point->hasDepthByVision()) {

        //ds create a new landmark using the vision depth
        if (!landmark) {
          landmark = _context->currentTrackingContext()->createNewLandmark();
          current_point->setLandmark(landmark);
          landmark->setIsByVision(true);
        }

        //ds adjust omega to include full depth information
        omega(2,2) = _landmark_optimization_depth_weight/depth_meters;
      }

      //ds no depth available
      else {

        //ds if there's also no landmark: skip processing
        if (!landmark) {
          continue;
        }

        //ds we have a landmark, for which currently no depth information is available: ignore depth in optimization
        omega(2,2) = 0.0;

        //ds set depth to 'infinity', otherwise the optimization will throw up nan jacobians
        depth_meters = _infinity_depth_meters;
      }
      assert(depth_meters > 0.0);
      assert(landmark);

      //gg compute the prediction in world coordinates
      const PointCoordinates camera_point = depth_meters*current_point->imageCoordinates();

      //ds optimize landmark position based on UVD coordinates
      const PointCoordinates coordinates_in_world = A*camera_point+b;
      const gt_real u = current_point->imageCoordinates().x();
      const gt_real v = current_point->imageCoordinates().y();
      const gt_real d = depth_meters;

      //ds construct jacobian
      Matrix3 Ju(Matrix3::Identity());
      Ju <<
      d, 0, u,
      0, d, v,
      0, 0, 1;
      const Matrix3 J = (A*Ju).inverse();

      //ds build information matrix for eigensolving
      const Matrix3 information_matrix = J.transpose()*omega*J;
      landmark->update(coordinates_in_world,
                       information_matrix,
                       current_point->descriptor());
    }
  }

  const TransformMatrix3D Tracker::addImage(Camera* camera_,
                                            const cv::Mat& intensity_image_,
                                            const cv::Mat& depth_image_,
                                            const Identifier& sequence_number_raw_,
                                            const TransformMatrix3D& estimate_world_previous_to_current_){
    CHRONOMETER_START(overall)

    //ds retrieve odometry prior - might be identity
    TransformMatrix3D robot_to_world_current = _context->currentTrackingContext()->robotToWorldPrevious();
    if (_context->currentTrackingContext()->currentFrame()) {
      robot_to_world_current = estimate_world_previous_to_current_*robot_to_world_current;
      //_odometry_motions->push_back(estimate_world_previous_to_current_*_odometry_motions->back());
    }

    //ds create new frame (memory lock expected)
    Frame* current_frame = _context->currentTrackingContext()->createNewFrame(robot_to_world_current, sequence_number_raw_);
    current_frame->setCamera(camera_);                  //ds we have a camera
    current_frame->setIntensityImage(intensity_image_); //ds we have RGB information
    current_frame->setDepthImage(depth_image_);         //ds we have depth information
    //current_frame->setFrameToWorldOdometry(_odometry_motions->back());

    //ds if we have a previously processed frame in the chain
    if (current_frame->previous()) {

      //ds attempt to track the points from the previous frame
      trackFeatures(current_frame->previous());
      updateDepth();
      //std::ofstream log("points.txt", std::ofstream::out | std::ofstream::app);
      //log << seq << " " << current_frame->index() << " " << _context->currentTrackingContext()->keyframes().size() << " " << current_frame->points().size() << "\n";
      //log.close();
    }

    //ds updated odometry change
    TransformMatrix3D world_previous_to_current(TransformMatrix3D::Identity());

    //ds check tracker status
    switch(_status) {

      //ds track lost - localizing
      case Frame::Localizing: {
        LOG_INFO("Tracker::addImage", "STATE: LOCALIZING");

        //gg count the number of points old enough and with a good depth
        const Count number_of_good_points = current_frame->countPoints(Frame::minimum_landmark_age, Unknown, True);
        if (number_of_good_points > _min_num_landmarks_to_track) {
          updateLandmarks();
          _status = Frame::Tracking;
        }
        break;
      }

      //ds on the track
      case Frame::Tracking: {

        //gg count the number of points old enough and with a landmark
        const Count number_of_good_points = current_frame->countPoints(Frame::minimum_landmark_age, True, Unknown);
        if (number_of_good_points < _min_num_landmarks_to_track) {

          std::cerr << "LOST TRACK due to insufficient good points" << std::endl;
          _status = Frame::Localizing;
          current_frame->setStatus(_status);
          _context->createNewTrackingContext();
          CHRONOMETER_STOP(overall)
          return world_previous_to_current;
        }

        //ds call pose solver
        _context->currentTrackingContext()->landmarks().clearActive();
        _aligner->init(current_frame);
        for (Count iteration = 0; iteration < _maximum_number_of_iterations_solver; ++iteration) {
          _aligner->oneRound(iteration > (_maximum_number_of_iterations_solver-5));
        }

        //ds update current frame points: use only inliers or ones with a low error
        Index index_inlier = 0;
        for (Index index_point = 0; index_point < current_frame->points().size(); index_point++) {

          //ds keep the point if it has been skipped (due to insufficient maturity) or is an inlier
          if (_aligner->errors()[index_point] == -1 || _aligner->inliers()[index_point]) {
            current_frame->points()[index_inlier] = current_frame->points()[index_point];
            index_inlier++;
          }
        }

        //ds resize point containers
        if (current_frame->points().size()) {
          current_frame->points().resize(index_inlier);
        } else {
          current_frame->points().clear();
        }
        assert(index_inlier >= _aligner->numberOfInliers());

//        std::cerr << "inliers: " << _solver.numReprojectionInliers() << " | outliers: " << _solver.numReprojectionOutliers()
//                  << " | by depth: " << _solver.numberOfPointsWithDepth()
//                  << " | by vision: " << _solver.numberOfPointsWithDepthByVision()
//                  << " | none: " << _solver.numberOfPointsWithoutDepth()
//                  << " | invalid: " << _solver.numberOfPointsNotValidated() << std::endl;
//        std::cerr << "angular: " << delta_angular << std::endl;
//        std::cerr << "translational: " << delta_translational << std::endl;

        //ds if the pose computation is acceptable
        if (_aligner->numberOfInliers() > _min_reprojection_inliers) {

          //ds compute solver deltas
          world_previous_to_current         = _aligner->robotToWorld()*current_frame->previous()->robotToWorld().inverse();
          const gt_real delta_angular       = Utility::toOrientationRodrigues(world_previous_to_current.linear()).norm();
          const gt_real delta_translational = world_previous_to_current.translation().norm();

          //ds if the posit result is significant enough
          if (delta_angular > 0.001 || delta_translational > 0.01) {

            //ds update robot pose with posit result
            current_frame->setRobotToWorld(_aligner->robotToWorld());
          } else {

            //ds keep original pose
            world_previous_to_current = TransformMatrix3D::Identity();
            current_frame->setRobotToWorld(current_frame->previous()->robotToWorld());
          }

          //ds update tracks
          _context->currentTrackingContext()->setRobotToWorldPrevious(current_frame->robotToWorld());
          updateLandmarks();
          _status = Frame::Tracking;

          //ds update point coordinates in robot frame
          const TransformMatrix3D transform_camera_to_robot = current_frame->camera()->cameraToRobot();
          const Matrix3 inverse_camera_matrix               = current_frame->camera()->inverseCameraMatrix();
          const Matrix3 A = transform_camera_to_robot.linear()*inverse_camera_matrix;
          const Vector3 b = transform_camera_to_robot.translation();
          for (FramePoint* frame_point: current_frame->points()) {

            //ds only for valid depth points
            if (frame_point->hasDepth()) {
              assert(frame_point->depth() > 0);

              //ds get point in robot frame (for frame matching)
              PointCoordinates camera_point = frame_point->depth()*frame_point->imageCoordinates();
              assert(frame_point->imageCoordinates().z() == 1);
              const PointCoordinates point_in_frame = A*camera_point+b;
              frame_point->setRobotCoordinates(point_in_frame);
            }
          }
        } else {

          std::cerr << "LOST TRACK due to insufficient posit - inlier count: " << _aligner->numberOfInliers() << std::endl;

          //ds pose solving failed: lost track
          _status = Frame::Localizing;
          current_frame->setStatus(_status);
          _context->createNewTrackingContext();
          CHRONOMETER_STOP(overall)
          return world_previous_to_current;
        }
        break;
      }

      default: {
        std::runtime_error("invalid tracker state");
      }
    }
    
    //ds compute delta
//    const TransformMatrix3D transform_previous_to_current = pose_frame_to_world_current*_context->currentTrackingContext()->frameToWorldPrevious().inverse();
//    std::cerr << "---------------------------------------------------" << std::endl;
//    std::cerr << "ODOMETRY: " << std::endl;
//    std::cerr << initial_guess_transform_previous_to_current_.matrix() << std::endl;
//    std::cerr << "POSIT: " << std::endl;
//    std::cerr << transform_previous_to_current.matrix() << std::endl;

    //ds retrieve new points and update depth again (redudant loop over previously existing points)
    assert(current_frame != 0);
    extractFeatures();
    updateDepth();
    current_frame->setStatus(_status);

    //ds keyframe generation
    if (_context->currentTrackingContext()->previousFrame()) {
      _context->currentTrackingContext()->createNewKeyframe();
    }

    CHRONOMETER_STOP(overall)
    return world_previous_to_current;
  }
}
