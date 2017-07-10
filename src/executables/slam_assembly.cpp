#include "slam_assembly.h"

#include "srrg_messages/pinhole_image_message.h"
#include "aligners/stereouv_aligner.h"
#include "aligners/uvd_aligner.h"
#include "motion_estimation/stereo_tracker.h"
#include "motion_estimation/depth_tracker.h"

namespace proslam {

SLAMAssembly::SLAMAssembly(ParameterCollection* parameters_): _parameters(parameters_),
                                                              _world_map(new WorldMap(_parameters->world_map_parameters)),
                                                              _optimizer(new GraphOptimizer()),
                                                              _relocalizer(new Relocalizer()),
                                                              _tracker(0),
                                                              _camera_left(0),
                                                              _camera_right(0),
                                                              _ui_server(0),
                                                              _image_viewer(0),
                                                              _map_viewer(0),
                                                              _minimap_viewer(0),
                                                              _is_termination_requested(false),
                                                              _is_viewer_open(false) {
  _relocalizer->configure(_parameters->relocalizer_parameters);
  _synchronizer.reset();
  _robot_to_world_ground_truth_poses.clear();
}

SLAMAssembly::~SLAMAssembly() {
  delete _tracker;
  delete _optimizer;
  delete _relocalizer;
  delete _world_map;

  //ds free cameras
  if (_camera_left) {delete _camera_left;}
  if (_camera_right) {delete _camera_right;}

  //ds free viewers if set
  if (_image_viewer) {delete _image_viewer;}
  if (_map_viewer) {delete _map_viewer;}
  if (_minimap_viewer) {delete _minimap_viewer;}
  if (_ui_server) {delete _ui_server;}
  _robot_to_world_ground_truth_poses.clear();
}

void SLAMAssembly::initializeMessageFile() {

  //ds check dataset length
  if (_parameters->command_line_parameters->dataset_file_name.length() == 0) {
    LOG_ERROR(std::cerr << "SLAMAssembly::initializeMessageFile|no dataset provided (enter -h for help)" << std::endl)
    throw std::runtime_error("no dataset provided");
  }

  //ds configure sensor message source
  _message_reader.open(_parameters->command_line_parameters->dataset_file_name);

  //ds terminate on failure
  if (!_message_reader.good()) {
    LOG_INFO(std::cerr << _parameters->banner << std::endl)
    throw std::runtime_error("unable to open dataset");
  }
}

void SLAMAssembly::_createStereoTracker(Camera* camera_left_, Camera* camera_right_){

  //ds if rectification is desired
  if (_parameters->command_line_parameters->option_undistort_and_rectify) {

    //ds sanity check
    if ((camera_left_->projectionMatrix().block<3,3>(0,0) - camera_right_->projectionMatrix().block<3,3>(0,0)).squaredNorm() != 0) {
      LOG_ERROR(std::cerr << "SLAMAssembly::_createStereoTracker|provided mismatching projection matrices" << std::endl)
      throw std::runtime_error("mismatching projection matrices");
    }

    //ds replace camera matrices with identical one
    camera_left_->setCameraMatrix(camera_left_->projectionMatrix().block<3,3>(0,0));
    camera_right_->setCameraMatrix(camera_left_->cameraMatrix());
  }

  //ds allocate and configure the framepoint generator
  StereoFramePointGenerator* framepoint_generator = new StereoFramePointGenerator();
  framepoint_generator->setCameraLeft(camera_left_);
  framepoint_generator->setCameraRight(camera_right_);
  framepoint_generator->configure(_parameters->stereo_framepoint_generator_parameters);

  //ds allocate and configure the aligner for motion estimation
  StereoUVAligner* pose_optimizer = new StereoUVAligner();
  pose_optimizer->configure(_parameters->stereo_tracker_parameters->aligner);

  //ds allocate and configure the tracker
  StereoTracker* tracker = new StereoTracker();
  tracker->setCameraLeft(camera_left_);
  tracker->setCameraRight(camera_right_);
  tracker->setFramePointGenerator(framepoint_generator);
  tracker->setAligner(pose_optimizer);
  tracker->configure(_parameters->stereo_tracker_parameters);
  _tracker = tracker;
  _tracker->setWorldMap(_world_map);
}

void SLAMAssembly::_createDepthTracker(const Camera* camera_left_, const Camera* camera_right_){

  //ds allocate and configure the framepoint generator
  DepthFramePointGenerator* framepoint_generator = new DepthFramePointGenerator();
  framepoint_generator->setCameraLeft(camera_left_);
  framepoint_generator->setCameraRight(camera_right_);
  framepoint_generator->configure(_parameters->depth_framepoint_generator_parameters);

  //ds allocate and configure the aligner for motion estimation
  UVDAligner* pose_optimizer = new UVDAligner();
  pose_optimizer->configure(_parameters->depth_tracker_parameters->aligner);

  //ds allocate and configure the tracker
  DepthTracker* tracker = new DepthTracker();
  tracker->setCameraLeft(camera_left_);
  tracker->setDepthCamera(camera_right_);
  tracker->setFramePointGenerator(framepoint_generator);
  tracker->setAligner(pose_optimizer);
  tracker->configure(_parameters->depth_tracker_parameters);
  _tracker = tracker;
  _tracker->setWorldMap(_world_map);
}

void SLAMAssembly::loadCamerasFromMessageFile() {

  //ds configure message synchronizer
  std::vector<std::string> camera_topics_synchronized(0);
  camera_topics_synchronized.push_back(_parameters->command_line_parameters->topic_image_left);
  camera_topics_synchronized.push_back(_parameters->command_line_parameters->topic_image_right);
  _synchronizer.setTimeInterval(0.001);
  _synchronizer.setTopics(camera_topics_synchronized);

  //ds quickly read the first messages to buffer camera info
  srrg_core::BaseMessage* message = 0;
  while ((message = _message_reader.readMessage())) {
    srrg_core::BaseSensorMessage* sensor_message = dynamic_cast<srrg_core::BaseSensorMessage*>(message);
    assert(sensor_message != 0);
    sensor_message->untaint();

    //ds check for the two set topics
    if (sensor_message->topic() == _parameters->command_line_parameters->topic_image_left) {
      srrg_core::PinholeImageMessage* message_image_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_message);

      //ds allocate a new camera
      _camera_left = new Camera(message_image_left->image().rows,
                                message_image_left->image().cols,
                                message_image_left->cameraMatrix().cast<real>(),
                                message_image_left->offset().cast<real>());
    } else if (sensor_message->topic() == _parameters->command_line_parameters->topic_image_right) {
      srrg_core::PinholeImageMessage* message_image_right  = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_message);

      //ds allocate a new camera
      _camera_right = new Camera(message_image_right->image().rows,
                                 message_image_right->image().cols,
                                 message_image_right->cameraMatrix().cast<real>(),
                                 message_image_right->offset().cast<real>());
    }
    delete sensor_message;

    //ds if we got all the information we need
    if (_camera_left && _camera_right) {
      break;
    }
  }
  _message_reader.close();

  //ds terminate on failure
  if (!_camera_left) {
    LOG_ERROR(std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|left camera not set" << std::endl)
    throw std::runtime_error("left camera not set");
  }
  if (!_camera_right) {
    LOG_ERROR(std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|right camera not set" << std::endl)
    throw std::runtime_error("right camera not set");
  }
  if (_camera_left->imageCols() == 0 || _camera_left->imageRows() == 0) {
    LOG_ERROR(std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|left camera images not set" << std::endl)
    throw std::runtime_error("left camera images not set");
  }
  if (_camera_right->imageCols() == 0 || _camera_right->imageRows() == 0) {
    LOG_ERROR(std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|right camera images not set" << std::endl)
    throw std::runtime_error("right camera images not set");
  }

  //ds check if we have to modify the cameras - if stereo from txt_io
  if (_parameters->command_line_parameters->tracker_mode == CommandLineParameters::TrackerMode::RGB_STEREO) {

    //ds reconstruct projection matrix from camera matrices (encoded in txt_io)
    ProjectionMatrix projection_matrix(ProjectionMatrix::Identity());
    projection_matrix.block<3,3>(0,0) = _camera_left->cameraMatrix()*_camera_left->robotToCamera().linear();

    //ds set left
    _camera_left->setProjectionMatrix(projection_matrix);

    //ds sanity check
    if ((_camera_left->cameraMatrix()-_camera_right->cameraMatrix()).squaredNorm() != 0) {
      LOG_ERROR(std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|provided mismatching camera matrices" << std::endl)
      throw std::runtime_error("provided mismatching camera matrices");
    }

    //ds compute right camera with baseline offset
    projection_matrix.block<3,1>(0,3) = _camera_left->cameraMatrix()*_camera_right->robotToCamera().translation();
    _camera_right->setProjectionMatrix(projection_matrix);
  }

  //ds load cameras to assembly
  loadCameras(_camera_left, _camera_right);
}

void SLAMAssembly::loadCameras(Camera* camera_left_, Camera* camera_right_) {

  //ds if tracker is set
  if (!_tracker) {

    //ds allocate the tracker module with the given cameras
    switch (_parameters->command_line_parameters->tracker_mode){
      case CommandLineParameters::TrackerMode::RGB_STEREO: {
        _createStereoTracker(camera_left_, camera_right_);
        break;
      }
      case CommandLineParameters::TrackerMode::RGB_DEPTH: {
        _createDepthTracker(camera_left_, camera_right_);
        break;
      }
      default: {
        throw std::runtime_error("unknown tracker");
      }
    }
  }

  LOG_INFO(std::cerr << "SLAMAssembly::loadCameras|loaded cameras: " << 2 << std::endl)
  LOG_INFO(std::cerr << "SLAMAssembly::loadCameras|LEFT resolution: " << camera_left_->imageCols() << " x " << camera_left_->imageRows()
            << ", aspect ratio: " << static_cast<real>(camera_left_->imageCols())/camera_left_->imageRows() << std::endl)
  LOG_INFO(std::cerr << "SLAMAssembly::loadCameras|RIGHT resolution: " << camera_right_->imageCols() << " x " << camera_right_->imageRows()
            << ", aspect ratio: " << static_cast<real>(camera_right_->imageCols())/camera_right_->imageRows() << std::endl)
}

void SLAMAssembly::initializeGUI(QApplication* ui_server_) {
  if (_parameters->command_line_parameters->option_use_gui) {
    _ui_server = ui_server_;
    _image_viewer = new ImageViewer("input: images [OpenCV]");
    _map_viewer   = new MapViewer(0.25, "output: map [OpenGL]");
    _map_viewer->setCameraLeftToRobot(_camera_left->cameraToRobot());

    //ds orientation flip for proper camera following
    TransformMatrix3D orientation_correction(TransformMatrix3D::Identity());
    orientation_correction.matrix() << 0, -1, 0, 0,
                                       -1, 0, 0, 0,
                                       0, 0, -1, 0,
                                       0, 0, 0, 1;
    _map_viewer->setRotationRobotView(orientation_correction);
    _map_viewer->setWorldMap(_world_map);
    _map_viewer->show();

    //ds configure custom top viewer if requested
    if (_parameters->command_line_parameters->option_show_top_viewer) {
      _minimap_viewer = new MapViewer(1, "minimap [OpenGL]");
      _minimap_viewer->setCameraLeftToRobot(_camera_left->cameraToRobot());
      TransformMatrix3D center_for_kitti_sequence_00;
      center_for_kitti_sequence_00.matrix() << 1, 0, 0, 0,
                                               0, 0, -1, 200,
                                               0, 1, 0, 800,
                                               0, 0, 0, 1;
      _minimap_viewer->setWorldToRobotOrigin(center_for_kitti_sequence_00);
      _minimap_viewer->setFollowRobot(false);
      _minimap_viewer->setWorldToRobotOrigin(orientation_correction*center_for_kitti_sequence_00);
      _minimap_viewer->setWorldMap(_world_map);
      _minimap_viewer->show();
    }
    _is_viewer_open = _map_viewer->isVisible();
  }
}

void SLAMAssembly::updateGUI() {
  if (_parameters->command_line_parameters->option_use_gui) {
    _image_viewer->update(_world_map->currentFrame());
    _map_viewer->update(_world_map->currentFrame());

    //ds as long as stepwise playback is desired and no steps are set
    while (_map_viewer->optionStepwisePlayback() && _map_viewer->requestedPlaybackSteps() == 0) {

      //ds check if termination is requested
      if (_is_termination_requested) {
        break;
      }

      //ds we have to wait
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    //ds if steps are set
    if (_map_viewer->requestedPlaybackSteps() > 0) {
      _map_viewer->decrementPlaybackSteps();
    }
  }
}

void SLAMAssembly::draw() {

  //ds check if we still got an active GUI master
  _is_viewer_open = _map_viewer->isVisible();

  //ds if the GUI is running
  if (_is_viewer_open) {
    _image_viewer->draw();
    _map_viewer->updateGL();
    if (_minimap_viewer) {
      _minimap_viewer->updateGL();
    }
    _ui_server->processEvents();
  }
}

void SLAMAssembly::playbackMessageFile() {

  //ds restart stream
  _message_reader.open(_parameters->command_line_parameters->dataset_file_name);
  _robot_to_world_ground_truth_poses.clear();

  //ds frame counts
  Count number_of_processed_frames_total   = 0;
  Count number_of_processed_frames_current = 0;

  //ds store start time
  double time_start_seconds                          = srrg_core::getTime();
  const double time_start_seconds_first              = srrg_core::getTime();
  const double runtime_info_update_frequency_seconds = 5;

  //ds visualization/start point
  const TransformMatrix3D robot_to_camera_left(_camera_left->robotToCamera());

  //ds start playback
  srrg_core::BaseMessage* message = 0;
  while ((message = _message_reader.readMessage())) {
    srrg_core::BaseSensorMessage* sensor_message = dynamic_cast<srrg_core::BaseSensorMessage*>(message);
    assert(sensor_message != 0);
    sensor_message->untaint();

    //ds add to synchronizer
    if (sensor_message->topic() == _parameters->command_line_parameters->topic_image_left) {
      _synchronizer.putMessage(sensor_message);
    } else if (sensor_message->topic() == _parameters->command_line_parameters->topic_image_right) {
      _synchronizer.putMessage(sensor_message);
    } else {
      delete sensor_message;
    }

    //ds if we have a synchronized package of sensor messages ready
    if (_synchronizer.messagesReady()) {

      //ds check if termination is requested
      if (_is_termination_requested) {
        break;
      }

      //ds buffer sensor data
      srrg_core::PinholeImageMessage* image_message_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(_synchronizer.messages()[0].get());
      srrg_core::PinholeImageMessage* image_message_right = dynamic_cast<srrg_core::PinholeImageMessage*>(_synchronizer.messages()[1].get());

      //ds buffer images
      cv::Mat intensity_image_left_rectified;
      if(image_message_left->image().type() == CV_8UC3){
        cvtColor(image_message_left->image(), intensity_image_left_rectified, CV_BGR2GRAY);
      } else {
        intensity_image_left_rectified = image_message_left->image();
      }
      cv::Mat intensity_image_right_rectified;
      if (_parameters->command_line_parameters->tracker_mode == CommandLineParameters::TrackerMode::RGB_STEREO && image_message_right->image().type() == CV_8UC3) {
        cvtColor(image_message_right->image(), intensity_image_right_rectified, CV_BGR2GRAY);
      } else {
        intensity_image_right_rectified = image_message_right->image();
      }

      //ds preprocess the images if desired
      if (_parameters->command_line_parameters->option_equalize_histogram) {
        cv::equalizeHist(intensity_image_left_rectified, intensity_image_left_rectified);
        if (_parameters->command_line_parameters->tracker_mode == CommandLineParameters::TrackerMode::RGB_STEREO) {
          cv::equalizeHist(intensity_image_right_rectified, intensity_image_right_rectified);
        }
      }

      //ds check if first frame and odometry is available
      if (_world_map->frames().size() == 0 && image_message_left->hasOdom()) {
        _world_map->setRobotToWorld(image_message_left->odometry().cast<real>()*robot_to_camera_left);
        if (_parameters->command_line_parameters->option_use_gui) {
          _map_viewer->setWorldToRobotOrigin(_world_map->robotToWorld().inverse());
        }
      }

      //ds progress SLAM with the new images
      process(intensity_image_left_rectified,
              intensity_image_right_rectified,
              image_message_left->hasOdom() && _parameters->command_line_parameters->option_use_odometry,
              image_message_left->odometry().cast<real>());

      //ds record ground truth history for error computation
      if (image_message_left->hasOdom()) {
        addGroundTruthMeasurement(image_message_left->odometry().cast<real>()*robot_to_camera_left);
      }

      //ds runtime info
      ++number_of_processed_frames_total;
      ++number_of_processed_frames_current;
      const double total_duration_seconds_current = srrg_core::getTime()-time_start_seconds;
      if (total_duration_seconds_current > runtime_info_update_frequency_seconds) {

        //ds runtime info - depending on set modes
        if (_parameters->command_line_parameters->option_use_relocalization) {
          LOG_INFO(std::printf("SLAMAssembly::playbackMessageFile|frames: %5lu <FPS: %6.2f>|landmarks: %6lu|local maps: %4lu (%3.2f)|closures: %3lu (%3.2f)\n",
                      number_of_processed_frames_total,
                      number_of_processed_frames_current/total_duration_seconds_current,
                      _world_map->landmarks().size(),
                      _world_map->localMaps().size(),
                      _world_map->localMaps().size()/static_cast<real>(number_of_processed_frames_total),
                      _world_map->numberOfClosures(),
                      _world_map->numberOfClosures()/static_cast<real>(_world_map->localMaps().size())))
        } else {
          LOG_INFO(std::printf("SLAMAssembly::playbackMessageFile|frames: %5lu <FPS: %6.2f>|landmarks: %6lu\n",
                      number_of_processed_frames_total,
                      number_of_processed_frames_current/total_duration_seconds_current,
                      _world_map->landmarksInWindowForLocalMap().size()))
        }

        //ds reset stats for new measurement window
        time_start_seconds                 = srrg_core::getTime();
        number_of_processed_frames_current = 0;
      }

      image_message_left->release();
      image_message_right->release();
      _synchronizer.reset();

      //ds update gui (no effect if no GUI is active)
      updateGUI();

      //ds free image references (GUI gets copies)
      intensity_image_left_rectified.release();
      intensity_image_right_rectified.release();
    }
  }
  _duration_total_seconds = srrg_core::getTime()-time_start_seconds_first;
  _message_reader.close();
  LOG_INFO(std::cerr << "SLAMAssembly::playbackMessageFile|dataset completed" << std::endl)
}

void SLAMAssembly::addGroundTruthMeasurement(const TransformMatrix3D& robot_to_world_ground_truth_) {
  _robot_to_world_ground_truth_poses.push_back(robot_to_world_ground_truth_);
  _world_map->setRobotToWorldGroundTruth(robot_to_world_ground_truth_);
}

void SLAMAssembly::process(const cv::Mat& intensity_image_left_,
                           const cv::Mat& intensity_image_right_,
                           const bool& use_odometry_,
                           const TransformMatrix3D& odometry_) {

  //ds call the tracker
  _tracker->setIntensityImageLeft(&intensity_image_left_);

  //ds depending on tracking mode
  switch (_parameters->command_line_parameters->tracker_mode){
    case CommandLineParameters::TrackerMode::RGB_STEREO: {
      StereoTracker* stereo_tracker = dynamic_cast<StereoTracker*>(_tracker);
      assert(stereo_tracker);
      stereo_tracker->setIntensityImageRight(&intensity_image_right_);
      break;
    }
    case CommandLineParameters::TrackerMode::RGB_DEPTH: {
      DepthTracker* depth_tracker = dynamic_cast<DepthTracker*>(_tracker);
      assert(depth_tracker);
      depth_tracker->setDepthImageRight(&intensity_image_right_);
      break;
    }
    default: {
      throw std::runtime_error("unknown tracker");
    }
  }
  if (use_odometry_) {
    _tracker->setOdometry(odometry_);
  }

  //ds track framepoints and derive new robot pose
  _tracker->compute();

  //ds check if relocalization is desired
  if (_parameters->command_line_parameters->option_use_relocalization) {

    //ds if we have a valid frame (not the case after the track is lost)
    if (_world_map->currentFrame()) {

      //ds local map generation - regardless of tracker state
      if (_world_map->createLocalMap(_parameters->command_line_parameters->option_drop_framepoints)) {

        //ds trigger relocalization
        _relocalizer->initialize(_world_map->currentLocalMap());
        _relocalizer->detect();
        _relocalizer->compute();

        //ds check the closures
        for(LocalMapCorrespondence* closure: _relocalizer->closures()) {
          if (closure->is_valid) {
            assert(_world_map->currentLocalMap() == closure->local_map_query);

            //ds add loop closure constraint (also detects if local maps from two different tracks are connected)
            _world_map->addCorrespondence(_world_map->currentLocalMap(),
                                          closure->local_map_reference,
                                          closure->query_to_reference,
                                          closure->icp_inlier_ratio);
            if (_parameters->command_line_parameters->option_use_gui) {
              for (const LandmarkCorrespondence* match: closure->correspondences) {
                _world_map->landmarks().get(match->query->landmark->identifier())->setIsInLoopClosureQuery(true);
                _world_map->landmarks().get(match->reference->landmark->identifier())->setIsInLoopClosureReference(true);
              }
            }
          }
        }
        _relocalizer->train();
      }

      //ds check if we closed a local map
      if (_world_map->relocalized()) {

        //ds check if we're running with a GUI
        if (_map_viewer) {

          //ds lock the GUI before the critical phase
          _map_viewer->lock();
        }

        //ds optimize graph
        _optimizer->optimize(_world_map);

        //ds reenable the GUI
        if (_map_viewer) {_map_viewer->unlock();}
      }
    }
  } else {

    //ds open loop options: save memory
    if (_parameters->command_line_parameters->option_drop_framepoints) {

      //ds free framepoints if available (previous is still required to draw the optical flow from current to previous point)
      if (_world_map->currentFrame()->previous() && _world_map->currentFrame()->previous()->previous()) {
        _world_map->currentFrame()->previous()->previous()->clear();
      }
    }
  }
}

void SLAMAssembly::printReport() const {

  //ds header
  const Count number_of_processed_frames_total = _world_map->frames().size();
  std::cerr << DOUBLE_BAR << std::endl;
  std::cerr << "performance summary" << std::endl;
  std::cerr << BAR << std::endl;

  //ds if not at least 2 frames were processed - exit right away
  if (number_of_processed_frames_total <= 1) {
    std::cerr << "no frames processed" << std::endl;
    std::cerr << DOUBLE_BAR << std::endl;
    return;
  }

  //ds if we got consistent ground truth data
  if (_robot_to_world_ground_truth_poses.size() == number_of_processed_frames_total) {

    //ds compute squared errors
    std::vector<real> errors_translation_relative(0);
    std::vector<real> squared_errors_translation_absolute(0);
    TransformMatrix3D odometry_robot_to_world_previous_ground_truth = TransformMatrix3D::Identity();
    Index index_frame     = 0;
    Frame* previous_frame = 0;
    for (FramePointerMapElement frame: _world_map->frames()) {

      //ds compute squared errors between frames
      if (index_frame > 0) {
        const TransformMatrix3D world_previous_to_current_ground_truth = _robot_to_world_ground_truth_poses[index_frame]*odometry_robot_to_world_previous_ground_truth.inverse();
        errors_translation_relative.push_back(((frame.second->robotToWorld()*previous_frame->robotToWorld().inverse()).translation()-world_previous_to_current_ground_truth.translation()).norm());
      }
      squared_errors_translation_absolute.push_back((frame.second->robotToWorld().translation()-_robot_to_world_ground_truth_poses[index_frame].translation()).squaredNorm());
      odometry_robot_to_world_previous_ground_truth = _robot_to_world_ground_truth_poses[index_frame];
      previous_frame = frame.second;
      index_frame++;
    }

    //ds compute RMSEs
    real root_mean_squared_error_translation_absolute = 0;
    for (const real squared_error: squared_errors_translation_absolute) {
      root_mean_squared_error_translation_absolute += squared_error;
    }
    root_mean_squared_error_translation_absolute /= squared_errors_translation_absolute.size();
    root_mean_squared_error_translation_absolute = std::sqrt(root_mean_squared_error_translation_absolute);
    real mean_error_translation_relative = 0;
    for (const real error: errors_translation_relative) {
      mean_error_translation_relative += error;
    }
    mean_error_translation_relative /= errors_translation_relative.size();

    std::cerr << "    absolute translation RMSE (m): " << root_mean_squared_error_translation_absolute << std::endl;
    std::cerr << "    relative translation   ME (m): " << mean_error_translation_relative << std::endl;
    std::cerr << "    final translational error (m): " << (_world_map->currentFrame()->robotToWorld().translation()-odometry_robot_to_world_previous_ground_truth.translation()).norm() << std::endl;
    std::cerr << BAR << std::endl;
  }

  //ds compute trajectory length
  double trajectory_length = 0;
  for (FramePointerMapElement frame: _world_map->frames()) {
    if (frame.second->previous()) {
      trajectory_length += (frame.second->worldToRobot()*frame.second->previous()->robotToWorld()).translation().norm();
    }
  }

  //ds general stats
  std::cerr << "      total trajectory length (m): " << trajectory_length << std::endl;
  std::cerr << "                     total frames: " << number_of_processed_frames_total << std::endl;
  std::cerr << "    total processing duration (s): " << _duration_total_seconds << std::endl;
  std::cerr << "                      average FPS: " << number_of_processed_frames_total/_duration_total_seconds << std::endl;
  std::cerr << "          average velocity (km/h): " << 3.6*trajectory_length/_duration_total_seconds << std::endl;
  std::cerr << "average processing time (s/frame): " << _duration_total_seconds/number_of_processed_frames_total << std::endl;
  std::cerr << "average landmarks close per frame: " << _tracker->totalNumberOfLandmarksClose()/number_of_processed_frames_total << std::endl;
  std::cerr << "  average landmarks far per frame: " << _tracker->totalNumberOfLandmarksFar()/number_of_processed_frames_total << std::endl;
  std::cerr << "         average tracks per frame: " << _tracker->totalNumberOfTrackedPoints()/number_of_processed_frames_total << std::endl;
  std::cerr << "        average tracks per second: " << _tracker->totalNumberOfTrackedPoints()/_duration_total_seconds << std::endl;
  std::cerr << BAR << std::endl;

  //ds computational costs
  std::cerr << std::endl;
  std::cerr << "time consumption overview - processing units" << std::endl;
  std::cerr << BAR << std::endl;
  std::cerr << "           module name | relative | absolute (s)" << std::endl;
  std::cerr << BAR << std::endl;
  std::printf("    keypoint detection | %f | %f\n", _tracker->framepointGenerator()->getTimeConsumptionSeconds_keypoint_detection()/_duration_total_seconds,
                                                         _tracker->framepointGenerator()->getTimeConsumptionSeconds_keypoint_detection());
  std::printf(" descriptor extraction | %f | %f\n", _tracker->framepointGenerator()->getTimeConsumptionSeconds_descriptor_extraction()/_duration_total_seconds,
                                                         _tracker->framepointGenerator()->getTimeConsumptionSeconds_descriptor_extraction());

  //ds display further information depending on tracking mode
  switch (_parameters->command_line_parameters->tracker_mode){
    case CommandLineParameters::TrackerMode::RGB_STEREO: {
      StereoFramePointGenerator* stereo_framepoint_generator = dynamic_cast<StereoFramePointGenerator*>(_tracker->framepointGenerator());
      std::printf("stereo keypoint search | %f | %f\n", stereo_framepoint_generator->getTimeConsumptionSeconds_point_triangulation()/_duration_total_seconds,
                                                             stereo_framepoint_generator->getTimeConsumptionSeconds_point_triangulation());
      break;
    }
    case CommandLineParameters::TrackerMode::RGB_DEPTH: {
      break;
    }
    default: {
      break;
    }
  }

  std::printf("              tracking | %f | %f\n", _tracker->getTimeConsumptionSeconds_tracking()/_duration_total_seconds, _tracker->getTimeConsumptionSeconds_tracking());
  std::printf("     pose optimization | %f | %f\n", _tracker->getTimeConsumptionSeconds_pose_optimization()/_duration_total_seconds, _tracker->getTimeConsumptionSeconds_pose_optimization());
  std::printf(" landmark optimization | %f | %f\n", _tracker->getTimeConsumptionSeconds_landmark_optimization()/_duration_total_seconds, _tracker->getTimeConsumptionSeconds_landmark_optimization());
  std::printf("        point recovery | %f | %f\n", _tracker->getTimeConsumptionSeconds_point_recovery()/_duration_total_seconds, _tracker->getTimeConsumptionSeconds_point_recovery());
  std::printf("        relocalization | %f | %f\n", _relocalizer->getTimeConsumptionSeconds_overall()/_duration_total_seconds, _relocalizer->getTimeConsumptionSeconds_overall());
  std::printf("            map update | %f | %f\n", _optimizer->getTimeConsumptionSeconds_overall()/_duration_total_seconds, _optimizer->getTimeConsumptionSeconds_overall());
  std::cerr << DOUBLE_BAR << std::endl;
}

void SLAMAssembly::reset() {
  _world_map->clear();
}
}
