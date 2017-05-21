#include "slam_assembly.h"

#include "srrg_txt_io/pinhole_image_message.h"
#include "aligners/stereouv_aligner.h"
#include "aligners/uvd_aligner.h"
#include "motion_estimation/stereo_tracker.h"
#include "motion_estimation/depth_tracker.h"

namespace proslam {

  //ds default constructor - already allocating required default objects
  SLAMAssembly::SLAMAssembly(ParameterCollection* parameters_): _parameters(parameters_),
                                                                _world_map(new WorldMap()),
                                                                _optimizer(new GraphOptimizer()),
                                                                _relocalizer(new Relocalizer()),
                                                                _tracker(0),
                                                                _camera_left(0),
                                                                _camera_right(0),
                                                                _ui_server(0),
                                                                _viewer_input_images(0),
                                                                _context_viewer_bird(0),
                                                                _context_viewer_top(0),
                                                                _is_gui_running(false) {
    _relocalizer->configure(_parameters->relocalizer_parameters);
    _world_map->configure(_parameters->world_map_parameters);
    _synchronizer.reset();
    _robot_to_world_ground_truth_poses.clear();
  }

  //ds default destructor
  SLAMAssembly::~SLAMAssembly() {
    delete _tracker;
    delete _optimizer;
    delete _relocalizer;
    delete _world_map;

    //ds free cameras
    if (_camera_left) {delete _camera_left;}
    if (_camera_right) {delete _camera_right;}

    //ds free viewers if set
    if (_viewer_input_images) {delete _viewer_input_images;}
    if (_context_viewer_bird) {delete _context_viewer_bird;}
    if (_context_viewer_top) {delete _context_viewer_top;}
    if (_ui_server) {delete _ui_server;}
    _robot_to_world_ground_truth_poses.clear();
  }

  //ds initializes txt_io playback modules
  void SLAMAssembly::initializeMessageFile() {

    //ds check dataset length
    if (_parameters->command_line_parameters->filename_dataset.length() == 0) {
      std::cerr << "ERROR: no dataset provided (enter -h for help)" << std::endl;
      exit(0);
    }

    //ds configure sensor message source
    _sensor_message_reader.open(_parameters->command_line_parameters->filename_dataset);

    //ds terminate on failure
    if (!_sensor_message_reader.good()) {
      std::cerr << _parameters->banner << std::endl;
      exit(0);
    }
  }

  void SLAMAssembly::_createStereoTracker(Camera* camera_left_, Camera* camera_right_){

    //ds if rectification is desired
    if (_parameters->command_line_parameters->option_rectify_and_undistort) {

      //ds sanity check
      if ((camera_left_->projectionMatrix().block<3,3>(0,0) - camera_right_->projectionMatrix().block<3,3>(0,0)).squaredNorm() != 0) {
        std::cerr << "SLAMAssembly::_makeStereoTracker|ERROR: provided mismatching projection matrices" << std::endl;
        exit(0);
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

  //ds attempts to load the camera configuration based on the current input setting
  void SLAMAssembly::loadCamerasFromMessageFile() {

    //ds configure message synchronizer
    std::vector<std::string> camera_topics_synchronized(0);
    camera_topics_synchronized.push_back(_parameters->command_line_parameters->topic_image_left);
    camera_topics_synchronized.push_back(_parameters->command_line_parameters->topic_image_right);
    _synchronizer.setTimeInterval(0.001);
    _synchronizer.setTopics(camera_topics_synchronized);

    //ds quickly read the first messages to buffer camera info
    srrg_core::BaseMessage* base_message = 0;
    while ((base_message = _sensor_message_reader.readMessage())) {
      srrg_core::BaseSensorMessage* sensor_msg = dynamic_cast<srrg_core::BaseSensorMessage*>(base_message);
      assert(sensor_msg != 0);
      sensor_msg->untaint();

      //ds check for the two set topics
      if (sensor_msg->topic() == _parameters->command_line_parameters->topic_image_left) {
        srrg_core::PinholeImageMessage* message_image_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_msg);

        //ds allocate a new camera
        _camera_left = new Camera(message_image_left->image().rows,
                                  message_image_left->image().cols,
                                  message_image_left->cameraMatrix().cast<real>(),
                                  message_image_left->offset().cast<real>());
      } else if (sensor_msg->topic() == _parameters->command_line_parameters->topic_image_right) {
        srrg_core::PinholeImageMessage* message_image_right  = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_msg);

        //ds allocate a new camera
        _camera_right = new Camera(message_image_right->image().rows,
                                   message_image_right->image().cols,
                                   message_image_right->cameraMatrix().cast<real>(),
                                   message_image_right->offset().cast<real>());
      }
      delete sensor_msg;

      //ds if we got all the information we need
      if (_camera_left != 0 && _camera_right != 0) {
        break;
      }
    }
    _sensor_message_reader.close();

    //ds terminate on failure
    if (_camera_left == 0) {
      std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|ERROR: left camera not set" << std::endl;
      exit(0);
    }
    if (_camera_right == 0) {
      std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|ERROR: right camera not set" << std::endl;
      exit(0);
    }
    if (_camera_left->imageCols() == 0 || _camera_left->imageRows() == 0) {
      std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|ERROR: left camera images not set" << std::endl;
      exit(0);
    }
    if (_camera_right->imageCols() == 0 || _camera_right->imageRows() == 0) {
      std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|ERROR: right camera images not set" << std::endl;
      exit(0);
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
        std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|ERROR: provided mismatching camera matrices" << std::endl;
        exit(0);
      }

      //ds compute right camera with baseline offset
      projection_matrix.block<3,1>(0,3) = _camera_left->cameraMatrix()*_camera_right->robotToCamera().translation();
      _camera_right->setProjectionMatrix(projection_matrix);
    }

    //ds load cameras to assembly
    loadCameras(_camera_left, _camera_right);
  }

  //ds attempts to load the camera configuration based on the current input setting
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

    std::cerr << "SLAMAssembly::loadCameras|loaded cameras: " << 2 << std::endl;
    std::cerr << "SLAMAssembly::loadCameras|LEFT resolution: " << camera_left_->imageCols() << " x " << camera_left_->imageRows()
              << ", aspect ratio: " << static_cast<real>(camera_left_->imageCols())/camera_left_->imageRows() << std::endl;
    std::cerr << "SLAMAssembly::loadCameras|RIGHT resolution: " << camera_right_->imageCols() << " x " << camera_right_->imageRows()
              << ", aspect ratio: " << static_cast<real>(camera_right_->imageCols())/camera_right_->imageRows() << std::endl;
  }

  //ds initializes gui components
  void SLAMAssembly::initializeGUI(QApplication* ui_server_) {
    if (_parameters->command_line_parameters->option_use_gui && _world_map) {
      _ui_server = ui_server_;
      _viewer_input_images = new ViewerInputImages(_world_map);
      _context_viewer_bird = new ViewerOutputMap(_world_map, 0.1, "output: map (bird view)");
      _context_viewer_bird->setCameraLeftToRobot(_camera_left->cameraToRobot());

      //ds orientation flip for proper camera following
      TransformMatrix3D orientation_correction(TransformMatrix3D::Identity());
      orientation_correction.matrix() << 0, -1, 0, 0,
                                         -1, 0, 0, 0,
                                         0, 0, -1, 0,
                                         0, 0, 0, 1;
      _context_viewer_bird->setRotationRobotView(orientation_correction);
      _context_viewer_bird->show();

      //ds configure custom top viewer if requested
      if (_parameters->command_line_parameters->option_show_top_viewer) {
        _context_viewer_top = new ViewerOutputMap(_world_map, 1, "output: map (top view)");
        _context_viewer_top->setCameraLeftToRobot(_camera_left->cameraToRobot());
        TransformMatrix3D center_for_kitti_sequence_00;
        center_for_kitti_sequence_00.matrix() << 1, 0, 0, 0,
                                                 0, 0, -1, 200,
                                                 0, 1, 0, 800,
                                                 0, 0, 0, 1;
        _context_viewer_top->setWorldToRobotOrigin(center_for_kitti_sequence_00);
        _context_viewer_top->setFollowRobot(false);
        _context_viewer_top->setWorldToRobotOrigin(orientation_correction*center_for_kitti_sequence_00);
        _context_viewer_top->show();
      }

      _viewer_input_images->setTracker(_tracker);
      _is_gui_running = true;
    }
  }

  //ds updated gui components
  bool SLAMAssembly::updateGUI() {
    if (_parameters->command_line_parameters->option_use_gui) {
      if (_optimizer->numberOfOptimizations() > 0) {
        _context_viewer_bird->setIsOpen(false);
        if (_context_viewer_top) {
          _context_viewer_top->setIsOpen(false);
        }
      }
      _viewer_input_images->initDrawing();
      _viewer_input_images->drawFeatureTracking();
      _viewer_input_images->drawFeatures();
      _is_gui_running = _context_viewer_bird->isVisible() && _viewer_input_images->updateGUI();
      _context_viewer_bird->updateGL();
      if (_context_viewer_top) {
        _context_viewer_top->updateGL();
      }
      _ui_server->processEvents();
      return _is_gui_running;
    } else {
      return true;
    }
  }

  //ds clean gui components
  int32_t SLAMAssembly::closeGUI(const bool& let_user_close_) {

    //ds if no manual termination was requested
    if (_is_gui_running && let_user_close_) {

      //ds exit in viewer if available
      if (_context_viewer_bird->isVisible()) {
        return _ui_server->exec();
      } else {
        return 0;
      }
    } else {
      return 0;
    }
  }

  //ds playback txt_io message file
  void SLAMAssembly::playbackMessageFile() {

    //ds restart stream
    _sensor_message_reader.open(_parameters->command_line_parameters->filename_dataset);
    _robot_to_world_ground_truth_poses.clear();

    //ds frame counts
    Count number_of_processed_frames_total   = 0;
    Count number_of_processed_frames_current = 0;

    //ds store start time
    double time_start_seconds             = srrg_core::getTime();
    const double time_start_seconds_first = srrg_core::getTime();

    //ds visualization/start point
    const TransformMatrix3D robot_to_camera_left(_camera_left->robotToCamera());

    //ds start playback
    srrg_core::BaseMessage* base_message = 0;
    while ((base_message = _sensor_message_reader.readMessage())) {
      srrg_core::BaseSensorMessage* sensor_msg = dynamic_cast<srrg_core::BaseSensorMessage*>(base_message);
      assert(sensor_msg != 0);
      sensor_msg->untaint();

      //ds add to synchronizer
      if (sensor_msg->topic() == _parameters->command_line_parameters->topic_image_left) {
        _synchronizer.putMessage(sensor_msg);
      } else if (sensor_msg->topic() == _parameters->command_line_parameters->topic_image_right) {
        _synchronizer.putMessage(sensor_msg);
      } else {
        delete sensor_msg;
      }

      //ds if we have a synchronized package of sensor messages ready
      if (_synchronizer.messagesReady()) {

        //ds buffer sensor data
        srrg_core::PinholeImageMessage* message_image_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(_synchronizer.messages()[0].get());
        srrg_core::PinholeImageMessage* message_image_right = dynamic_cast<srrg_core::PinholeImageMessage*>(_synchronizer.messages()[1].get());

        //ds buffer images
        cv::Mat intensity_image_left_rectified;
        if(message_image_left->image().type() == CV_8UC3){
          cvtColor(message_image_left->image(), intensity_image_left_rectified, CV_BGR2GRAY);
        } else {
          intensity_image_left_rectified = message_image_left->image();
        }
        cv::Mat intensity_image_right_rectified;
        if (_parameters->command_line_parameters->tracker_mode == CommandLineParameters::TrackerMode::RGB_STEREO && message_image_right->image().type() == CV_8UC3) {
          cvtColor(message_image_right->image(), intensity_image_right_rectified, CV_BGR2GRAY);
        } else {
          intensity_image_right_rectified = message_image_right->image();
        }

        //ds preprocess the images if desired
        if (_parameters->command_line_parameters->option_equalize_histogram) {
          cv::equalizeHist(intensity_image_left_rectified, intensity_image_left_rectified);
          if (_parameters->command_line_parameters->tracker_mode == CommandLineParameters::TrackerMode::RGB_STEREO) {
            cv::equalizeHist(intensity_image_right_rectified, intensity_image_right_rectified);
          }
        }

        //ds check if first frame and odometry is available
        if (_world_map->frames().size() == 0 && message_image_left->hasOdom()) {
          _world_map->setRobotToWorld(message_image_left->odometry().cast<real>()*robot_to_camera_left);
          if (_parameters->command_line_parameters->option_use_gui) {
            _context_viewer_bird->setWorldToRobotOrigin(_world_map->robotToWorld().inverse());
          }
        }

//        //ds shift images, correcting invalid rectification
//        translate(intensity_image_left_rectified, 0, -1);

        //ds progress SLAM with the new images
        process(intensity_image_left_rectified,
                intensity_image_right_rectified,
                message_image_left->hasOdom() && _parameters->command_line_parameters->option_use_odometry,
                message_image_left->odometry().cast<real>());

        //ds record ground truth history for error computation
        if (message_image_left->hasOdom()) {
          addGroundTruthMeasurement(message_image_left->odometry().cast<real>()*robot_to_camera_left);
        }

        //ds runtime info
        ++number_of_processed_frames_total;
        ++number_of_processed_frames_current;
        if (number_of_processed_frames_current%100 == 0) {

          //ds compute durations
          const double total_duration_seconds_current = srrg_core::getTime()-time_start_seconds;

          //ds runtime info - depending on set modes
          if (_parameters->command_line_parameters->option_use_relocalization) {
            std::printf("processed frames: %5lu|landmarks: %6lu|local maps: %4lu (%3.2f)|closures: %3lu (%3.2f)|current fps: %5.2f (%3lu/%3.2fs)\n",
                        number_of_processed_frames_total,
                        _world_map->landmarks().size(),
                        _world_map->localMaps().size(),
                        _world_map->localMaps().size()/static_cast<real>(number_of_processed_frames_total),
                        _world_map->numberOfClosures(),
                        _world_map->numberOfClosures()/static_cast<real>(_world_map->localMaps().size()),
                        number_of_processed_frames_current/total_duration_seconds_current,
                        number_of_processed_frames_current,
                        total_duration_seconds_current);
          } else {
            std::printf("processed frames: %5lu|landmarks: %6lu|current fps: %5.2f (%3lu/%3.2fs)\n",
                        number_of_processed_frames_total,
                        _world_map->landmarksInWindowForLocalMap().size(),
                        number_of_processed_frames_current/total_duration_seconds_current,
                        number_of_processed_frames_current,
                        total_duration_seconds_current);
          }

          //ds reset stats
          time_start_seconds = srrg_core::getTime();
          number_of_processed_frames_current = 0;
        }
        _synchronizer.reset();

        //ds update gui and check if termination is requested
        if (!updateGUI() ){
          break;
        }
      }
    }
    _duration_total_seconds = srrg_core::getTime()-time_start_seconds_first;
    _sensor_message_reader.close();
  }

  //ds sets ground truth to current frame
  void SLAMAssembly::addGroundTruthMeasurement(const TransformMatrix3D& robot_to_world_ground_truth_) {
    _robot_to_world_ground_truth_poses.push_back(robot_to_world_ground_truth_);
    _world_map->setRobotToWorldGroundTruth(robot_to_world_ground_truth_);
  }

  //ds process a pair of rectified and undistorted stereo images
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
                                            closure->transform_frame_query_to_frame_reference,
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

          //ds optimize graph
          _optimizer->optimize(_world_map);
        }
      }
    } else {

      //ds open loop options: save memory
      if (_parameters->command_line_parameters->option_drop_framepoints) {

        //ds free framepoints if available
        if (_world_map->currentFrame()->previous() && _world_map->currentFrame()->previous()->previous()) {
          _world_map->currentFrame()->previous()->previous()->clear();
        }
      }
    }
  }

  //ds prints extensive run summary
  void SLAMAssembly::printReport() {

    //ds header
    const Count number_of_processed_frames_total = _world_map->frames().size();
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
    std::cerr << "dataset completed" << std::endl;
    std::cerr << "-------------------------------------------------------------------------" << std::endl;

    //ds if nothing was processed - exit right away
    if (number_of_processed_frames_total == 0) {
      std::cerr << "no frames processed" << std::endl;
      std::cerr << "-------------------------------------------------------------------------" << std::endl;
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
    }
    
    std::cerr << "                     total frames: " << number_of_processed_frames_total << std::endl;
    std::cerr << "               total duration (s): " << _duration_total_seconds << std::endl;
    std::cerr << "                      average fps: " << number_of_processed_frames_total/_duration_total_seconds << std::endl;
    std::cerr << "average processing time (s/frame): " << _duration_total_seconds/number_of_processed_frames_total << std::endl;
    std::cerr << "average landmarks close per frame: " << _tracker->totalNumberOfLandmarksClose()/number_of_processed_frames_total << std::endl;
    std::cerr << "  average landmarks far per frame: " << _tracker->totalNumberOfLandmarksFar()/number_of_processed_frames_total << std::endl;
    std::cerr << "         average tracks per frame: " << _tracker->totalNumberOfTrackedPoints()/number_of_processed_frames_total << std::endl;
    std::cerr << "        average tracks per second: " << _tracker->totalNumberOfTrackedPoints()/_duration_total_seconds << std::endl;
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
    std::cerr << "runtime" << std::endl;
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
    std::cerr << "       feature detection: " << _tracker->framepointGenerator()->getTimeConsumptionSeconds_feature_detection()/_duration_total_seconds
                                             << " (" << _tracker->framepointGenerator()->getTimeConsumptionSeconds_feature_detection() << "s)" << std::endl;
    std::cerr << "   descriptor extraction: " << _tracker->framepointGenerator()->getTimeConsumptionSeconds_descriptor_extraction()/_duration_total_seconds
                                             << " (" << _tracker->framepointGenerator()->getTimeConsumptionSeconds_descriptor_extraction() << "s)" << std::endl;

    //ds display further information depending on tracking mode
    switch (_parameters->command_line_parameters->tracker_mode){
      case CommandLineParameters::TrackerMode::RGB_STEREO: {
        StereoFramePointGenerator* stereo_framepoint_generator = dynamic_cast<StereoFramePointGenerator*>(_tracker->framepointGenerator());
        std::cerr << "  stereo keypoint search: " << stereo_framepoint_generator->getTimeConsumptionSeconds_point_triangulation()/_duration_total_seconds
                  << " (" << stereo_framepoint_generator->getTimeConsumptionSeconds_point_triangulation() << "s)" << std::endl;
        break;
      }
      case CommandLineParameters::TrackerMode::RGB_DEPTH: {
        break;
      }
      default: {
        break;
      }
    }

    std::cerr << "                tracking: " << _tracker->getTimeConsumptionSeconds_tracking()/_duration_total_seconds
                                             << " (" << _tracker->getTimeConsumptionSeconds_tracking() << "s)" << std::endl;
    std::cerr << "       pose optimization: " << _tracker->getTimeConsumptionSeconds_pose_optimization()/_duration_total_seconds
                                             << " (" << _tracker->getTimeConsumptionSeconds_pose_optimization() << "s)" << std::endl;
    std::cerr << "   landmark optimization: " << _tracker->getTimeConsumptionSeconds_landmark_optimization()/_duration_total_seconds
                                             << " (" << _tracker->getTimeConsumptionSeconds_landmark_optimization() << "s)" << std::endl;
    std::cerr << " correspondence recovery: " << _tracker->getTimeConsumptionSeconds_point_recovery()/_duration_total_seconds
                                             << " (" << _tracker->getTimeConsumptionSeconds_point_recovery() << "s)" << std::endl;
    std::cerr << "similarity search (HBST): " << _relocalizer->getTimeConsumptionSeconds_overall()/_duration_total_seconds
                                             << " (" << _relocalizer->getTimeConsumptionSeconds_overall() << "s)" << std::endl;
    std::cerr << "              map update: " << _optimizer->getTimeConsumptionSeconds_overall()/_duration_total_seconds
                                             << " (" << _optimizer->getTimeConsumptionSeconds_overall() << "s)" << std::endl;
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
  }

  void SLAMAssembly::translate(cv::Mat &image_, const int32_t& offsetx_, const int32_t& offsety_){
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx_, 0, 1, offsety_);
    warpAffine(image_, image_, trans_mat,image_.size());
  }

  void SLAMAssembly::reset() {
    _world_map->clear();
  }
}
