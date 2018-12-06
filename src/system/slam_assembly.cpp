#include "slam_assembly.h"

#include "srrg_messages/pinhole_image_message.h"
#include "position_tracking/depth_tracker.h"
#include "position_tracking/stereo_tracker.h"
#include "aligners/stereouv_aligner.h"
#include "aligners/uvd_aligner.h"

namespace proslam {

SLAMAssembly::SLAMAssembly(ParameterCollection* parameters_): _parameters(parameters_),
                                                              _world_map(new WorldMap(_parameters->world_map_parameters)),
                                                              _graph_optimizer(new GraphOptimizer(_parameters->graph_optimizer_parameters)),
                                                              _relocalizer(new Relocalizer(_parameters->relocalizer_parameters)),
                                                              _tracker(0),
                                                              _camera_left(0),
                                                              _camera_right(0),
                                                              _ui_server(0),
                                                              _image_viewer(0),
                                                              _map_viewer(0),
                                                              _minimap_viewer(0),
//                                                              _new_image_available(false),
                                                              _is_termination_requested(false),
                                                              _is_viewer_open(false) {
  _synchronizer.reset();
  _processing_times_seconds.clear();

  //ds reset all static object counters
  Frame::reset();
  FramePoint::reset();
  LocalMap::reset();
  Landmark::reset();
  LOG_INFO(std::cerr << "SLAMAssembly::SLAMAssembly|constructed" << std::endl)
}

SLAMAssembly::~SLAMAssembly() {
  LOG_INFO(std::cerr << "SLAMAssembly::~SLAMAssembly|destroying assembly" << std::endl)
  delete _tracker;
  delete _graph_optimizer;
  delete _relocalizer;
  delete _world_map;
  delete _camera_left;
  delete _camera_right;
  _message_reader.close();
  _synchronizer.reset();
  LOG_INFO(std::cerr << "SLAMAssembly::~SLAMAssembly|destroyed" << std::endl)
}

void SLAMAssembly::_createStereoTracker(Camera* camera_left_, Camera* camera_right_){

  //ds sanity check
  if ((camera_left_->projectionMatrix().block<3,3>(0,0) - camera_right_->projectionMatrix().block<3,3>(0,0)).squaredNorm() != 0) {
    LOG_ERROR(std::cerr << "SLAMAssembly::_createStereoTracker|provided mismatching projection matrices" << std::endl)
    throw std::runtime_error("mismatching projection matrices");
  }

  //ds replace camera matrices with identical one
  camera_left_->setCameraMatrix(camera_left_->projectionMatrix().block<3,3>(0,0));
  camera_right_->setCameraMatrix(camera_left_->cameraMatrix());

  //ds allocate and configure the framepoint generator
  StereoFramePointGenerator* framepoint_generator = new StereoFramePointGenerator(_parameters->stereo_framepoint_generator_parameters);
  framepoint_generator->setCameraLeft(camera_left_);
  framepoint_generator->setCameraRight(camera_right_);
  framepoint_generator->configure();

  //ds allocate and configure the aligner for motion estimation
  StereoUVAligner* pose_optimizer = new StereoUVAligner(_parameters->stereo_tracker_parameters->aligner);
  pose_optimizer->setMaximumReliableDepthMeters(_parameters->stereo_framepoint_generator_parameters->maximum_reliable_depth_meters);
  pose_optimizer->configure();

  //ds allocate and configure the tracker
  StereoTracker* tracker = new StereoTracker(_parameters->stereo_tracker_parameters);
  tracker->setCameraLeft(camera_left_);
  tracker->setCameraRight(camera_right_);
  tracker->setFramePointGenerator(framepoint_generator);
  tracker->setAligner(pose_optimizer);
  tracker->configure();
  _tracker = tracker;
  _tracker->setWorldMap(_world_map);

  //ds configure components
  _graph_optimizer->configure();
  _relocalizer->configure();
}

void SLAMAssembly::_createDepthTracker(Camera* camera_left_, Camera* camera_right_){

  //ds allocate and configure the framepoint generator
  _camera_left = camera_left_;
  _camera_right = camera_right_;
  DepthFramePointGenerator* framepoint_generator = new DepthFramePointGenerator(_parameters->depth_framepoint_generator_parameters);
  framepoint_generator->setCameraLeft(camera_left_);
  framepoint_generator->setCameraRight(camera_right_);
  framepoint_generator->configure();

  //ds allocate and configure the aligner for motion estimation
  UVDAligner* pose_optimizer = new UVDAligner(_parameters->depth_tracker_parameters->aligner);
  pose_optimizer->configure();

  //ds allocate and configure the tracker
  DepthTracker* tracker = new DepthTracker(_parameters->depth_tracker_parameters);
  tracker->setCameraLeft(camera_left_);
  tracker->setDepthCamera(camera_right_);
  tracker->setFramePointGenerator(framepoint_generator);
  tracker->setAligner(pose_optimizer);
  tracker->configure();
  _tracker = tracker;
  _tracker->setWorldMap(_world_map);

  //ds configure components
  _graph_optimizer->configure();
  _relocalizer->configure();
}

void SLAMAssembly::loadCamerasFromMessageFile() {

  //ds configure sensor message source
  _message_reader.open(_parameters->command_line_parameters->dataset_file_name);

  //ds terminate on failure
  if (!_message_reader.good()) {
    LOG_INFO(std::cerr << _parameters->banner << std::endl)
    throw std::runtime_error("unable to open dataset");
  }

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
    if (sensor_message) {

      //ds check for the two set topics to set the camera objects
      if (_camera_left == 0 && sensor_message->topic().compare(_parameters->command_line_parameters->topic_image_left) == 0) {
        srrg_core::PinholeImageMessage* message_image_left = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_message);

        //ds allocate a new camera
        _camera_left = new Camera(message_image_left->image().rows,
                                  message_image_left->image().cols,
                                  message_image_left->cameraMatrix().cast<real>(),
                                  message_image_left->offset().cast<real>());
      } else if (_camera_right == 0 && sensor_message->topic().compare(_parameters->command_line_parameters->topic_image_right) == 0) {
        srrg_core::PinholeImageMessage* message_image_right = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_message);

        //ds allocate a new camera
        _camera_right = new Camera(message_image_right->image().rows,
                                   message_image_right->image().cols,
                                   message_image_right->cameraMatrix().cast<real>(),
                                   message_image_right->offset().cast<real>());
      }
    }
    message->untaint();
    delete message;

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
  if (_camera_left->numberOfImageCols() == 0 || _camera_left->numberOfImageRows() == 0) {
    LOG_ERROR(std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|left camera images not set" << std::endl)
    throw std::runtime_error("left camera images not set");
  }
  if (_camera_right->numberOfImageCols() == 0 || _camera_right->numberOfImageRows() == 0) {
    LOG_ERROR(std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|right camera images not set" << std::endl)
    throw std::runtime_error("right camera images not set");
  }

  //ds check if we have to modify the cameras to compute the projection matrices - if stereo from txt_io
  if (_parameters->command_line_parameters->tracker_mode == CommandLineParameters::TrackerMode::RGB_STEREO) {

    //ds reconstruct projection matrix from camera matrices (encoded in txt_io)
    ProjectionMatrix projection_matrix(ProjectionMatrix::Identity());
    projection_matrix.block<3,3>(0,0) = _camera_left->cameraMatrix()*_camera_left->robotToCamera().linear();

    //ds set left
    _camera_left->setProjectionMatrix(projection_matrix);
    LOG_DEBUG(std::cerr << "projection matrix LEFT: " << std::endl;)
    LOG_DEBUG(std::printf("%11.6f %11.6f %11.6f %11.6f\n", projection_matrix(0,0), projection_matrix(0,1), projection_matrix(0,2), projection_matrix(0,3)))
    LOG_DEBUG(std::printf("%11.6f %11.6f %11.6f %11.6f\n", projection_matrix(1,0), projection_matrix(1,1), projection_matrix(1,2), projection_matrix(1,3)))
    LOG_DEBUG(std::printf("%11.6f %11.6f %11.6f %11.6f\n", projection_matrix(2,0), projection_matrix(2,1), projection_matrix(2,2), projection_matrix(2,3)))

    //ds sanity check
    if ((_camera_left->cameraMatrix()-_camera_right->cameraMatrix()).squaredNorm() != 0) {
      LOG_ERROR(std::cerr << "SLAMAssembly::loadCamerasFromMessageFile|provided mismatching camera matrices" << std::endl)
      throw std::runtime_error("provided mismatching camera matrices");
    }

    //ds compute right camera with baseline offset
    projection_matrix.block<3,1>(0,3) = _camera_left->cameraMatrix()*_camera_right->robotToCamera().translation();
    _camera_right->setProjectionMatrix(projection_matrix);
    _camera_right->setBaselineHomogeneous(projection_matrix.col(3));
    LOG_DEBUG(std::cerr << "projection matrix RIGHT: " << std::endl;)
    LOG_DEBUG(std::printf("%11.6f %11.6f %11.6f %11.6f\n", projection_matrix(0,0), projection_matrix(0,1), projection_matrix(0,2), projection_matrix(0,3) ))
    LOG_DEBUG(std::printf("%11.6f %11.6f %11.6f %11.6f\n", projection_matrix(1,0), projection_matrix(1,1), projection_matrix(1,2), projection_matrix(1,3)))
    LOG_DEBUG(std::printf("%11.6f %11.6f %11.6f %11.6f\n", projection_matrix(2,0), projection_matrix(2,1), projection_matrix(2,2), projection_matrix(2,3)))
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

  //ds set system handles
  _camera_left  = camera_left_;
  _camera_right = camera_right_;

  LOG_INFO(std::cerr << "SLAMAssembly::loadCameras|loaded cameras: " << 2 << std::endl)
  LOG_INFO(std::cerr << "SLAMAssembly::loadCameras|LEFT resolution: " << camera_left_->numberOfImageCols() << " x " << camera_left_->numberOfImageRows()
            << ", aspect ratio: " << static_cast<real>(camera_left_->numberOfImageCols())/camera_left_->numberOfImageRows() << std::endl)
  LOG_INFO(std::cerr << "SLAMAssembly::loadCameras|RIGHT resolution: " << camera_right_->numberOfImageCols() << " x " << camera_right_->numberOfImageRows()
            << ", aspect ratio: " << static_cast<real>(camera_right_->numberOfImageCols())/camera_right_->numberOfImageRows() << std::endl)
}

void SLAMAssembly::initializeGUI(std::shared_ptr<QApplication> ui_server_) {
  if (_parameters->command_line_parameters->option_use_gui) {
    _ui_server = ui_server_;

    //ds allocate internal viewers (with new to keep Eigen's memory sane)
    _image_viewer = std::shared_ptr<ImageViewer>(new ImageViewer(_parameters->image_viewer_parameters));
    _map_viewer   = std::shared_ptr<MapViewer>(new MapViewer(_parameters->map_viewer_parameters));
    _map_viewer->setCameraLeftToRobot(_camera_left->cameraToRobot());

    //ds orientation flip for proper camera following
    TransformMatrix3D orientation_correction(TransformMatrix3D::Identity());
    orientation_correction.matrix() << 1, 0, 0, 0,
                                       0, -1, 0, 0,
                                       0, 0, -1, 0,
                                       0, 0, 0, 1;
    _map_viewer->setRotationRobotView(orientation_correction);
    _map_viewer->setWorldMap(_world_map);
    _map_viewer->show();

    //ds configure custom top viewer if requested
    if (_parameters->command_line_parameters->option_show_top_viewer) {
      _parameters->top_map_viewer_parameters->object_scale = 1;
      _parameters->top_map_viewer_parameters->window_title = "minimap [OpenGL]";
      _minimap_viewer = std::shared_ptr<MapViewer>(new MapViewer(_parameters->top_map_viewer_parameters));
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
//    _new_image_available = true;
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

//    //ds save images to disk
//    if (_new_image_available) {
//
//      //ds save images to disk
//      _map_viewer->setSnapshotFileName("images/map.jpg");
//      _map_viewer->saveSnapshot();
//      if (_minimap_viewer) {
//        _minimap_viewer->setSnapshotFileName("images/minimap.jpg");
//        _minimap_viewer->saveSnapshot();
//      }
//      _image_viewer->saveToDisk();
//      _new_image_available = false;
//    }
  }
}

void SLAMAssembly::writePoseGraphToFile(const std::string& file_name_) const {
  if (_graph_optimizer && _world_map) {
    _graph_optimizer->writePoseGraphToFile(_world_map, file_name_);
  }
}

void SLAMAssembly::playbackMessageFile() {

  //ds restart stream
  _message_reader.open(_parameters->command_line_parameters->dataset_file_name);

  //ds frame counts
  _number_of_processed_frames = 0;
  Count number_of_processed_frames_current = 0;

  //ds time measurement
  const double runtime_info_update_frequency_seconds = 5;
  double processing_time_seconds_current             = 0;

  //ds visualization/start point
  const TransformMatrix3D robot_to_camera_left(_camera_left->robotToCamera());

  //ds start playback
  srrg_core::BaseMessage* message = 0;
  while ((message = _message_reader.readMessage())) {
    srrg_core::BaseSensorMessage* sensor_message = dynamic_cast<srrg_core::BaseSensorMessage*>(message);
    assert(sensor_message);
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

      //ds start measuring time
      const double time_start_seconds = srrg_core::getTime();

      //ds progress SLAM with the new images
      process(intensity_image_left_rectified,
              intensity_image_right_rectified,
              image_message_left->timestamp(),
              image_message_left->hasOdom() && _parameters->command_line_parameters->option_use_odometry,
              image_message_left->odometry().cast<real>());

      //ds update timing stats
      const double processing_time_seconds = srrg_core::getTime()-time_start_seconds;
      _processing_times_seconds.push_back(processing_time_seconds);
      _processing_time_total_seconds  += processing_time_seconds;
      processing_time_seconds_current += processing_time_seconds;
      ++_number_of_processed_frames;
      ++number_of_processed_frames_current;
      _current_fps = _number_of_processed_frames/_processing_time_total_seconds;

      //ds record ground truth history for error computation
      if (image_message_left->hasOdom()) {
        _world_map->setRobotToWorldGroundTruth(image_message_left->odometry().cast<real>()*robot_to_camera_left);
      }

      //ds runtime info
      if (processing_time_seconds_current > runtime_info_update_frequency_seconds) {

        //ds runtime info - depending on set modes and available information
        if (!_parameters->command_line_parameters->option_disable_relocalization && !_world_map->localMaps().empty()) {
          LOG_INFO(std::printf("SLAMAssembly::playbackMessageFile|frames: %5lu <FPS: %6.2f>|landmarks: %6lu|local maps: %4lu (%3.2f)|closures: %3lu (%3.2f)\n",
                      _number_of_processed_frames,
                      number_of_processed_frames_current/processing_time_seconds_current,
                      _world_map->landmarks().size(),
                      _world_map->localMaps().size(),
                      _world_map->localMaps().size()/static_cast<real>(_number_of_processed_frames),
                      _world_map->numberOfClosures(),
                      _world_map->numberOfClosures()/static_cast<real>(_world_map->localMaps().size())))
        } else {
          LOG_INFO(std::printf("SLAMAssembly::playbackMessageFile|frames: %5lu <FPS: %6.2f>|landmarks: %6lu|map updates: %3lu\n",
                      _number_of_processed_frames,
                      number_of_processed_frames_current/processing_time_seconds_current,
                      _world_map->landmarks().size(),
                      _graph_optimizer->numberOfOptimizations()))
        }

        //ds reset stats for new measurement window
        processing_time_seconds_current    = 0;
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
  _message_reader.close();
  LOG_INFO(std::cerr << "SLAMAssembly::playbackMessageFile|dataset completed" << std::endl)
}

void SLAMAssembly::process(const cv::Mat& intensity_image_left_,
                           const cv::Mat& intensity_image_right_,
                           const double& timestamp_image_left_seconds_,
                           const bool& use_odometry_,
                           const TransformMatrix3D& odometry_) {

  //ds call the tracker
  _tracker->setIntensityImageLeft(intensity_image_left_);

  //ds depending on tracking mode
  switch (_parameters->command_line_parameters->tracker_mode){
    case CommandLineParameters::TrackerMode::RGB_STEREO: {
      StereoTracker* stereo_tracker = dynamic_cast<StereoTracker*>(_tracker);
      assert(stereo_tracker);
      stereo_tracker->setIntensityImageRight(intensity_image_right_);
      break;
    }
    case CommandLineParameters::TrackerMode::RGB_DEPTH: {
      DepthTracker* depth_tracker = dynamic_cast<DepthTracker*>(_tracker);
      assert(depth_tracker);
      depth_tracker->setDepthImage(intensity_image_right_);
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

  //ds if we generated a valid frame
  if (_world_map->currentFrame()) {

    //ds set additional fields
    _world_map->currentFrame()->setTimestampImageLeftSeconds(timestamp_image_left_seconds_);

    //ds if relocalization is not disabled
    if (!_parameters->command_line_parameters->option_disable_relocalization) {

      //ds local map generation - regardless of tracker state
      if (_map_viewer) {_map_viewer->lock();}
      const bool created_local_map = _world_map->createLocalMap(_parameters->command_line_parameters->option_drop_framepoints);
      if (_map_viewer) {_map_viewer->unlock();}

      //ds if we successfully created a local map
      if (created_local_map) {

        //ds localize in database (not yet optimizing the graph)
        _relocalizer->detectClosures(_world_map->currentLocalMap());
        _relocalizer->registerClosures();
//        _relocalizer->prune();

        //ds check the closures
        for(Closure* closure: _relocalizer->closures()) {
          if (closure->is_valid) {
            assert(_world_map->currentLocalMap() == closure->local_map_query);

            //ds add loop closure constraint (merging corresponding landmarks)
            _world_map->addLoopClosure(_world_map->currentLocalMap(),
                                       closure->local_map_reference,
                                       closure->query_to_reference,
                                       closure->correspondences,
                                       closure->icp_inlier_ratio);
            if (_parameters->command_line_parameters->option_use_gui) {
              for (const Closure::Correspondence* match: closure->correspondences) {
                _world_map->landmarks().at(match->query->identifier())->setIsInLoopClosureQuery(true);
                _world_map->landmarks().at(match->reference->identifier())->setIsInLoopClosureReference(true);
              }
            }
          }
        }

        //ds clear buffer (automatically purges invalidated closures)
        _relocalizer->clear();
      }

      //ds if bundle-adjustment is desired
      if (!_parameters->command_line_parameters->option_disable_bundle_adjustment) {

        //ds add frame and its landmarks to the pose graph
        _graph_optimizer->addFrameWithLandmarks(_world_map->currentFrame());

        //ds check if a periodic bundle adjustment is required
        if (_world_map->frames().size() % _parameters->graph_optimizer_parameters->number_of_frames_per_bundle_adjustment == 0) {

          //ds check if we're running with a GUI and lock the GUI before the critical phase
          if (_map_viewer) {_map_viewer->lock();}

          //ds optimize graph
          _graph_optimizer->optimizeFramesWithLandmarks(_world_map);

          //ds reenable the GUI
          if (_map_viewer) {_map_viewer->unlock();}
        }
      } else {

        //ds just add the frame to the pose graph
        _graph_optimizer->addFrame(_world_map->currentFrame());

        //ds if we closed a local map - otherwise there is no need to optimize the pose graph
        if (_world_map->relocalized()) {

          //ds check if we're running with a GUI and lock the GUI before the critical phase
          if (_map_viewer) {_map_viewer->lock();}

          //ds optimize pose graph with the loop closure constraint
          _graph_optimizer->optimizeFrames(_world_map);

          //ds merge landmarks for the current local map and its closures
          _world_map->mergeLandmarks(_world_map->currentLocalMap()->closures());

          //ds re-enable the GUI
          if (_map_viewer) {_map_viewer->unlock();}
        }
      }
    } else if (_parameters->command_line_parameters->option_drop_framepoints) {

      //ds free disconnected framepoints if available: TODO safe window
      if (_world_map->frames().size() >= 500) {
        _world_map->frames().at(_world_map->frames().size()-500)->clear();
      }
    }
  }
}

void SLAMAssembly::printReport() const {

  //ds header
  std::cerr << DOUBLE_BAR << std::endl;
  std::cerr << "performance summary" << std::endl;
  std::cerr << BAR << std::endl;

  //ds if not at least 2 frames were processed - exit right away
  if (_number_of_processed_frames <= 1) {
    std::cerr << "no frames processed" << std::endl;
    std::cerr << DOUBLE_BAR << std::endl;
    return;
  }

  //ds compute trajectory length
  double trajectory_length = 0;
  for (FramePointerMapElement frame: _world_map->frames()) {
    if (frame.second->previous()) {
      trajectory_length += (frame.second->worldToRobot()*frame.second->previous()->robotToWorld()).translation().norm();
    }
  }

  //ds compute mean processing time and standard deviation
  const double processing_time_mean_seconds = _processing_time_total_seconds/_number_of_processed_frames;
  double processing_time_standard_deviation_seconds = 0;
  for (const double& processing_time_seconds: _processing_times_seconds) {
    processing_time_standard_deviation_seconds += (processing_time_mean_seconds-processing_time_seconds)
                                                 *(processing_time_mean_seconds-processing_time_seconds);
  }
  processing_time_standard_deviation_seconds /= _processing_times_seconds.size();
  processing_time_standard_deviation_seconds = std::sqrt(processing_time_standard_deviation_seconds);

  //ds general stats
  std::cerr << "        total trajectory length (m): " << trajectory_length << std::endl;
  std::cerr << "                       total frames: " << _number_of_processed_frames << std::endl;
  std::cerr << "      total processing duration (s): " << _processing_time_total_seconds << std::endl;
  std::cerr << "                        average FPS: " << _current_fps << std::endl;
  std::cerr << "            average velocity (km/h): " << 3.6*trajectory_length/_processing_time_total_seconds << std::endl;
  std::cerr << "     mean processing time (s/frame): " << processing_time_mean_seconds
            << " (standard deviation: " << processing_time_standard_deviation_seconds << ")" << std::endl;
  std::cerr << "           mean number of keypoints: " << _tracker->meanNumberOfKeypoints() << std::endl;
  std::cerr << "         mean number of framepoints: " << _tracker->meanNumberOfFramepoints() << std::endl;
  std::cerr << "           mean landmarks per frame: " << _tracker->totalNumberOfLandmarks()/_number_of_processed_frames << std::endl;
  std::cerr << "              mean tracks per frame: " << _tracker->totalNumberOfTrackedPoints()/_number_of_processed_frames << std::endl;
  std::cerr << "             mean tracks per second: " << _tracker->totalNumberOfTrackedPoints()/_processing_time_total_seconds << std::endl;
  std::cerr << "            number of loop closures: " << _world_map->numberOfClosures() << std::endl;
  std::cerr << "         number of merged landmarks: " << _world_map->numberOfMergedLandmarks()
            << " (of total landmarks: " << static_cast<real>(_world_map->numberOfMergedLandmarks())/_world_map->landmarks().size() <<  ")" << std::endl;
  std::cerr << "  number of recursive registrations: " << _tracker->numberOfRecursiveRegistrations() << std::endl;

  //ds display further information depending on tracking mode
  switch (_parameters->command_line_parameters->tracker_mode){
    case CommandLineParameters::TrackerMode::RGB_STEREO: {
      StereoFramePointGenerator* stereo_framepoint_generator = dynamic_cast<StereoFramePointGenerator*>(_tracker->framepointGenerator());
      std::cerr << "average triangulation success ratio: " << stereo_framepoint_generator->meanTriangulationSuccessRatio() << std::endl;
      break;
    }
    case CommandLineParameters::TrackerMode::RGB_DEPTH: {
      break;
    }
    default: {
      break;
    }
  }
  std::cerr << BAR << std::endl;

  //ds computational costs
  std::cerr << std::endl;
  std::cerr << "time consumption overview - processing units" << std::endl;
  std::cerr << BAR << std::endl;
  std::cerr << "            module name | relative | absolute (s)" << std::endl;
  std::cerr << BAR << std::endl;
  std::printf("     keypoint detection | %f | %f\n", _tracker->framepointGenerator()->getTimeConsumptionSeconds_keypoint_detection()/_processing_time_total_seconds,
                                                         _tracker->framepointGenerator()->getTimeConsumptionSeconds_keypoint_detection());
  std::printf("  descriptor extraction | %f | %f\n", _tracker->framepointGenerator()->getTimeConsumptionSeconds_descriptor_extraction()/_processing_time_total_seconds,
                                                         _tracker->framepointGenerator()->getTimeConsumptionSeconds_descriptor_extraction());

  //ds display further information depending on tracking mode
  switch (_parameters->command_line_parameters->tracker_mode){
    case CommandLineParameters::TrackerMode::RGB_STEREO: {
      StereoFramePointGenerator* stereo_framepoint_generator = dynamic_cast<StereoFramePointGenerator*>(_tracker->framepointGenerator());
      std::printf(" stereo keypoint search | %f | %f\n", stereo_framepoint_generator->getTimeConsumptionSeconds_point_triangulation()/_processing_time_total_seconds,
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

  std::printf("               tracking | %f | %f\n", _tracker->getTimeConsumptionSeconds_tracking()/_processing_time_total_seconds, _tracker->getTimeConsumptionSeconds_tracking());
  std::printf("      pose optimization | %f | %f\n", _tracker->getTimeConsumptionSeconds_pose_optimization()/_processing_time_total_seconds, _tracker->getTimeConsumptionSeconds_pose_optimization());
  std::printf("  landmark optimization | %f | %f\n", _tracker->getTimeConsumptionSeconds_landmark_optimization()/_processing_time_total_seconds, _tracker->getTimeConsumptionSeconds_landmark_optimization());
  std::printf("         point recovery | %f | %f\n", _tracker->getTimeConsumptionSeconds_point_recovery()/_processing_time_total_seconds, _tracker->getTimeConsumptionSeconds_point_recovery());
  std::printf("         relocalization | %f | %f\n", _relocalizer->getTimeConsumptionSeconds_overall()/_processing_time_total_seconds, _relocalizer->getTimeConsumptionSeconds_overall());
  std::printf("    pose graph addition | %f | %f\n", _graph_optimizer->getTimeConsumptionSeconds_addition()/_processing_time_total_seconds, _graph_optimizer->getTimeConsumptionSeconds_addition());
  std::printf("pose graph optimization | %f | %f\n", _graph_optimizer->getTimeConsumptionSeconds_optimization()/_processing_time_total_seconds, _graph_optimizer->getTimeConsumptionSeconds_optimization());
  std::printf("       landmark merging | %f | %f\n", _world_map->getTimeConsumptionSeconds_landmark_merging()/_processing_time_total_seconds, _world_map->getTimeConsumptionSeconds_landmark_merging());
  std::cerr << DOUBLE_BAR << std::endl;
}

void SLAMAssembly::reset() {
  _synchronizer.reset();
  _processing_times_seconds.clear();
  _world_map->clear();
}
}
