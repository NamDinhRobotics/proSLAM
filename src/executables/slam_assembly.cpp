#include "slam_assembly.h"

#include "parameter_server.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "aligners/stereouv_aligner.h"
#include "aligners/uvd_aligner.h"
#include "motion_estimation/stereo_tracker.h"
#include "motion_estimation/depth_tracker.h"

namespace proslam {

  //ds default constructor - already allocating required default objects
  SLAMAssembly::SLAMAssembly(): _world_map(new WorldMap()),
                                _optimizer(new GraphOptimizer()),
                                _relocalizer(new Relocalizer()),
                                _tracker(0),
                                _ui_server(0),
                                _viewer_input_images(0),
                                _context_viewer_bird(0),
                                _context_viewer_top(0) {
    _synchronizer.reset();
    _robot_to_world_ground_truth_poses.clear();
  }

  //ds default destructor
  SLAMAssembly::~SLAMAssembly() {
    delete _tracker;
    delete _optimizer;
    delete _relocalizer;
    delete _world_map;

    //ds other structures
    for (CameraMapElement camera_element: _cameras_by_topic) {
      delete camera_element.second;
    }

    //ds free viewers if set
    if (_viewer_input_images) delete _viewer_input_images;
    if (_context_viewer_bird) delete _context_viewer_bird;
    if (_context_viewer_top) delete _context_viewer_top;
    if (_ui_server) delete _ui_server;
  }

  //ds initializes txt_io playback modules
  void SLAMAssembly::initializeMessageFile() {

    //ds check dataset length
    if (ParameterServer::filenameDataset().length() == 0) {
      std::cerr << "ERROR: no dataset provided (enter -h for help)" << std::endl;
      exit(0);
    }

    //ds configure sensor message source
    _sensor_message_reader.open(ParameterServer::filenameDataset());

    //ds terminate on failure
    if (!_sensor_message_reader.good()) {
      ParameterServer::printBanner();
      exit(0);
    }
  }

  BaseTracker* SLAMAssembly::_makeStereoTracker(const Camera* camera_left,
						  const Camera* camera_right){

      StereoUVAligner* pose_optimizer=new StereoUVAligner;
      
      //ds allocate the tracker module with the given cameras
      StereoFramePointGenerator* framepoint_generator=new StereoFramePointGenerator();
      framepoint_generator->setCameraLeft(camera_left);
      framepoint_generator->setCameraRight(camera_right);
      framepoint_generator->setup();
      
      StereoTracker* tracker=new StereoTracker();
      tracker->setCameraLeft(camera_left);
      tracker->setCameraRight(camera_right);
      tracker->setFramePointGenerator(framepoint_generator);
      tracker->setAligner(pose_optimizer);
      tracker->setup();
      return tracker;
  }

  BaseTracker* SLAMAssembly::_makeDepthTracker(const Camera* camera_left,
					       const Camera* camera_right){

      UVDAligner* pose_optimizer=new UVDAligner;
      //ds allocate the tracker module with the given cameras
      DepthFramePointGenerator* framepoint_generator=new DepthFramePointGenerator();
      framepoint_generator->setCameraLeft(camera_left);
      framepoint_generator->setCameraRight(camera_right);
      framepoint_generator->setup();
      
      DepthTracker* tracker=new DepthTracker();
      tracker->setCameraLeft(camera_left);
      tracker->setCameraRight(camera_right);
      tracker->setFramePointGenerator(framepoint_generator);
      tracker->setAligner(pose_optimizer);
      tracker->setup();
      tracker->setMinimumNumberOfLandmarksToTrack(50);
      return tracker;
  }

  
  //ds attempts to load the camera configuration based on the current input setting
  void SLAMAssembly::loadCameras() {

    //ds configure message synchronizer
    std::vector<std::string> camera_topics_synchronized(0);
    camera_topics_synchronized.push_back(ParameterServer::topicImageLeft());
    camera_topics_synchronized.push_back(ParameterServer::topicImageRight());
    _synchronizer.setTimeInterval(0.001);
    _synchronizer.setTopics(camera_topics_synchronized);
    _cameras_by_topic.clear();

    //ds quickly read the first messages to buffer camera info
    srrg_core::BaseMessage* base_message = 0;
    while ((base_message = _sensor_message_reader.readMessage())) {
      srrg_core::BaseSensorMessage* sensor_msg = dynamic_cast<srrg_core::BaseSensorMessage*>(base_message);
      assert(sensor_msg != 0);
      sensor_msg->untaint();

      //ds check for the two set topics
      if (sensor_msg->topic() == ParameterServer::topicImageLeft()) {
        srrg_core::PinholeImageMessage* message_image_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_msg);

        //ds allocate a new camera
        Camera* camera_left = new Camera(message_image_left->image().rows,
                                         message_image_left->image().cols,
                                         message_image_left->cameraMatrix().cast<real>(),
					 message_image_left->offset().cast<real>()
					 );
        _cameras_by_topic.insert(std::make_pair(message_image_left->topic(), camera_left));
      } else if (sensor_msg->topic() == ParameterServer::topicImageRight()) {
        srrg_core::PinholeImageMessage* message_image_right  = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_msg);

        //ds allocate a new camera
        Camera* camera_right = new Camera(message_image_right->image().rows,
                                          message_image_right->image().cols,
                                          message_image_right->cameraMatrix().cast<real>(),
                                          message_image_right->offset().cast<real>()
					  );
        _cameras_by_topic.insert(std::make_pair(message_image_right->topic(), camera_right));
      }
      delete sensor_msg;

      //ds if we got all the information we need
      if (_cameras_by_topic.size() == camera_topics_synchronized.size()) {
        break;
      }
    }
    _sensor_message_reader.close();

    //ds terminate on failure
    if (_cameras_by_topic.size() != camera_topics_synchronized.size()) {
      ParameterServer::printBanner();
      exit(0);
    }

    //ds if no tracker is set
    if (!_tracker) {
      const Camera* camera_left=_cameras_by_topic.at(ParameterServer::topicImageLeft());
      const Camera* camera_right=_cameras_by_topic.at(ParameterServer::topicImageRight());
      switch (ParameterServer::trackerMode()){
        case ParameterServer::Stereo:
	  _tracker=_makeStereoTracker(camera_left, camera_right);
	  break;
      case ParameterServer::Depth:
	  _tracker=_makeDepthTracker(camera_left, camera_right);
	  break;
      default:
	throw std::runtime_error("unknown tracker");
      }
    }

    std::cerr << "loaded cameras: " << _cameras_by_topic.size() << std::endl;
    for (CameraMapElement camera: _cameras_by_topic) {
      std::cerr << "-topic: " << camera.first << ", resolution: " << camera.second->imageCols() << " x " << camera.second->imageRows()
                                              << ", aspect ratio: " << static_cast<real>(camera.second->imageCols())/camera.second->imageRows() << std::endl;
    }
  }

  //ds attempts to load the camera configuration based on the current input setting
  void SLAMAssembly::loadCameras(const Camera* camera_left_, const Camera* camera_right_) {

    //ds if no tracker is set
    if (!_tracker) {
      //ds allocate the tracker module with the given cameras
      switch (ParameterServer::trackerMode()){
        case ParameterServer::Stereo:
	  _tracker=_makeStereoTracker(camera_left_, camera_right_);
	  break;
      case ParameterServer::Depth:
	  _tracker=_makeDepthTracker(camera_left_, camera_right_);
	  break;
      default:
	throw std::runtime_error("unknown tracker");
      }
    }
  }

  //ds initializes gui components
  void SLAMAssembly::initializeGUI(QApplication* ui_server_) {
    if (ParameterServer::optionUseGUI() && _world_map) {
      _ui_server = ui_server_;
      _viewer_input_images = new ViewerInputImages(_world_map);
      _context_viewer_bird = new ViewerOutputMap(_world_map, 0.1, "output: map (bird view)");
      _context_viewer_bird->show();

      //ds orientation flip for proper camera following
      TransformMatrix3D orientation_correction(TransformMatrix3D::Identity());
      orientation_correction.matrix() << 0, -1, 0, 0,
                                         -1, 0, 0, 0,
                                         0, 0, -1, 0,
                                         0, 0, 0, 1;
      _context_viewer_bird->setRotationRobotView(orientation_correction);

      //ds configure custom top viewer if requested
      if (ParameterServer::optionShowTopViewer()) {
        _context_viewer_top = new ViewerOutputMap(_world_map, 1, "output: map (top view)");
        _context_viewer_top->show();
        TransformMatrix3D center_for_kitti_sequence_00;
        center_for_kitti_sequence_00.matrix() << 1, 0, 0, 0,
                                                 0, 0, -1, 200,
                                                 0, 1, 0, 800,
                                                 0, 0, 0, 1;
        _context_viewer_top->setWorldToRobotOrigin(center_for_kitti_sequence_00);
        _context_viewer_top->setFollowRobot(false);
        _context_viewer_top->setWorldToRobotOrigin(orientation_correction*center_for_kitti_sequence_00);
      }
    }
  }

  //ds updated gui components
  void SLAMAssembly::updateGUI() {
    if (ParameterServer::optionUseGUI()) {
      if (_optimizer->numberOfOptimizations() > 0) {
        _context_viewer_bird->setIsOpen(false);
        if (_context_viewer_top) _context_viewer_top->setIsOpen(false);
      }
      _viewer_input_images->initDrawing();
      _viewer_input_images->drawFeatureTracking();
      _viewer_input_images->drawFeatures();
      _is_gui_running = _context_viewer_bird->isVisible() && _viewer_input_images->updateGUI();
      _context_viewer_bird->updateGL();
      if (_context_viewer_top) _context_viewer_top->updateGL();
      _ui_server->processEvents();
    }
  }

  //ds clean gui components
  int32_t SLAMAssembly::closeGUI(const bool& let_user_close_) {

    //ds if no manual termination was requested
    if (_is_gui_running && let_user_close_) {

      //ds exit in viewer if available
      if (ParameterServer::optionUseGUI() && _context_viewer_bird->isVisible()) {
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
    _sensor_message_reader.open(ParameterServer::filenameDataset());
    _robot_to_world_ground_truth_poses.clear();

    //ds frame counts
    Count number_of_processed_frames_total   = 0;
    Count number_of_processed_frames_current = 0;

    //ds store start time
    double time_start_seconds             = srrg_core::getTime();
    const double time_start_seconds_first = srrg_core::getTime();

    //ds start playback
    srrg_core::BaseMessage* base_message = 0;
    while ((base_message = _sensor_message_reader.readMessage()) && _is_gui_running) {
      srrg_core::BaseSensorMessage* sensor_msg = dynamic_cast<srrg_core::BaseSensorMessage*>(base_message);
      assert(sensor_msg != 0);
      sensor_msg->untaint();

      //ds add to synchronizer
      if (sensor_msg->topic() == ParameterServer::topicImageLeft()) {
        _synchronizer.putMessage(sensor_msg);
      } else if (sensor_msg->topic() == ParameterServer::topicImageRight()) {
        _synchronizer.putMessage(sensor_msg);
      } else {
        delete sensor_msg;
      }

      //ds if we have a synchronized package of sensor messages ready
      if (_synchronizer.messagesReady()) {

        //ds buffer sensor data
        srrg_core::PinholeImageMessage* message_image_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(_synchronizer.messages()[0].get());
        srrg_core::PinholeImageMessage* message_image_right = dynamic_cast<srrg_core::PinholeImageMessage*>(_synchronizer.messages()[1].get());

        //ds buffer images and cameras
        cv::Mat intensity_image_left_rectified;
	if(message_image_left->image().type()==CV_8UC3){
	  cvtColor(message_image_left->image(), intensity_image_left_rectified, CV_BGR2GRAY);
	} else {
	  intensity_image_left_rectified = message_image_left->image();
	}

	cv::Mat intensity_image_right_rectified;
	if(message_image_right->image().type()==CV_8UC3){
	  cvtColor(message_image_right->image(), intensity_image_right_rectified, CV_BGR2GRAY);
	} else {
	  intensity_image_right_rectified = message_image_right->image();
	}
	
	Camera* camera_left = _cameras_by_topic.at(message_image_left->topic());

        //ds preprocess the images if desired
        if (ParameterServer::optionEqualizeHistogram()) {
          cv::equalizeHist(intensity_image_left_rectified, intensity_image_left_rectified);
	  if (ParameterServer::trackerMode()!=ParameterServer::Depth)
	    cv::equalizeHist(intensity_image_right_rectified, intensity_image_right_rectified);
        }

        //ds check if first frame and odometry is available
        if (_world_map->frames().size() == 0 && message_image_left->hasOdom()) {
          _world_map->setRobotToWorld(message_image_left->odometry().cast<real>()*camera_left->robotToCamera());
          if (ParameterServer::optionUseGUI()) {
            _context_viewer_bird->setWorldToRobotOrigin((message_image_left->odometry().cast<real>()*camera_left->robotToCamera()).inverse());
          }
        }

	//ds progress SLAM with the new images
        process(intensity_image_left_rectified,
		intensity_image_right_rectified,
		message_image_left->hasOdom() && ParameterServer::optionUseOdometry(),
		message_image_left->odometry().cast<real>());

        //ds record ground truth history for error computation
        if (message_image_left->hasOdom()) {
          addGroundTruthMeasurement(message_image_left->odometry().cast<real>()*camera_left->robotToCamera());
        }

        //ds runtime info
        ++number_of_processed_frames_total;
        ++number_of_processed_frames_current;
        if (number_of_processed_frames_current%100 == 0) {

          //ds compute durations
          const double total_duration_seconds_current = srrg_core::getTime()-time_start_seconds;

          //ds runtime info - depending on set modes
          if (ParameterServer::optionUseRelocalization()) {
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
        updateGUI();
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
			     bool use_odometry,
			     const TransformMatrix3D& odometry) {

    //ds call the tracker
    _tracker->setWorldMap(_world_map);
    _tracker->setIntensityImageLeft(intensity_image_left_);
    StereoTracker* stereo_tracker=dynamic_cast<StereoTracker*>(_tracker);
      
    if (stereo_tracker)
      stereo_tracker->setIntensityImageRight(&intensity_image_right_);

    DepthTracker* depth_tracker=dynamic_cast<DepthTracker*>(_tracker);
    if (depth_tracker)
      depth_tracker->setDepthImageRight(&intensity_image_right_);

    if (use_odometry)
      _tracker->setOdometry(odometry);
    
    _tracker->compute();

    //ds check if relocalization is desired
    if (ParameterServer::optionUseRelocalization()) {

      //ds if we have a valid frame (not the case after the track is lost)
      if (_world_map->currentFrame()) {

        //ds local map generation - regardless of tracker state
        if (_world_map->createLocalMap()) {

          //ds trigger relocalization
          _relocalizer->init(_world_map->currentLocalMap());
	  if (ParameterServer::trackerMode()==ParameterServer::Depth) {
	    // adjust threshold for depth mode/indoor loop closing
	    _relocalizer->aligner()->setMaximumErrorKernel(0.01);
	    _relocalizer->setMinimumNumberOfMatchesPerLandmark(100);
	  }
          _relocalizer->detect();
          _relocalizer->compute();

          //ds check the closures
          for(Closure* closure: _relocalizer->closures()) {
            if (closure->is_valid) {
              assert(_world_map->currentLocalMap() == closure->local_map_query);

              //ds add loop closure constraint
              _world_map->addLoopClosure(_world_map->currentLocalMap(),
                                         closure->local_map_reference,
                                         closure->transform_frame_query_to_frame_reference,
                                         closure->icp_inlier_ratio);
              if (ParameterServer::optionUseGUI()) {
                for (const Correspondence* match: closure->correspondences) {
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
    }
  }

  //ds prints extensive run summary
  void SLAMAssembly::printReport() {

    std::cerr << "printing report" << std::endl;

    std::cerr << "computing squared errors" << std::endl;
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

    std::cerr << "computing RMSEs" << std::endl;
    
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

    std::cerr << "printing shit" << std::endl;
    
    //ds report
    const Count number_of_processed_frames_total = _world_map->frames().size();
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
    std::cerr << "dataset completed" << std::endl;
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
    if (number_of_processed_frames_total == 0) {
      std::cerr << "no frames processed" << std::endl;
    } else {
      std::cerr << "    absolute translation RMSE (m): " << root_mean_squared_error_translation_absolute << std::endl;
      std::cerr << "    relative translation   ME (m): " << mean_error_translation_relative << std::endl;
      std::cerr << "    final translational error (m): " << (_world_map->currentFrame()->robotToWorld().translation()-odometry_robot_to_world_previous_ground_truth.translation()).norm() << std::endl;
      std::cerr << "              total stereo frames: " << number_of_processed_frames_total << std::endl;
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
      std::cerr << "       feature detection: " << _tracker->getTimeConsumptionSeconds_feature_detection()/_duration_total_seconds
                                               << " (" << _tracker->getTimeConsumptionSeconds_feature_detection() << "s)" << std::endl;
      std::cerr << " keypoint regularization: " << _tracker->getTimeConsumptionSeconds_keypoint_pruning()/_duration_total_seconds
                                                << " (" << _tracker->getTimeConsumptionSeconds_keypoint_pruning() << "s)" << std::endl;
      std::cerr << "   descriptor extraction: " << _tracker->getTimeConsumptionSeconds_descriptor_extraction()/_duration_total_seconds
                                               << " (" << _tracker->getTimeConsumptionSeconds_descriptor_extraction() << "s)" << std::endl;
      StereoTracker* stereo_tracker=dynamic_cast<StereoTracker*>(_tracker);
      if (stereo_tracker){
	std::cerr << "  stereo keypoint search: " << stereo_tracker->getTimeConsumptionSeconds_point_triangulation()/_duration_total_seconds
		  << " (" << stereo_tracker->getTimeConsumptionSeconds_point_triangulation() << "s)" << std::endl;
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
    }
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
  }
}
