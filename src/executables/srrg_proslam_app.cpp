#include "qapplication.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/pinhole_image_message.h"

#include "map_optimization/graph_optimizer.h"
#include "relocalization/relocalizer.h"
#include "motion_estimation/tracker.h"
#include "visualization/viewer_input_images.h"
#include "visualization/viewer_output_map.h"

using namespace proslam;
using namespace srrg_core;

//ds TODO make sure this is up to date!
const char* banner[] = {
  "-------------------------------------------------------------------------",
  "srrg_proslam_app: simple SLAM application",
  "usage: srrg_proslam_app [options] <dataset>",
  "",
  "<dataset>: path to a SRRG txt_io dataset file",
  "",
  "[options]",
  "-camera-left-topic <string>:  topic name in txt_io dataset file)",
  "-camera-right-topic <string>: topic name in txt_io dataset file)",
  "-use-gui:                     displays GUI elements",
  "-open:                        disables relocalization",
  "-show-top:                    enable top map viewer",
  "-------------------------------------------------------------------------",
  0
};

//ds playback modules - lazy globals
static SystemUsageCounter system_usage;
MessageTimestampSynchronizer synchronizer;
bool use_gui                                = false;
bool use_relocalization                     = true;
TransformMatrix3D world_previous_to_current = TransformMatrix3D::Identity();

//ds user interface related
QApplication* ui_server              = 0;
ViewerInputImages* tracker_viewer    = 0;
ViewerOutputMap* context_viewer_bird = 0;
ViewerOutputMap* context_viewer_top  = 0;

//ds process a pair of rectified and undistorted stereo images
void process(WorldMap* world_map_,
             Tracker* tracker_,
             GraphOptimizer* mapper_,
             Relocalizer* relocalizer_,
             const cv::Mat& intensity_image_left_,
             const cv::Mat& intensity_image_right_,
             const TransformMatrix3D& world_previous_to_current_estimate_ = TransformMatrix3D::Identity());

//ds prints extensive run summary
void printReport(const std::vector<TransformMatrix3D>& robot_to_world_ground_truth_poses_,
                 const Count& number_of_processed_frames_total_,
                 const real& duration_total_seconds_,
                 const WorldMap* world_context_,
                 const Tracker* tracker_,
                 const GraphOptimizer* mapper_,
                 const Relocalizer* relocalizer_);

//ds don't allow any windoof compilation attempt!
int32_t main(int32_t argc, char ** argv) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(0);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds obtain configuration
  std::string topic_image_stereo_left  = "/camera_left/image_raw";
  std::string topic_image_stereo_right = "/camera_right/image_raw";
  std::string filename_sensor_messages = "";
  int32_t count_added_arguments        = 1;
  bool show_top_viewer                 = false;
  while(count_added_arguments < argc){
    if (!std::strcmp(argv[count_added_arguments], "-camera-left-topic")){
      count_added_arguments++;
      topic_image_stereo_left = argv[count_added_arguments];
    } else if (!std::strcmp(argv[count_added_arguments], "-camera-right-topic")){
      count_added_arguments++;
      topic_image_stereo_right = argv[count_added_arguments];
    } else if (!std::strcmp(argv[count_added_arguments], "-h")) {
      printBanner(banner);
      return 0;
    } else if (!std::strcmp(argv[count_added_arguments], "-use-gui")) {
      use_gui = true;
    } else if (!std::strcmp(argv[count_added_arguments], "-open")) {
      use_relocalization = false;
    } else if (!std::strcmp(argv[count_added_arguments], "-show-top")) {
      show_top_viewer = true;
    } else {
      filename_sensor_messages = argv[count_added_arguments];
    }
    count_added_arguments++;
  }

  //ds log configuration
  std::cerr << "main|-------------------------------------------------------------------------" << std::endl;
  std::cerr << "main|running with params: " << std::endl;
  std::cerr << "main|-camera-left-topic  " << topic_image_stereo_left << std::endl;
  std::cerr << "main|-camera-right-topic " << topic_image_stereo_right << std::endl;
  std::cerr << "main|-use-gui            " << use_gui << std::endl;
  std::cerr << "main|-open               " << !use_relocalization << std::endl;
  std::cerr << "main|-show-top           " << show_top_viewer << std::endl;
  std::cerr << "main|-dataset            " << filename_sensor_messages << std::endl;
  std::cerr << "main|-------------------------------------------------------------------------" << std::endl;

  //ds configure sensor message source
  if (filename_sensor_messages.length() == 0) {
      printBanner(banner);
      return 0;
  }
  MessageReader sensor_message_reader;
  sensor_message_reader.open(filename_sensor_messages);

  //ds configure message synchronizer
  std::vector<std::string> camera_topics_synchronized;
  camera_topics_synchronized.push_back(topic_image_stereo_left);
  camera_topics_synchronized.push_back(topic_image_stereo_right);
  synchronizer.setTimeInterval(0.001);
  synchronizer.setTopics(camera_topics_synchronized);
  StringCameraMap cameras_by_topic;
  cameras_by_topic.clear();

  //ds quickly read the first messages to buffer camera info
  BaseMessage* base_message = 0;
  while ((base_message = sensor_message_reader.readMessage())) {
    BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(base_message);
    assert(sensor_msg != 0);
    sensor_msg->untaint();

    //ds check for the two set topics
    if (sensor_msg->topic() == topic_image_stereo_left) {
      PinholeImageMessage* message_image_left  = dynamic_cast<PinholeImageMessage*>(sensor_msg);

      //ds allocate a new camera
      Camera* camera_left = new Camera(message_image_left->image().rows,
                                       message_image_left->image().cols,
                                       message_image_left->cameraMatrix().cast<real>(),
                                       message_image_left->offset().cast<real>());
      cameras_by_topic.insert(std::make_pair(message_image_left->topic(), camera_left));
    } else if (sensor_msg->topic() == topic_image_stereo_right) {
      PinholeImageMessage* message_image_right  = dynamic_cast<PinholeImageMessage*>(sensor_msg);

      //ds allocate a new camera
      Camera* camera_right = new Camera(message_image_right->image().rows,
                                        message_image_right->image().cols,
                                        message_image_right->cameraMatrix().cast<real>(),
                                        message_image_right->offset().cast<real>());
      cameras_by_topic.insert(std::make_pair(message_image_right->topic(), camera_right));
    }
    delete sensor_msg;

    //ds if we got all the information we need
    if (cameras_by_topic.size() == camera_topics_synchronized.size()) {
      break;
    }
  }

  //ds terminate on failure
  if (cameras_by_topic.size() != camera_topics_synchronized.size()) {
    printBanner(banner);
    return 0;
  }

  //ds restart stream
  sensor_message_reader.close();
  sensor_message_reader.open(filename_sensor_messages);

  std::cerr << "main|loaded cameras: " << cameras_by_topic.size() << std::endl;
  for (StringCameraMapElement camera: cameras_by_topic) {
    std::cerr << "main|" << camera.first << " - resolution: " << camera.second->imageCols() << " x " << camera.second->imageRows()
                                         << " aspect ratio: " << static_cast<real>(camera.second->imageCols())/camera.second->imageRows() << std::endl;
  }

  //ds allocate SLAM modules
  WorldMap* world_map      = new WorldMap();
  GraphOptimizer* mapper   = new GraphOptimizer();
  Relocalizer* relocalizer = new Relocalizer();
  Tracker* tracker         = new Tracker(cameras_by_topic.at(topic_image_stereo_left), cameras_by_topic.at(topic_image_stereo_right));

  //ds configure SLAM modules
  relocalizer->aligner()->setMaximumErrorKernel(0.5);
  relocalizer->aligner()->setMinimumNumberOfInliers(25);
  relocalizer->aligner()->setMinimumInlierRatio(0.5);
  relocalizer->setMinimumAbsoluteNumberOfMatchesPointwise(25);

  //ds initialize gui
  if (use_gui) {
    ui_server      = new QApplication(argc, argv);
    tracker_viewer = new ViewerInputImages(world_map);
    context_viewer_bird = new ViewerOutputMap(world_map, 0.1, "output: map (bird view)");
    context_viewer_bird->show();

    //ds orientation flip for proper camera following
    TransformMatrix3D orientation_correction;
    orientation_correction.matrix() << 0, -1, 0, 0,
                                       -1, 0, 0, 0,
                                       0, 0, -1, 0,
                                       0, 0, 0, 1;
    context_viewer_bird->setRotationRobotView(orientation_correction);

    //ds configure custom top viewer if requested
    if (show_top_viewer) {
      context_viewer_top = new ViewerOutputMap(world_map, 1, "output: map (top view)");
      context_viewer_top->show();
      TransformMatrix3D center_for_kitti_sequence_00;
      center_for_kitti_sequence_00.matrix() << 1, 0, 0, 0,
                                               0, 0, -1, 200,
                                               0, 1, 0, 800,
                                               0, 0, 0, 1;
      context_viewer_top->setWorldToRobotOrigin(center_for_kitti_sequence_00);
      context_viewer_top->setFollowRobot(false);
      context_viewer_top->setWorldToRobotOrigin(orientation_correction*center_for_kitti_sequence_00);
    }
  }

  //ds error measurements
  std::vector<TransformMatrix3D> robot_to_world_ground_truth_poses(0);

  //ds frame counts
  Count number_of_processed_frames_total   = 0;
  Count number_of_processed_frames_current = 0;

  //ds store start time
  double time_start_seconds       = getTime();
  const double time_start_seconds_first = getTime();

  //ds start playback
  base_message = 0;
  bool running = true;
  while ((base_message = sensor_message_reader.readMessage()) && running) {
    BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(base_message);
    assert(sensor_msg != 0);
    sensor_msg->untaint();

    //ds add to synchronizer
    if (sensor_msg->topic() == topic_image_stereo_left) {
      synchronizer.putMessage(sensor_msg);
    } else if (sensor_msg->topic() == topic_image_stereo_right) {
      synchronizer.putMessage(sensor_msg);
    } else {
      delete sensor_msg;
    }

    //ds if we have a synchronized package of sensor messages ready
    if (synchronizer.messagesReady()) {

      //ds buffer sensor data
      PinholeImageMessage* message_image_left  = dynamic_cast<PinholeImageMessage*>(synchronizer.messages()[0].get());
      PinholeImageMessage* message_image_right = dynamic_cast<PinholeImageMessage*>(synchronizer.messages()[1].get());

      //ds buffer images and cameras
      cv::Mat intensity_image_left_rectified  = message_image_left->image();
      cv::Mat intensity_image_right_rectified = message_image_right->image();
      Camera* camera_left  = cameras_by_topic.at(message_image_left->topic());

      //ds check if first frame and odometry is available
      if (world_map->frames().size() == 0 && message_image_left->hasOdom()) {
        world_map->setRobotToWorldPrevious(message_image_left->odometry().cast<real>()*camera_left->robotToCamera());
        if (use_gui) {
          context_viewer_bird->setWorldToRobotOrigin((message_image_left->odometry().cast<real>()*camera_left->robotToCamera()).inverse());
        }
      }

      //ds trigger SLAM pipeline with all available input TODO purify this function
      process(world_map,
              tracker,
              mapper,
              relocalizer,
              intensity_image_left_rectified,
              intensity_image_right_rectified,
              world_previous_to_current);

      //ds record ground truth history for error computation
      if (message_image_left->hasOdom()) {
        const TransformMatrix3D robot_to_world_ground_truth = message_image_left->odometry().cast<real>()*camera_left->robotToCamera();
        robot_to_world_ground_truth_poses.push_back(robot_to_world_ground_truth);
        world_map->currentFrame()->setRobotToWorldGroundTruth(robot_to_world_ground_truth);
      }

      //ds runtime info
      ++number_of_processed_frames_total;
      ++number_of_processed_frames_current;
      system_usage.update();
      if (number_of_processed_frames_current%100 == 0) {

        //ds compute durations
        const double total_duration_seconds_current = getTime()-time_start_seconds;

        //ds runtime info
        std::printf("main|processed frames: %5lu|landmarks: %6lu|local maps: %4lu (%3.2f)|closures: %3lu (%3.2f)|current fps: %5.2f (%3lu/%3.2fs)\n",
                    number_of_processed_frames_total,
                    world_map->landmarks().size(),
                    world_map->localMaps().size(),
                    world_map->localMaps().size()/static_cast<real>(number_of_processed_frames_total),
                    world_map->numberOfClosures(),
                    world_map->numberOfClosures()/static_cast<real>(world_map->localMaps().size()),
                    number_of_processed_frames_current/total_duration_seconds_current,
                    number_of_processed_frames_current,
                    total_duration_seconds_current);

        //ds reset stats
        time_start_seconds = getTime();
        number_of_processed_frames_current = 0;
      }
      synchronizer.reset();

      //ds update ui
      if (use_gui) {
        if (mapper->numberOfOptimizations() > 0) {
          context_viewer_bird->setIsOpen(false);
          if (context_viewer_top) context_viewer_top->setIsOpen(false);
        }
        tracker_viewer->initDrawing();
        tracker_viewer->drawFeatureTracking();
        tracker_viewer->drawFeatures();
        running = context_viewer_bird->isVisible() && tracker_viewer->updateGUI();
        context_viewer_bird->updateGL();
        if (context_viewer_top) context_viewer_top->updateGL();
        ui_server->processEvents();
      }
    }
  }
  const double duration_total_seconds = getTime()-time_start_seconds_first;

  //ds print full report
  printReport(robot_to_world_ground_truth_poses,
              number_of_processed_frames_total,
              duration_total_seconds,
              world_map,
              tracker,
              mapper,
              relocalizer);
  robot_to_world_ground_truth_poses.clear();
  sensor_message_reader.close();

  //ds save trajectory to disk
  world_map->writeTrajectory("trajectory.txt");

  //ds controlled destruction: SLAM modules
  delete tracker;
  delete mapper;
  delete relocalizer;

  //ds other structures
  for (StringCameraMapElement camera_element: cameras_by_topic) {
    delete camera_element.second;
  }

  if (use_gui) {
    delete tracker_viewer;
  }

  //ds if no manual termination was requested
  if (running) {

    //ds exit in viewer if available
    if (use_gui && context_viewer_bird->isVisible()) {
      return ui_server->exec();
    } else {
      if (use_gui) {
        delete ui_server;
      }
      delete world_map;
      return 0;
    }
  } else {

    //ds destroy world context
    delete world_map;
    if (use_gui) {
      delete context_viewer_bird;
      if (context_viewer_top) delete context_viewer_top;
      delete ui_server;
    }
    return 0;
  }
}

//ds process a pair of rectified and undistorted stereo images
void process(WorldMap* world_map_,
             Tracker* tracker_,
             GraphOptimizer* mapper_,
             Relocalizer* relocalizer_,
             const cv::Mat& intensity_image_left_,
             const cv::Mat& intensity_image_right_,
             const TransformMatrix3D& world_previous_to_current_estimate_) {

  //ds call the tracker
  world_previous_to_current = tracker_->compute(world_map_,
                                                intensity_image_left_,
                                                intensity_image_right_,
                                                world_previous_to_current_estimate_);

  //ds check if relocalization is desired
  if (use_relocalization) {

    //ds if we have a valid frame (not the case after the track is lost)
    if (world_map_->currentFrame() != 0) {

      //ds local map generation - regardless of tracker state
      if (world_map_->createLocalMap()) {

        //ds if we have a fresh track (start or lost)
        if (world_map_->localMaps().size() == 1) {
          relocalizer_->flush();
        }

        //ds trigger relocalization
        relocalizer_->init(world_map_->currentLocalMap());
        relocalizer_->detect();
        relocalizer_->compute();

        //ds check the closures
        for(CorrespondenceCollection* closure: relocalizer_->closures()) {
          if (closure->is_valid) {
            assert(world_map_->currentLocalMap() == closure->local_map_query);

            //ds add loop closure constraint
            world_map_->closeLocalMaps(world_map_->currentLocalMap(),
                                       closure->local_map_reference,
                                       closure->transform_frame_query_to_frame_reference);
            if (use_gui) {
              for (const Correspondence* match: closure->correspondences) {
                world_map_->landmarks().get(match->item_query->landmark->identifier())->setIsInLoopClosureQuery(true);
                world_map_->landmarks().get(match->item_reference->landmark->identifier())->setIsInLoopClosureReference(true);
              }
            }
          }
        }
        relocalizer_->train();
      }

      //ds check if we closed a local map
      if (world_map_->relocalized()) {

        //ds optimize graph
        mapper_->optimize(world_map_);
      }
    }
  }
}

//ds prints extensive run summary
void printReport(const std::vector<TransformMatrix3D>& robot_to_world_ground_truth_poses_,
                 const Count& number_of_processed_frames_total_,
                 const real& duration_total_seconds_,
                 const WorldMap* world_context_,
                 const Tracker* tracker_,
                 const GraphOptimizer* mapper_,
                 const Relocalizer* relocalizer_) {

  //ds compute squared errors
  std::vector<real> errors_translation_relative(0);
  std::vector<real> squared_errors_translation_absolute(0);
  TransformMatrix3D odometry_robot_to_world_previous_ground_truth = TransformMatrix3D::Identity();
  Index index_frame     = 0;
  Frame* previous_frame = 0;
  for (FramePointerMapElement frame: world_context_->frames()) {

    //ds compute squared errors between frames
    if (index_frame > 0) {
      const TransformMatrix3D world_previous_to_current_ground_truth = robot_to_world_ground_truth_poses_[index_frame]*odometry_robot_to_world_previous_ground_truth.inverse();
      errors_translation_relative.push_back(((frame.second->robotToWorld()*previous_frame->robotToWorld().inverse()).translation()-world_previous_to_current_ground_truth.translation()).norm());
    }
    squared_errors_translation_absolute.push_back((frame.second->robotToWorld().translation()-robot_to_world_ground_truth_poses_[index_frame].translation()).squaredNorm());
    odometry_robot_to_world_previous_ground_truth = robot_to_world_ground_truth_poses_[index_frame];
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

  //ds report
  std::cerr << "main|printReport|-------------------------------------------------------------------------" << std::endl;
  std::cerr << "main|printReport|dataset completed" << std::endl;
  std::cerr << "main|printReport|-------------------------------------------------------------------------" << std::endl;
  if (number_of_processed_frames_total_ == 0) {
    std::cerr << "main|no frames processed" << std::endl;
  } else {
    std::cerr << "main|printReport|    absolute translation RMSE (m): " << root_mean_squared_error_translation_absolute << std::endl;
    std::cerr << "main|printReport|    relative translation   ME (m): " << mean_error_translation_relative << std::endl;
    std::cerr << "main|printReport|    final translational error (m): " << (world_context_->currentFrame()->robotToWorld().translation()-odometry_robot_to_world_previous_ground_truth.translation()).norm() << std::endl;
    std::cerr << "main|printReport|              total stereo frames: " << number_of_processed_frames_total_ << std::endl;
    std::cerr << "main|printReport|               total duration (s): " << duration_total_seconds_ << std::endl;
    std::cerr << "main|printReport|                      average fps: " << number_of_processed_frames_total_/duration_total_seconds_ << std::endl;
    std::cerr << "main|printReport|average processing time (s/frame): " << duration_total_seconds_/number_of_processed_frames_total_ << std::endl;
    std::cerr << "main|printReport|average landmarks close per frame: " << tracker_->totalNumberOfLandmarksClose()/number_of_processed_frames_total_ << std::endl;
    std::cerr << "main|printReport|  average landmarks far per frame: " << tracker_->totalNumberOfLandmarksFar()/number_of_processed_frames_total_ << std::endl;
    std::cerr << "main|printReport|         average tracks per frame: " << tracker_->totalNumberOfTrackedPoints()/number_of_processed_frames_total_ << std::endl;
    std::cerr << "main|printReport|        average tracks per second: " << tracker_->totalNumberOfTrackedPoints()/duration_total_seconds_ << std::endl;
    std::cerr << "main|printReport|-------------------------------------------------------------------------" << std::endl;
    std::cerr << "main|printReport|runtime" << std::endl;
    std::cerr << "main|printReport|-------------------------------------------------------------------------" << std::endl;
    std::cerr << "main|printReport|       feature detection: " << tracker_->getTimeConsumptionSeconds_feature_detection()/duration_total_seconds_
                                             << " (" << tracker_->getTimeConsumptionSeconds_feature_detection() << "s)" << std::endl;
    std::cerr << "main|printReport| keypoint regularization: " << tracker_->getTimeConsumptionSeconds_keypoint_pruning()/duration_total_seconds_
                                              << " (" << tracker_->getTimeConsumptionSeconds_keypoint_pruning() << "s)" << std::endl;
    std::cerr << "main|printReport|   descriptor extraction: " << tracker_->getTimeConsumptionSeconds_descriptor_extraction()/duration_total_seconds_
                                             << " (" << tracker_->getTimeConsumptionSeconds_descriptor_extraction() << "s)" << std::endl;
    std::cerr << "main|printReport|  stereo keypoint search: " << tracker_->getTimeConsumptionSeconds_point_triangulation()/duration_total_seconds_
                                             << " (" << tracker_->getTimeConsumptionSeconds_point_triangulation() << "s)" << std::endl;
    std::cerr << "main|printReport|                tracking: " << tracker_->getTimeConsumptionSeconds_tracking()/duration_total_seconds_
                                             << " (" << tracker_->getTimeConsumptionSeconds_tracking() << "s)" << std::endl;
    std::cerr << "main|printReport|       pose optimization: " << tracker_->getTimeConsumptionSeconds_pose_optimization()/duration_total_seconds_
                                             << " (" << tracker_->getTimeConsumptionSeconds_pose_optimization() << "s)" << std::endl;
    std::cerr << "main|printReport|   landmark optimization: " << tracker_->getTimeConsumptionSeconds_landmark_optimization()/duration_total_seconds_
                                             << " (" << tracker_->getTimeConsumptionSeconds_landmark_optimization() << "s)" << std::endl;
    std::cerr << "main|printReport| correspondence recovery: " << tracker_->getTimeConsumptionSeconds_point_recovery()/duration_total_seconds_
                                             << " (" << tracker_->getTimeConsumptionSeconds_point_recovery() << "s)" << std::endl;
    std::cerr << "main|printReport|similarity search (HBST): " << relocalizer_->getTimeConsumptionSeconds_overall()/duration_total_seconds_
                                             << " (" << relocalizer_->getTimeConsumptionSeconds_overall() << "s)" << std::endl;
    std::cerr << "main|printReport|              map update: " << mapper_->getTimeConsumptionSeconds_overall()/duration_total_seconds_
                                             << " (" << mapper_->getTimeConsumptionSeconds_overall() << "s)" << std::endl;
  }
  std::cerr << "main|printReport|-------------------------------------------------------------------------" << std::endl;
}
