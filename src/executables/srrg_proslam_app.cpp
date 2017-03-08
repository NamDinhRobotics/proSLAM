#include "qapplication.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_writer.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/pinhole_image_message.h"

#include "core/mapper.h"
#include "core/relocalizer.h"
#include "core/tracker.h"
#include "visualization/viewer_input_images.h"
#include "visualization/viewer_output_map.h"

using namespace proslam;
using namespace srrg_core;

const char* banner[] = {
  "srrg_proslam_app: simple SLAM application",
  " it reads sequentially all elements in the file and tracks the camera",
  "",
  "usage: srrg_proslam_app  [options] [-camera-left-topic <string>] [-camera-right-topic <string>] <messages>",
  "options:",
  "-camera-left-topic <string>",
  "-camera-right-topic <string>",
  " -use-gui:  display gui",
  0
};

//ds playback modules
static SystemUsageCounter system_usage;
MessageTimestampSynchronizer synchronizer;
bool running     = true;
bool use_gui     = false;
TransformMatrix3D world_previous_to_current = TransformMatrix3D::Identity();
StringCameraMap cameras_by_topic;

//ds ui
QApplication* ui_server               = 0;
TrackerViewer* tracker_viewer         = 0;
TrackingContextViewer* context_viewer_bird  = 0;
TrackingContextViewer* context_viewer_top   = 0;

//ds trigger SLAM pipeline
void process(TrackingContext* world_context_,
             TrackerSVI* tracker_,
             Mapper* mapper_,
             Relocalizer* relocalizer_,
             const cv::Mat& intensity_image_left_,
             const cv::Mat& intensity_image_right_,
             const TransformMatrix3D& world_previous_to_current_estimate_ = TransformMatrix3D::Identity());

//ds don't allow any windoof compilation attempt!
int32_t main(int32_t argc, char ** argv) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(0);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds all SLAM modules TODO proper generation
  TrackingContext* world_map = new TrackingContext();
  Mapper* mapper             = new Mapper();
  Relocalizer* relocalizer   = new Relocalizer();
  relocalizer->aligner()->setMaximumErrorKernel(0.5);
  relocalizer->aligner()->setMinimumNumberOfInliers(10);
  relocalizer->aligner()->setMinimumInlierRatio(0.4);
  relocalizer->setMinimumAbsoluteNumberOfMatchesPointwise(25);

  //ds obtain configuration
  synchronizer.setTimeInterval(0.001);
  std::string topic_image_stereo_left      = "/camera_left/image_raw";
  std::string topic_image_stereo_right     = "/camera_right/image_raw";
  std::string filename_sensor_messages     = "";
  int count_added_arguments = 1;
  while(count_added_arguments < argc){
    if (! strcmp(argv[count_added_arguments],"-stereo-camera-left-topic")){
      count_added_arguments++;
      topic_image_stereo_left=argv[count_added_arguments];
    } else if (! strcmp(argv[count_added_arguments],"-stereo-camera-right-topic")){
      count_added_arguments++;
      topic_image_stereo_right=argv[count_added_arguments];
    } else if (! strcmp(argv[count_added_arguments],"-h")) {
      printBanner(banner);
      return 0;
    } else if (! strcmp(argv[count_added_arguments],"-use-gui")) {
      use_gui = true;
    } else {
      filename_sensor_messages=argv[count_added_arguments];
    }
    count_added_arguments++;
  }

  //ds log configuration
  std::cerr << "main|running with params: " << std::endl;
  std::cerr << "main| -camera-left-topic  " << topic_image_stereo_left << std::endl;
  std::cerr << "main| -camera-right-topic " << topic_image_stereo_right << std::endl;
  std::cerr << "main| -messages           " << filename_sensor_messages << std::endl;

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
  synchronizer.setTopics(camera_topics_synchronized);
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

  //ds terminate on failur
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





  //ds allocate tracker with fixed cameras
  TrackerSVI* tracker = new TrackerSVI(cameras_by_topic.at(topic_image_stereo_left), cameras_by_topic.at(topic_image_stereo_right));

  //ds error measurements
  std::vector<real> errors_translation_relative(0);
  std::vector<real> squared_errors_translation_absolute(0);
  std::vector<TransformMatrix3D> robot_to_world_ground_truth_poses(0);

  //ds frame counts
  Count number_of_processed_frames_total   = 0;
  Count number_of_processed_frames_current = 0;

  //ds store start time
  double time_start_seconds       = getTime();
  double time_start_seconds_first = getTime();

  //ds initialize gui
  if (use_gui) {
    ui_server      = new QApplication(argc, argv);
    tracker_viewer = new TrackerViewer(world_map);
    context_viewer_bird = new TrackingContextViewer(world_map, 1, "output: map (bird view)");
    context_viewer_bird->show();
    context_viewer_top = new TrackingContextViewer(world_map, 1, "output: map (top view)");
    context_viewer_top->show();
    TransformMatrix3D center_for_kitti_sequence_00;
    center_for_kitti_sequence_00.matrix() << 1, 0, 0, 0,
                                             0, 0, -1, 200,
                                             0, 1, 0, 800,
                                             0, 0, 0, 1;
    context_viewer_top->setViewpointOrigin(center_for_kitti_sequence_00);
    context_viewer_top->setFollowRobot(false);
  }

  //ds start playback
  base_message = 0;
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
      cv::Mat intensity_image_left  = message_image_left->image();
      cv::Mat intensity_image_right = message_image_right->image();
      Camera* camera_left  = cameras_by_topic.at(message_image_left->topic());

      //ds check if first frame and odometry is available
      if (world_map->frames().size() == 0 && message_image_left->hasOdom()) {
        world_map->setRobotToWorldPrevious(message_image_left->odometry().cast<real>()*camera_left->robotToCamera());
        if (use_gui) {
          context_viewer_bird->setViewpointOrigin((message_image_left->odometry().cast<real>()*camera_left->robotToCamera()).inverse());
        }
      }

      //ds trigger SLAM pipeline with all available input TODO purify this function
      process(world_map,
              tracker,
              mapper,
              relocalizer,
              intensity_image_left,
              intensity_image_right,
              world_previous_to_current);

      if (message_image_left->hasOdom()) {
        const TransformMatrix3D robot_to_world_ground_truth = message_image_left->odometry().cast<real>()*camera_left->robotToCamera();
        robot_to_world_ground_truth_poses.push_back(robot_to_world_ground_truth);
        world_map->currentFrame()->setRobotToWorldOdometry(robot_to_world_ground_truth);
      }

      //ds info
      ++number_of_processed_frames_total;
      ++number_of_processed_frames_current;
      system_usage.update();
      if (number_of_processed_frames_current%100 == 0) {

        //ds compute durations
        const double total_duration_seconds_current = getTime()-time_start_seconds;

        //ds runtime info
        std::cerr << "main|processed frames: " << number_of_processed_frames_total;
        std::cerr << " | current fps: " << number_of_processed_frames_current/total_duration_seconds_current
                              << " (" << number_of_processed_frames_current << "/" << total_duration_seconds_current << "s)";
        std::cerr << " | local maps: " << world_map->keyframes().size()
                                     << " (" << world_map->keyframes().size()/static_cast<real>(number_of_processed_frames_total) << ")" << std::endl;

        //ds reset stats
        time_start_seconds = getTime();
        number_of_processed_frames_current = 0;
      }

      //ds update ui
      synchronizer.reset();
      if (use_gui) {
        tracker_viewer->initDrawing();
        tracker_viewer->drawFeatureTracking();
        tracker_viewer->drawFeatures();
        running = context_viewer_bird->isVisible() && tracker_viewer->updateGUI();
        context_viewer_bird->updateGL();
        context_viewer_top->updateGL();
        ui_server->processEvents();
      }
    }
  }
  const double total_duration_seconds_total = getTime()-time_start_seconds_first;

  //ds compute squared errors
  TransformMatrix3D odometry_robot_to_world_previous_ground_truth = TransformMatrix3D::Identity();
  Index index_frame     = 0;
  Frame* previous_frame = 0;
  for (FramePtrMapElement frame: world_map->frames()) {

    //ds compute squared errors between frames
    if (index_frame > 0) {
      const TransformMatrix3D world_previous_to_current_ground_truth = robot_to_world_ground_truth_poses[index_frame]*odometry_robot_to_world_previous_ground_truth.inverse();
      errors_translation_relative.push_back(((frame.second->robotToWorld()*previous_frame->robotToWorld().inverse()).translation()-world_previous_to_current_ground_truth.translation()).norm());
    }
    squared_errors_translation_absolute.push_back((frame.second->robotToWorld().translation()-robot_to_world_ground_truth_poses[index_frame].translation()).squaredNorm());
    odometry_robot_to_world_previous_ground_truth = robot_to_world_ground_truth_poses[index_frame];
    previous_frame = frame.second;
    index_frame++;
  }
  robot_to_world_ground_truth_poses.clear();

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
  std::cerr << "main|-------------------------------------------------------------------------" << std::endl;
  std::cerr << "main|dataset completed" << std::endl;
  std::cerr << "main|-------------------------------------------------------------------------" << std::endl;
  if (number_of_processed_frames_total == 0) {
    std::cerr << "main|no frames processed" << std::endl;
  } else {
    std::cerr << "main|    absolute translation RMSE (m): " << root_mean_squared_error_translation_absolute << std::endl;
    std::cerr << "main|    relative translation   ME (m): " << mean_error_translation_relative << std::endl;
    std::cerr << "main|    final translational error (m): " << (world_map->currentFrame()->robotToWorld().translation()-odometry_robot_to_world_previous_ground_truth.translation()).norm() << std::endl;
    std::cerr << "main|              total stereo frames: " << number_of_processed_frames_total << std::endl;
    std::cerr << "main|               total duration (s): " << total_duration_seconds_total << std::endl;
    std::cerr << "main|                      average fps: " << number_of_processed_frames_total/total_duration_seconds_total << std::endl;
    std::cerr << "main|average processing time (s/frame): " << total_duration_seconds_total/number_of_processed_frames_total << std::endl;
    std::cerr << "main|average landmarks close per frame: " << tracker->totalNumberOfLandmarksClose()/number_of_processed_frames_total << std::endl;
    std::cerr << "main|  average landmarks far per frame: " << tracker->totalNumberOfLandmarksFar()/number_of_processed_frames_total << std::endl;
    std::cerr << "main|         average tracks per frame: " << tracker->totalNumberOfTrackedPoints()/number_of_processed_frames_total << std::endl;
    std::cerr << "main|        average tracks per second: " << tracker->totalNumberOfTrackedPoints()/total_duration_seconds_total << std::endl;
    std::cerr << "main|-------------------------------------------------------------------------" << std::endl;
    std::cerr << "main|runtime" << std::endl;
    std::cerr << "main|-------------------------------------------------------------------------" << std::endl;
    std::cerr << "main|       feature detection: " << tracker->getTimeConsumptionSeconds_feature_detection()/total_duration_seconds_total
                                             << " (" << tracker->getTimeConsumptionSeconds_feature_detection() << "s)" << std::endl;
    std::cerr << "main| keypoint regularization: " << tracker->getTimeConsumptionSeconds_keypoint_pruning()/total_duration_seconds_total
                                              << " (" << tracker->getTimeConsumptionSeconds_keypoint_pruning() << "s)" << std::endl;
    std::cerr << "main|   descriptor extraction: " << tracker->getTimeConsumptionSeconds_descriptor_extraction()/total_duration_seconds_total
                                             << " (" << tracker->getTimeConsumptionSeconds_descriptor_extraction() << "s)" << std::endl;
    std::cerr << "main|  stereo keypoint search: " << tracker->getTimeConsumptionSeconds_point_triangulation()/total_duration_seconds_total
                                             << " (" << tracker->getTimeConsumptionSeconds_point_triangulation() << "s)" << std::endl;
    std::cerr << "main|                tracking: " << tracker->getTimeConsumptionSeconds_tracking()/total_duration_seconds_total
                                             << " (" << tracker->getTimeConsumptionSeconds_tracking() << "s)" << std::endl;
    std::cerr << "main|       pose optimization: " << tracker->getTimeConsumptionSeconds_pose_optimization()/total_duration_seconds_total
                                             << " (" << tracker->getTimeConsumptionSeconds_pose_optimization() << "s)" << std::endl;
    std::cerr << "main|   landmark optimization: " << tracker->getTimeConsumptionSeconds_landmark_optimization()/total_duration_seconds_total
                                             << " (" << tracker->getTimeConsumptionSeconds_landmark_optimization() << "s)" << std::endl;
    std::cerr << "main| correspondence recovery: " << tracker->getTimeConsumptionSeconds_point_recovery()/total_duration_seconds_total
                                             << " (" << tracker->getTimeConsumptionSeconds_point_recovery() << "s)" << std::endl;
    std::cerr << "main|similarity search (HBST): " << relocalizer->getTimeConsumptionSeconds_overall()/total_duration_seconds_total
                                             << " (" << relocalizer->getTimeConsumptionSeconds_overall() << "s)" << std::endl;
    std::cerr << "main|              map update: " << mapper->getTimeConsumptionSeconds_overall()/total_duration_seconds_total
                                             << " (" << mapper->getTimeConsumptionSeconds_overall() << "s)" << std::endl;
  }
  std::cerr << "main|-------------------------------------------------------------------------" << std::endl;

  //ds factory cleanup
  AlignerFactory::clear();
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
    if (use_gui && context_viewer_bird->isVisible() && context_viewer_top->isVisible()) {
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
      delete context_viewer_top;
      delete ui_server;
    }
    return 0;
  }
}

void process(TrackingContext* world_map_,
             TrackerSVI* tracker_,
             Mapper* mapper_,
             Relocalizer* relocalizer_,
             const cv::Mat& intensity_image_left_,
             const cv::Mat& intensity_image_right_,
             const TransformMatrix3D& world_previous_to_current_estimate_) {

  //ds call the tracker
  world_previous_to_current = tracker_->addImage(world_map_,
                                                 intensity_image_left_,
                                                 intensity_image_right_,
                                                 world_previous_to_current_estimate_);

  //ds if we have a valid frame (not the case after the track is lost)
  if (world_map_->currentFrame() != 0) {

    //ds local map generation - regardless of tracker state
    if (world_map_->createLocalMap()) {

      //ds if we have a fresh track (start or lost)
      if (world_map_->keyframes().size() == 1) {
        relocalizer_->flush();
      }

      //ds trigger relocalization
      relocalizer_->init(world_map_->currentKeyframe());
      relocalizer_->detect();
      relocalizer_->compute();

      //ds check the closures
      for(CorrespondenceCollection* closure: relocalizer_->closures()) {
        if (closure->is_valid) {
          assert(world_map_->currentKeyframe() == closure->keyframe_query);

          //ds add loop closure constraint
          world_map_->closeKeyframes(world_map_->currentKeyframe(),
                                                                 closure->keyframe_reference,
                                                                 closure->transform_frame_query_to_frame_reference);
          if (use_gui) {
            for (const Correspondence* match: closure->correspondences) {
              world_map_->landmarks().get(match->item_query->landmark()->index())->setIsInLoopClosureQuery(true);
              world_map_->landmarks().get(match->item_reference->landmark()->index())->setIsInLoopClosureReference(true);
            }
          }
        }
      }
      relocalizer_->train();
    }

    //ds check if we closed a local map
    if (world_map_->closedKeyframe()) {

      //ds optimize graph
      mapper_->optimize(world_map_);

      //ds wipe non-optimized landmarks
      world_map_->purifyLandmarks();
    }
  }
}
