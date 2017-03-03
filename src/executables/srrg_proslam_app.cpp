#include <cstring>
#include "qapplication.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_writer.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/pinhole_image_message.h"

#include "visualization/viewer_input_images.h"
#include "visualization/viewer_output_map.h"
#include "core/gt_mapper.h"
#include "core/gt_relocalizer.h"
#include "core/tracker_svi.h"

using namespace gslam;
using namespace srrg_core;

const char* banner[] = {
  "srrg_gslam_mapper_app: simple mapper application",
  " it reads sequentially all elements in the file and tracks the camera",
  "",
  "usage: srrg_gslam_mapper_app  [options] [-stereo-camera-left-topic <string>] [-stereo-camera-right-topic <string>] <dump_file>",
  "options:",
  " -o <file>:  string, sensor message output file of the mapper",
  " -map <file>:  string, serialized map output file of the mapper",
  " -tracker-max-features:  int, max number of features in tracker",
  " -tracker-min-age:  int, number of consecutive frames a feature should be detected before being promoted",
  " -tracker-min-covered-region:  floar, region of the image that needs to be covered with redetected features before a redetection is thrown",
  0
};

//ds playback modules
static SystemUsageCounter system_usage;
MessageTimestampSynchronizer synchronizer;
MessageWriter sensor_message_writer;
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
void process(WorldContext* world_context_,
             TrackerSVI* tracker_,
             Mapper* mapper_,
             Relocalizer* relocalizer_,
             const Camera* camera_left_,
             const cv::Mat& intensity_image_left_,
             const Camera* camera_right_,
             const cv::Mat& intensity_image_right_,
             const TransformMatrix3D& world_previous_to_current_estimate_ = TransformMatrix3D::Identity());

//ds don't allow any windoof compilation attempt!
int32_t main(int32_t argc, char ** argv) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(1);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds all SLAM modules TODO proper generation
  WorldContext* world_context = new WorldContext();
  world_context->aligner()->setMaximumErrorKernel(0.5);
  world_context->aligner()->setMinimumInlierRatio(0.5);
  Mapper* mapper           = new Mapper();
  Relocalizer* relocalizer = new Relocalizer();
  relocalizer->aligner()->setMaximumErrorKernel(0.5);
  relocalizer->aligner()->setMinimumNumberOfInliers(10);
  relocalizer->aligner()->setMinimumInlierRatio(0.4);
  relocalizer->setMinimumAbsoluteNumberOfMatchesPointwise(25);

  //ds obtain configuration
  synchronizer.setTimeInterval(0.001);
  std::string topic_image_stereo_left      = "/camera_left/image_raw";
  std::string topic_image_stereo_right     = "/camera_right/image_raw";
  std::string filename_sensor_messages     = "";
  std::string filename_sensor_messages_out = "";
  std::string filename_save_map            = "";
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
    } else if (! strcmp(argv[count_added_arguments],"-o")){
      count_added_arguments++;
      filename_sensor_messages_out=argv[count_added_arguments];
    } else if (! strcmp(argv[count_added_arguments],"-map")){
      count_added_arguments++;
      filename_save_map=argv[count_added_arguments];
    } else {
      filename_sensor_messages=argv[count_added_arguments];
    }
    count_added_arguments++;
  }

  //ds log configuration
  std::cerr << "running with params: " << std::endl;
  std::cerr << "-stereo-camera-left-topic      " << topic_image_stereo_left << std::endl;
  std::cerr << "-stereo-camera-right-topic     " << topic_image_stereo_right << std::endl;
  std::cerr << "-messages                      " << filename_sensor_messages << std::endl;
  if (filename_sensor_messages_out.length()) {
    std::cerr << "-o                             " << filename_sensor_messages_out << std::endl;
  }
  if (filename_save_map.length()) {
    std::cerr << "-map                           " << filename_save_map << std::endl;
  }

  //ds clean filetree
  Optimizer::clearFilesUNIX();

  //ds configure sensor message source
  if (filename_sensor_messages.length() == 0) {
      printBanner(banner);
      return 0;
  }
  MessageReader sensor_message_reader;
  sensor_message_reader.open(filename_sensor_messages);

  //ds set outfiles if present
  if (filename_sensor_messages_out.length() > 0){
    sensor_message_writer.open(filename_sensor_messages_out);
  }

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
                                       message_image_left->cameraMatrix().cast<gt_real>(),
                                       message_image_left->offset().cast<gt_real>());
      cameras_by_topic.insert(std::make_pair(message_image_left->topic(), camera_left));
    } else if (sensor_msg->topic() == topic_image_stereo_right) {
      PinholeImageMessage* message_image_right  = dynamic_cast<PinholeImageMessage*>(sensor_msg);

      //ds allocate a new camera
      Camera* camera_right = new Camera(message_image_right->image().rows,
                                        message_image_right->image().cols,
                                        message_image_right->cameraMatrix().cast<gt_real>(),
                                        message_image_right->offset().cast<gt_real>());
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

  std::cerr << "loaded cameras: " << cameras_by_topic.size() << std::endl;
  for (StringCameraMapElement camera: cameras_by_topic) {
    std::cerr << camera.first << " - resolution: " << camera.second->imageCols() << " x " << camera.second->imageRows()
                              << " aspect ratio: " << static_cast<gt_real>(camera.second->imageCols())/camera.second->imageRows() << std::endl;
  }





  //ds allocate tracker with fixed cameras
  TrackerSVI* tracker = new TrackerSVI(world_context, cameras_by_topic.at(topic_image_stereo_left), cameras_by_topic.at(topic_image_stereo_right));

  //ds error measurements
  std::vector<gt_real> errors_translation_relative(0);
  std::vector<gt_real> squared_errors_translation_absolute(0);
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
    tracker_viewer = new TrackerViewer(world_context);
    context_viewer_bird = new TrackingContextViewer(world_context);
    context_viewer_bird->show();
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
      //cv::equalizeHist(message_image_left->image(), intensity_image_left);
      //cv::equalizeHist(message_image_right->image(), intensity_image_right);
      Camera* camera_left  = cameras_by_topic.at(message_image_left->topic());
      Camera* camera_right = cameras_by_topic.at(message_image_right->topic());

      //ds check if first frame and odometry is available
      if (world_context->currentTrackingContext()->frames().size() == 0 && message_image_left->hasOdom()) {
        world_context->currentTrackingContext()->setRobotToWorldPrevious(message_image_left->odometry().cast<gt_real>()*camera_left->robotToCamera());
        if (use_gui) {
          context_viewer_bird->setViewpointOrigin((message_image_left->odometry().cast<gt_real>()*camera_left->robotToCamera()).inverse());
        }
      }

      //ds trigger SLAM pipeline with all available input TODO purify this function
      process(world_context,
              tracker,
              mapper,
              relocalizer,
              camera_left,
              intensity_image_left,
              camera_right,
              intensity_image_right,
              world_previous_to_current);

      //ds save to txt_io
      if (sensor_message_writer.filename().length() > 0) {
        message_image_left->setOdometry(tracker->robotToWorld().cast<float>());
        sensor_message_writer.writeMessage(*message_image_left);
        message_image_right->setOdometry(tracker->robotToWorld().cast<float>());
        sensor_message_writer.writeMessage(*message_image_right);
      }

      if (message_image_left->hasOdom()) {
        const TransformMatrix3D robot_to_world_ground_truth = message_image_left->odometry().cast<gt_real>()*camera_left->robotToCamera();
        robot_to_world_ground_truth_poses.push_back(robot_to_world_ground_truth);
        tracker->setOdometryRobotToWorld(robot_to_world_ground_truth);
      }

      //ds info
      ++number_of_processed_frames_total;
      ++number_of_processed_frames_current;
      system_usage.update();
      if (number_of_processed_frames_current%100 == 0) {

        //ds compute durations
        const double total_duration_seconds_current = getTime()-time_start_seconds;
        const double total_duration_seconds_total   = getTime()-time_start_seconds_first;

        //ds runtime info
        std::cerr << "-------------------------------------------------------------------------" << std::endl;
        std::cerr << "fps: " << number_of_processed_frames_current/total_duration_seconds_current
                             << " (" << number_of_processed_frames_total << "/" << total_duration_seconds_total << ")" << std::endl;
        std::cerr << "key frames: " << world_context->currentTrackingContext()->keyframes().size()
                                    << " (" << world_context->currentTrackingContext()->keyframes().size()/static_cast<gt_real>(number_of_processed_frames_total) << ")" << std::endl;
        std::cerr << "memory usage (GB): " << static_cast<gt_real>(system_usage.totalMemory())/1e6 << std::endl;
        std::cerr << "runtime relocalizer (s): " << relocalizer->getTimeConsumptionSeconds_overall()
                                                 << " (" << relocalizer->getTimeConsumptionSeconds_overall()/total_duration_seconds_total << ")" << std::endl;
        std::cerr << "runtime      mapper (s): " << mapper->getTimeConsumptionSeconds_overall()
                                                 << " (" << mapper->getTimeConsumptionSeconds_overall()/total_duration_seconds_total << ")" << std::endl;

        //ds reset stats
        time_start_seconds = getTime();
        number_of_processed_frames_current = 0;
      }

      //ds update ui
      synchronizer.reset();
      if (use_gui) {
        running = context_viewer_bird->isVisible() && tracker_viewer->updateGUI();
        context_viewer_bird->updateGL();
        ui_server->processEvents();
      }
    }
  }
  const double total_duration_seconds_total = getTime()-time_start_seconds_first;

  //ds compute squared errors
  TransformMatrix3D odometry_robot_to_world_previous_ground_truth = TransformMatrix3D::Identity();
  Index index_frame     = 0;
  Frame* previous_frame = 0;
  for (FramePtrMapElement frame: world_context->currentTrackingContext()->frames()) {

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
  gt_real root_mean_squared_error_translation_absolute = 0;
  for (const gt_real squared_error: squared_errors_translation_absolute) {
    root_mean_squared_error_translation_absolute += squared_error;
  }
  root_mean_squared_error_translation_absolute /= squared_errors_translation_absolute.size();
  root_mean_squared_error_translation_absolute = std::sqrt(root_mean_squared_error_translation_absolute);
  gt_real mean_error_translation_relative = 0;
  for (const gt_real error: errors_translation_relative) {
    mean_error_translation_relative += error;
  }
  mean_error_translation_relative /= errors_translation_relative.size();

  //ds report
  std::cerr << "-------------------------------------------------------------------------" << std::endl;
  std::cerr << "dataset completed" << std::endl;
  std::cerr << "-------------------------------------------------------------------------" << std::endl;
  if (number_of_processed_frames_total == 0) {
    std::cerr << "no frames processed" << std::endl;
  } else {
    std::cerr << "        absolute translation RMSE: " << root_mean_squared_error_translation_absolute << std::endl;
    std::cerr << "        relative translation   ME: " << mean_error_translation_relative << std::endl;
    std::cerr << "        final translational error: " << (tracker->robotToWorld().translation()-odometry_robot_to_world_previous_ground_truth.translation()).norm() << std::endl;
    std::cerr << "              total stereo frames: " << number_of_processed_frames_total << std::endl;
    std::cerr << "               total duration (s): " << total_duration_seconds_total << std::endl;
    std::cerr << "                      average fps: " << number_of_processed_frames_total/total_duration_seconds_total << std::endl;
    std::cerr << "average processing time (s/frame): " << total_duration_seconds_total/number_of_processed_frames_total << std::endl;
    std::cerr << "average landmarks close per frame: " << tracker->totalNumberOfLandmarksClose()/number_of_processed_frames_total << std::endl;
    std::cerr << "  average landmarks far per frame: " << tracker->totalNumberOfLandmarksFar()/number_of_processed_frames_total << std::endl;
    std::cerr << "         average tracks per frame: " << tracker->totalNumberOfTrackedPoints()/number_of_processed_frames_total << std::endl;
    std::cerr << "        average tracks per second: " << tracker->totalNumberOfTrackedPoints()/total_duration_seconds_total << std::endl;
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
    std::cerr << "run times" << std::endl;
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
    std::cerr << "tracker      feature detection: " << tracker->getTimeConsumptionSeconds_feature_detection()/total_duration_seconds_total
                                                            << " (" << tracker->getTimeConsumptionSeconds_feature_detection() << "s)" << std::endl;
    std::cerr << "              keypoint pruning: " << tracker->getTimeConsumptionSeconds_keypoint_pruning()/total_duration_seconds_total
                                                          << " (" << tracker->getTimeConsumptionSeconds_keypoint_pruning() << "s)" << std::endl;
    std::cerr << "         descriptor extraction: " << tracker->getTimeConsumptionSeconds_descriptor_extraction()/total_duration_seconds_total
                                                          << " (" << tracker->getTimeConsumptionSeconds_descriptor_extraction() << "s)" << std::endl;
    std::cerr << "           point triangulation: " << tracker->getTimeConsumptionSeconds_point_triangulation()/total_duration_seconds_total
                                                          << " (" << tracker->getTimeConsumptionSeconds_point_triangulation() << "s)" << std::endl;
    std::cerr << "                      tracking: " << tracker->getTimeConsumptionSeconds_tracking()/total_duration_seconds_total
                                                          << " (" << tracker->getTimeConsumptionSeconds_tracking() << "s)" << std::endl;
    std::cerr << "                track creation: " << tracker->getTimeConsumptionSeconds_track_creation()/total_duration_seconds_total
                                                          << " (" << tracker->getTimeConsumptionSeconds_track_creation() << "s)" << std::endl;
    std::cerr << "             pose optimization: " << tracker->getTimeConsumptionSeconds_pose_optimization()/total_duration_seconds_total
                                                          << " (" << tracker->getTimeConsumptionSeconds_pose_optimization() << "s)" << std::endl;
    std::cerr << "         landmark optimization: " << tracker->getTimeConsumptionSeconds_landmark_optimization()/total_duration_seconds_total
                                                          << " (" << tracker->getTimeConsumptionSeconds_landmark_optimization() << "s)" << std::endl;
    std::cerr << "                point recovery: " << tracker->getTimeConsumptionSeconds_point_recovery()/total_duration_seconds_total
                                                          << " (" << tracker->getTimeConsumptionSeconds_point_recovery() << "s)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "relocalizer: " << relocalizer->getTimeConsumptionSeconds_overall()/total_duration_seconds_total
                                               << " (" << relocalizer->getTimeConsumptionSeconds_overall() << "s)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "     mapper: " << mapper->getTimeConsumptionSeconds_overall()/total_duration_seconds_total
                                             << " (" << mapper->getTimeConsumptionSeconds_overall() << "s)" << std::endl;
  }
  std::cerr << "-------------------------------------------------------------------------" << std::endl;

  //ds factory cleanup
  AlignerFactory::clear();
  sensor_message_reader.close();

  //ds save trajectory to disk
  world_context->writeTrajectory("trajectory.txt");

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
      delete world_context;
      return 0;
    }
  } else {

    //ds destroy world context
    delete world_context;
    if (use_gui) {
      delete context_viewer_bird;
      delete ui_server;
    }
    return 0;
  }
}

void process(WorldContext* world_context_,
             TrackerSVI* tracker_,
             Mapper* mapper_,
             Relocalizer* relocalizer_,
             const Camera* camera_left_,
             const cv::Mat& intensity_image_left_,
             const Camera* camera_right_,
             const cv::Mat& intensity_image_right_,
             const TransformMatrix3D& world_previous_to_current_estimate_) {

  //ds call the tracker
  world_previous_to_current = tracker_->addImage(camera_left_,
                                                 intensity_image_left_,
                                                 camera_right_,
                                                 intensity_image_right_,
                                                 world_previous_to_current_estimate_);

  //ds if we have a valid frame (not the case after the track is lost)
  if (world_context_->currentTrackingContext()->currentFrame() != 0) {

    //ds if a key frame was generated
    if (world_context_->currentTrackingContext()->generatedKeyframe()) {

      //ds if we have a fresh track (start or lost)
      if (world_context_->currentTrackingContext()->keyframes().size() == 1) {
        relocalizer_->flush();
      }

      //ds trigger relocalization
      relocalizer_->init(world_context_->currentTrackingContext()->currentKeyframe());
      relocalizer_->detect();
      relocalizer_->compute();

      //ds check the closures
      for(CorrespondenceCollection* closure: relocalizer_->closures()) {
        if (closure->is_valid) {
          assert(world_context_->currentTrackingContext()->currentKeyframe() == closure->keyframe_query);

          //ds check if were matching onto another context
          if (world_context_->currentTrackingContext() != closure->keyframe_reference->trackingContext()) {
            world_context_->add(closure);
          } else {
            world_context_->currentTrackingContext()->closeKeyframes(world_context_->currentTrackingContext()->currentKeyframe(),
                                                                   closure->keyframe_reference,
                                                                   closure->transform_frame_query_to_frame_reference);

            if (use_gui) {
              for (const Correspondence* match: closure->correspondences) {
                world_context_->currentTrackingContext()->landmarks().get(match->item_query->landmark()->index())->setIsInLoopClosureQuery(true);
                world_context_->currentTrackingContext()->landmarks().get(match->item_reference->landmark()->index())->setIsInLoopClosureReference(true);
              }
            }
          }
        }
      }
      relocalizer_->train();
    }

    //ds if there were merge requests caused by added closures
    if (world_context_->isMergeable()) {
      //tracker_viewer.switchToStepwiseMode();
      world_context_->merge(use_gui);
    }

    //ds trigger regular tracking context optimization
    if (world_context_->currentTrackingContext()->closedKeyframe()) {
      mapper_->optimize(world_context_->currentTrackingContext());

      //ds wipe non-optimized landmarks
      world_context_->currentTrackingContext()->purifyLandmarks();
    }
  }

  if (use_gui) {
    tracker_viewer->initDrawing();
    tracker_viewer->drawFeatureTracking();
    tracker_viewer->drawFeatures();
  }
}
