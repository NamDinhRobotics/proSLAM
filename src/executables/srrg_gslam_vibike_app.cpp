#include <cstring>
#include "qapplication.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_writer.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/pinhole_image_message.h"

#include "ui/gt_tracker_viewer.h"
#include "ui/gt_tracking_context_viewer.h"
#include "core/gt_mapper.h"
#include "core/gt_relocalizer.h"
#include "core/tracker_svi.h"
#include "ui/gt_viewer_icp.h"

using namespace gslam;
using namespace srrg_core;

//ds help banner
const char* banner[] = {
  "srrg_gslam_sv_mapper_app: simple mapper application",
  " it reads sequentially all elements in the file and tracks the camera",
  "",
  "usage: srrg_gslam_mapper_app <dump_file>",
  "options: TBD",
  0
};

//ds playback modules
static SystemUsageCounter system_usage;
MessageTimestampSynchronizer synchronizer;
MessageWriter sensor_message_writer;
bool running     = true;
bool camera_only = true;
bool use_gui     = false;
TransformMatrix3D odometry_robot_to_world_previous_ground_truth = TransformMatrix3D::Identity();
TransformMatrix3D world_previous_to_current = TransformMatrix3D::Identity();
StringCameraMap cameras_by_topic;

//ds ui
QApplication* ui_server               = 0;
TrackerViewer* tracker_viewer         = 0;
TrackingContextViewer* context_viewer = 0;

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

  //ds assertion activity validation
  //assert(false);

  //ds lock opencv to use only 1 thread
  cv::setNumThreads(1);

  //ds enable maximally deterministic mode
  cv::setUseOptimized(false);

  //ds all SLAM modules TODO proper generation
  WorldContext* world_context = new WorldContext();
  world_context->aligner()->setMaximumErrorKernel(0.5);
  world_context->aligner()->setMinimumInlierRatio(0.5);
  Mapper* mapper           = new Mapper();
  Relocalizer* relocalizer = new Relocalizer();
  relocalizer->aligner()->setMaximumErrorKernel(0.5);
  relocalizer->aligner()->setMinimumNumberOfInliers(50);
  relocalizer->aligner()->setMinimumInlierRatio(0.5);
  relocalizer->setMinimumAbsoluteNumberOfMatchesPointwise(50);

  //ds obtain configuration
  synchronizer.setTimeInterval(0.001);
//  std::string topic_image_stereo_left      = "/camera_left/image_raw";
//  std::string topic_image_stereo_right     = "/camera_right/image_raw";
  std::string topic_image_stereo_left      = "/thin_visensor_node/camera_left/image_raw";
  std::string topic_image_stereo_right     = "/thin_visensor_node/camera_right/image_raw";
  std::string filename_sensor_messages     = "";
  std::string filename_sensor_messages_out = "";
  std::string filename_save_map            = "";
  int count_added_arguments = 1;
  while(count_added_arguments < argc){
    if (! strcmp(argv[count_added_arguments],"-h")) {
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
  std::cerr << "running with params: " << endl;
  std::cerr << "-topic-name-camera-left (-cl)  " << topic_image_stereo_left << std::endl;
  std::cerr << "-topic-name-camera-right (-cr) " << topic_image_stereo_right << std::endl;
  std::cerr << "-camera-only                   " << camera_only << std::endl;
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
                                       message_image_left->offset().cast<gt_real>(),
                                       false);
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
  tracker->setMaximumPixelDistanceTrackingMinimum(25);
  tracker->gridSensor()->setTargetNumberOfPoints(250);
  tracker->gridSensor()->setDetectorThreshold(25);
  tracker->gridSensor()->setDetectorThresholdMaximum(100);
  tracker->gridSensor()->setMaximumMatchingDistanceTriangulation(100);
  tracker->aligner()->setMaximumErrorKernel(9);
  tracker->aligner()->setDamping(1);

  //ds frame counts
  Count number_of_processed_frames_total   = 0;
  Count number_of_processed_frames_current = 0;

  //ds store start time
  double time_start_seconds             = getTime();
  const double time_start_seconds_first = time_start_seconds;

  //ds initialize gui
  if (use_gui) {
    ui_server      = new QApplication(argc, argv);
    tracker_viewer = new TrackerViewer(world_context);
    context_viewer = new TrackingContextViewer(world_context, 0.1);
//    aligner_viewer = new AlignerViewer();
    context_viewer->show();
//    aligner_viewer->show();
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
      Camera* camera_right = cameras_by_topic.at(message_image_right->topic());

      //ds if there's external odometry information available - otherwise constant velocity motion model is used
      if (!camera_only && message_image_left->hasOdom()) {
        world_previous_to_current = message_image_left->odometry().cast<gt_real>()*odometry_robot_to_world_previous_ground_truth.inverse();
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
        tracker->setOdometryRobotToWorld(message_image_left->odometry().cast<gt_real>());
        odometry_robot_to_world_previous_ground_truth = message_image_left->odometry().cast<gt_real>();
      } else {
        odometry_robot_to_world_previous_ground_truth = tracker->robotToWorld();
      }

      //ds info
      ++number_of_processed_frames_total;
      ++number_of_processed_frames_current;
      system_usage.update();
      if (number_of_processed_frames_current%100 == 0) {

        //ds compute durations
        const double total_duration_seconds_current = getTime()-time_start_seconds;
        const double total_duration_seconds_total   = getTime()-time_start_seconds_first;

        //ds info
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
        cv::imshow("TrackerDebug", tracker->imageDisplayDebug());
        running = tracker_viewer->updateGUI();
        context_viewer->updateGL();
        ui_server->processEvents();
      }
    }
  }
  const double total_duration_seconds_total = getTime()-time_start_seconds_first;

  //ds report
  std::cerr << "-------------------------------------------------------------------------" << std::endl;
  std::cerr << "dataset completed" << std::endl;
  std::cerr << "-------------------------------------------------------------------------" << std::endl;
  if (number_of_processed_frames_total == 0) {
    std::cerr << "no frames processed" << std::endl;
  } else {
    std::cerr << "        final translational error: " << (tracker->robotToWorld().translation()-odometry_robot_to_world_previous_ground_truth.translation()).norm() << std::endl;
    std::cerr << "              total stereo frames: " << number_of_processed_frames_total << std::endl;
    std::cerr << "               total duration (s): " << total_duration_seconds_total << std::endl;
    std::cerr << "                      average fps: " << number_of_processed_frames_total/total_duration_seconds_total << std::endl;
    std::cerr << "average processing time (s/frame): " << total_duration_seconds_total/number_of_processed_frames_total << std::endl;
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
  world_context->writeTrajectory();

  //ds save accumulated maps to disk if desired
  if (filename_save_map.length() > 0) {
    world_context->write();
  }

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
    if (use_gui && context_viewer->isVisible()) {
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
      delete context_viewer;
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
    tracker_viewer->drawFeatures();
  }
}
