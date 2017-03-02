#include <cstring>
#include "qapplication.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/base_image_message.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "srrg_txt_io/message_enlister_trigger.h"
#include "srrg_txt_io/static_transform_tree.h"
#include "srrg_txt_io/message_writer.h"

#include "core/gt_tracker.h"
#include "ui/gt_tracker_viewer.h"
#include "ui/gt_tracking_context_viewer.h"
#include "core/gt_mapper.h"
#include "core/gt_relocalizer.h"
#include "ui/gt_viewer_icp.h"

using namespace gslam;
using namespace srrg_core;

const char* banner[] = {
  "srrg_gslam_mapper_app: simple mapper application",
  " it reads sequentially all elements in the file and tracks the camera",
  "",
  "usage: srrg_gslam_mapper_app  [options] [-depth-topic <string>] [-rgb-topic <string>] <dump_file>",
  "options:",
  " -o <file>:  string, sensor message output file of the mapper",
  " -map <file>:  string, serialized map output file of the mapper",
  " -tracker-max-features:  int, max number of features in tracker",
  " -tracker-min-age:  int, number of consecutive frames a feature should be detected before being promoted",
  " -tracker-min-covered-region:  floar, region of the image that needs to be covered with redetected features before a redetection is thrown",
  " -voc <string>:  path to the vocabulary file",
  " -fcorr <float>: focal length correction, default 1",
  " -tf <string>: path to the transform tree",
  0
};

//ds playback modules
static SystemUsageCounter system_usage;
MessageTimestampSynchronizer synchronizer;
MessageWriter sensor_message_writer;
StringCameraMap camera_map;
bool running     = true;
bool camera_only = true;
bool use_gui     = true;
TransformMatrix3D odometry_robot_to_world_previous_ground_truth = TransformMatrix3D::Identity();
TransformMatrix3D world_previous_to_current        = TransformMatrix3D::Identity();
gt_real camera_focal_correction                    = 1;

//ds ui
QApplication* ui_server               = 0;
TrackerViewer* tracker_viewer         = 0;
TrackingContextViewer* context_viewer = 0;

//ds trigger SLAM pipeline
void process(WorldContext* world_context_, Tracker* tracker_, Mapper* mapper_, Relocalizer* relocalizer_);

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
  Tracker* tracker            = new Tracker(world_context);
  Mapper* mapper              = new Mapper();
  Relocalizer* relocalizer    = new Relocalizer();

  //ds obtain configuration
  synchronizer.setTimeInterval(0.001);
  std::string topic_image_depth = "/camera/depth/image_raw"; //"/kinect2/qhd/image_depth_rect";
  std::string topic_image_rgb   = "/camera/rgb/image_raw"; //"/kinect2/qhd/image_color_rect";
  std::string filename_sensor_messages     = "";
  std::string filename_sensor_messages_out = "";
  std::string filename_save_map            = "";
  std::string filename_transform_tree      = "";
  std::string filename_vocabulary          = "/home/dom/datasets/rgbdt/brief_k10L6.voc.gz";
  int count_added_arguments = 1;
  while(count_added_arguments < argc){
    if (! strcmp(argv[count_added_arguments],"-depth-topic")){
      count_added_arguments++;
      topic_image_depth=argv[count_added_arguments];
    } else if (! strcmp(argv[count_added_arguments],"-rgb-topic")){
      count_added_arguments++;
      topic_image_rgb=argv[count_added_arguments];
    } else if (! strcmp(argv[count_added_arguments],"-h")) {
      printBanner(banner);
      return 0;
    } else if (! strcmp(argv[count_added_arguments],"-camera-only")) {
      camera_only = true;
    } else if (! strcmp(argv[count_added_arguments],"-use-gui")) {
      use_gui = true;
    } else if (! strcmp(argv[count_added_arguments],"-tracker-min-image-age")){
      count_added_arguments++;
      tracker->setMinImageAge(atoi(argv[count_added_arguments]));
    } else if (! strcmp(argv[count_added_arguments],"-tracker-min-landmark-age")){
      count_added_arguments++;
      tracker->setMinLandmarkAge(atoi(argv[count_added_arguments]));
    } else if (! strcmp(argv[count_added_arguments],"-tracker-max-depth")){
      count_added_arguments++;
      tracker->setMaxDepth(atof(argv[count_added_arguments]));
    } else if (! strcmp(argv[count_added_arguments],"-tracker-min-feature-distance")){
      count_added_arguments++;
      tracker->setMinFeatureDistance(atof(argv[count_added_arguments]));
    } else if (! strcmp(argv[count_added_arguments],"-fcorr")){
      count_added_arguments++;
      camera_focal_correction=atof(argv[count_added_arguments]);
    } else if (! strcmp(argv[count_added_arguments],"-tf")){
      count_added_arguments++;
      filename_transform_tree=argv[count_added_arguments];
    } else if (! strcmp(argv[count_added_arguments],"-voc")){
      count_added_arguments++;
      filename_vocabulary=argv[count_added_arguments];
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

  //ds load transform overwriter if requested
  StaticTransformTree* transform_tree = 0;
  if(filename_transform_tree.length() > 0) {
    transform_tree = new StaticTransformTree();
    transform_tree->load(filename_transform_tree);
  }

  //ds log configuration
  std::cerr << "running with params: " << endl;
  std::cerr << "-depth-topic                   " << topic_image_depth << std::endl;
  std::cerr << "-rgb-topic                     " << topic_image_rgb << std::endl;
  std::cerr << "-tracker-min-feature-distance  " << tracker->minFeatureDistance() << std::endl;
  std::cerr << "-tracker-min-image-age         " << tracker->minImageAge() << std::endl;
  std::cerr << "-tracker-min-landmark-age      " << tracker->minLandmarkAge() << std::endl;
  std::cerr << "-tracker-max-depth             " << tracker->maxDepth() << std::endl;
  std::cerr << "-voc                           " << filename_vocabulary << std::endl;
  std::cerr << "-fcorr                         " << camera_focal_correction << std::endl;
  std::cerr << "-camera-only                   " << camera_only << std::endl;
  std::cerr << "-messages                      " << filename_sensor_messages << std::endl;
  if (filename_sensor_messages_out.length()) {
    std::cerr << "-o                             " << filename_sensor_messages_out << std::endl;
  }
  if (filename_save_map.length()) {
    std::cerr << "-map                           " << filename_save_map << std::endl;
  }
  if (filename_vocabulary.length()) {
    std::cerr << "-voc                           " << filename_vocabulary << std::endl;
  }
  if (filename_transform_tree.length()) {
    std::cerr << "-tf                            " << filename_transform_tree << std::endl;
  }

  //ds load BoW vocabulary
  //relocalizer.loadVocabulary(vocabulary_filename);
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
  if (filename_save_map.length() > 0){
    world_context->setMapSavefilepath(filename_save_map);
  }

  //ds configure message synchronizer
  std::vector<std::string> topics_synchronized;
  topics_synchronized.push_back(topic_image_rgb);
  topics_synchronized.push_back(topic_image_depth);
  synchronizer.setTopics(topics_synchronized);

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
    context_viewer = new TrackingContextViewer(world_context);
    context_viewer->show();
  }

  //ds start playback
  BaseMessage* base_message = 0;
  while ((base_message = sensor_message_reader.readMessage()) && running) {
    BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(base_message);
    assert(sensor_msg != 0);

    //ds update transforms if present
    if(transform_tree) {
      transform_tree->applyTransform(*sensor_msg);
    }
    sensor_msg->untaint();

    //ds add to synchronizer
    if (sensor_msg->topic() == topic_image_rgb) {
      synchronizer.putMessage(sensor_msg);
    } else if (sensor_msg->topic() == topic_image_depth) {
      synchronizer.putMessage(sensor_msg);
    } else {
      if (sensor_message_writer.filename().length() > 0) {
        sensor_message_writer.writeMessage(*sensor_msg);
        delete sensor_msg;
      }
    }

    //ds if we have a synchronized package of sensor messages ready
    if (synchronizer.messagesReady()) {

      //ds trigger SLAM pipeline
      process(world_context, tracker, mapper, relocalizer);

      //ds info
      ++number_of_processed_frames_current;
      if (number_of_processed_frames_current%100 == 0) {

        //ds update count
        number_of_processed_frames_total += number_of_processed_frames_current;

        //ds compute durations
        const double total_duration_seconds_current = getTime()-time_start_seconds;
        const double total_duration_seconds_total   = getTime()-time_start_seconds_first;

        //ds info
        system_usage.update();
        LOG_INFO("Tracker::addImage", "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
        std::cerr << "Tracker::addImage|fps: " << number_of_processed_frames_current/total_duration_seconds_current
                                               << " (" << number_of_processed_frames_total << "/" << total_duration_seconds_total << ")" << std::endl;
        std::cerr << "Tracker::addImage|key frames: " << world_context->currentTrackingContext()->keyframes().size()
                                                      << " (" << world_context->currentTrackingContext()->keyframes().size()/static_cast<gt_real>(number_of_processed_frames_total) << ")" << std::endl;
        std::cerr << "Tracker::addImage|runtime     tracker: " << tracker->getTimeConsumptionSeconds_overall()
                                                               << " (" << tracker->getTimeConsumptionSeconds_overall()/total_duration_seconds_total << ")" << std::endl;
        std::cerr << "Tracker::addImage|runtime relocalizer: " << relocalizer->getTimeConsumptionSeconds_overall()
                                                               << " (" << relocalizer->getTimeConsumptionSeconds_overall()/total_duration_seconds_total << ")" << std::endl;
        std::cerr << "Tracker::addImage|runtime      mapper: " << mapper->getTimeConsumptionSeconds_overall()
                                                               << " (" << mapper->getTimeConsumptionSeconds_overall()/total_duration_seconds_total << ")" << std::endl;

        //ds reset stats
        time_start_seconds = getTime();
        number_of_processed_frames_current = 0;
      }

      //ds update ui
      synchronizer.reset();
      if (use_gui) {
        running = tracker_viewer->updateGUI();
        context_viewer->updateGL();
        ui_server->processEvents();
      }

//      if (number_of_processed_frames_total == 10000) {
//        running = false;
//      }
    }
  }

  //ds factory cleanup
  AlignerFactory::clear();
  sensor_message_writer.close();
  sensor_message_reader.close();

  //ds save accumulated maps to disk if desired
  if (filename_save_map.length() > 0) {
    world_context->write();
  }

  //ds controlled destruction: SLAM modules
  delete tracker;
  delete mapper;
  delete relocalizer;

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

void process(WorldContext* world_context_, Tracker* tracker_, Mapper* mapper_, Relocalizer* relocalizer_) {
  assert(synchronizer.messagesReady());

  //ds buffer sensor data
  PinholeImageMessage* image_rgb   = dynamic_cast<PinholeImageMessage*>(synchronizer.messages()[0].get());
  PinholeImageMessage* image_depth = dynamic_cast<PinholeImageMessage*>(synchronizer.messages()[1].get());
  image_rgb->untaint();
  image_depth->untaint();

  //ds camera for both (TODO allow mixtures?) images
  Camera* camera = 0;

  //ds attempt to find the camera in the history
  StringCameraMap::iterator camera_map_element = camera_map.find(image_rgb->topic());
  if (camera_map_element == camera_map.end()) {

    //ds update camera matrix with given focal correction
    Matrix3 polished_camera_matrix = image_rgb->cameraMatrix().cast<gt_real>();
    polished_camera_matrix(0,0)   *= camera_focal_correction;
    polished_camera_matrix(1,1)   *= camera_focal_correction;

    //ds allocate a new camera object
    camera = new Camera(image_rgb->image().rows,
                        image_rgb->image().cols,
                        polished_camera_matrix,
                        image_rgb->offset().cast<gt_real>());

    //ds check if camera only mode is desired
    if (camera_only) {
      camera->setOffset(TransformMatrix3D::Identity());
    }

    camera_map.insert(std::make_pair(image_rgb->topic(), camera));
  } else {
    camera = camera_map_element->second;
  }

  //ds if theres odometry data available
  if (image_rgb->hasOdom() && !camera_only) {
    world_previous_to_current        = image_rgb->odometry().cast<gt_real>()*odometry_robot_to_world_previous_ground_truth.inverse();
    odometry_robot_to_world_previous_ground_truth = image_rgb->odometry().cast<gt_real>();
  } else {
    odometry_robot_to_world_previous_ground_truth = world_previous_to_current*odometry_robot_to_world_previous_ground_truth;
    image_rgb->setOffset(TransformMatrix3D::Identity().cast<float>());
    image_depth->setOffset(TransformMatrix3D::Identity().cast<float>());
  }

  //ds convert rgb to grayscale
  cv::Mat intensity_image;
  cvtColor(image_rgb->image(), intensity_image, CV_RGB2GRAY);

  //ds call the tracker
  world_previous_to_current = tracker_->addImage(camera,
                                                 intensity_image,
                                                 image_depth->image(),
                                                 image_rgb->seq(),
                                                 world_previous_to_current);

  //ds if a key frame was generated
  if (world_context_->currentTrackingContext()->generatedKeyframe()) {

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
          //ViewerICP viewer(closure);
          //app->exec();
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

  if (use_gui) {
    tracker_viewer->initDrawing();
    tracker_viewer->drawFeatures();
    //tracker_viewer.drawFeatureTracking();
  }

  //ds update txt_io
  if (sensor_message_writer.filename().length() > 0) {
    image_rgb->setOdometry(tracker_->getRobotToWorld().cast<float>());
    sensor_message_writer.writeMessage(*image_rgb);
    image_depth->setOdometry(tracker_->getRobotToWorld().cast<float>());
    sensor_message_writer.writeMessage(*image_depth);
  }
}
