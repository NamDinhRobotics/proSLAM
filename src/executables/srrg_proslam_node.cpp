#include "qapplication.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

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
  "-save-memory:                 enforce deallocation of unused memory at runtime",
  "-equalize-histogram           equalize stereo image histogram before processing",
  "-------------------------------------------------------------------------",
  0
};

//ds playback modules - lazy globals
static SystemUsageCounter system_usage;
bool use_gui                                = false;
bool use_relocalization                     = true;
TransformMatrix3D world_previous_to_current = TransformMatrix3D::Identity();
std::string topic_camera_image_left  = "/wide_stereo/left/image_raw";
std::string topic_camera_image_right = "/wide_stereo/right/image_raw";
std::string topic_camera_info_left   = "/wide_stereo/left/camera_info";
std::string topic_camera_info_right  = "/wide_stereo/right/camera_info";
Camera* camera_left  = 0;
Camera* camera_right = 0;

//ds image handling globals
cv::Mat undistort_rectify_maps_left[2];
cv::Mat undistort_rectify_maps_right[2];
std::pair<double, cv::Mat> image_left;
std::pair<double, cv::Mat> image_right;
bool is_set_image_left = false;
bool is_set_image_right = false;

//ds user interface related
QApplication* ui_server              = 0;
ViewerInputImages* tracker_viewer    = 0;
ViewerOutputMap* context_viewer_bird = 0;

//ds process a pair of rectified and undistorted stereo images
void process(WorldMap* world_map_,
             Tracker* tracker_,
             GraphOptimizer* mapper_,
             Relocalizer* relocalizer_,
             const cv::Mat& intensity_image_left_,
             const cv::Mat& intensity_image_right_);

//ds ros synchronization
void callbackCameraInfoLeft(const sensor_msgs::CameraInfoConstPtr& message_) {

  //ds if not set yet
  if (camera_left == 0) {

    //ds obtain eigen formatted data
    const Matrix3 camera_matrix_transposed(message_->K.elems);
    const Matrix4_3 projection_matrix_transposed(message_->P.elems);
    const Matrix3 rectification_matrix_transposed(message_->R.elems);
    const Vector5 distortion_coefficients(message_->D.data());

    //ds allocate a new camera
    camera_left = new Camera(message_->height,
                             message_->width,
                             camera_matrix_transposed.transpose());
    camera_left->setProjectionMatrix(projection_matrix_transposed.transpose());
    camera_left->setDistortionCoefficients(distortion_coefficients);
    camera_left->setRectificationMatrix(rectification_matrix_transposed.transpose());
  }
}
void callbackCameraInfoRight(const sensor_msgs::CameraInfoConstPtr& message_) {

  //ds if not set yet
  if (camera_right == 0) {

    //ds obtain eigen formatted data
    const Matrix3 camera_matrix_transposed(message_->K.elems);
    const Matrix4_3 projection_matrix_transposed(message_->P.elems);
    const Matrix3 rectification_matrix_transposed(message_->R.elems);
    const Vector5 distortion_coefficients(message_->D.data());

    //ds allocate a new camera
    camera_right = new Camera(message_->height,
                              message_->width,
                              camera_matrix_transposed.transpose());
    camera_right->setProjectionMatrix(projection_matrix_transposed.transpose());
    camera_right->setDistortionCoefficients(distortion_coefficients);
    camera_right->setRectificationMatrix(rectification_matrix_transposed.transpose());
  }
}

//ds image handling
void callbackImageLeft(const sensor_msgs::ImageConstPtr& message_) {
  try {

    //ds obtain cv image pointer
    cv_bridge::CvImagePtr image_pointer = cv_bridge::toCvCopy(message_, sensor_msgs::image_encodings::MONO8);

    //ds rectify image
    cv::remap(image_pointer->image, image_left.second, undistort_rectify_maps_left[0], undistort_rectify_maps_left[1], cv::INTER_LINEAR);

    //ds set timestmap
    image_left.first = image_pointer->header.stamp.toSec();

    //ds set image
    is_set_image_left = true;
  }
  catch (cv_bridge::Exception& exception_) {
    std::cerr << "callbackImageLeft|exception: " << exception_.what() << std::endl;
    return;
  }
}
void callbackImageRight(const sensor_msgs::ImageConstPtr& message_) {
  try {

    //ds obtain cv image pointer
    cv_bridge::CvImagePtr image_pointer = cv_bridge::toCvCopy(message_, sensor_msgs::image_encodings::MONO8);

    //ds rectify image
    cv::remap(image_pointer->image, image_right.second, undistort_rectify_maps_right[0], undistort_rectify_maps_right[1], cv::INTER_LINEAR);

    //ds set timestmap
    image_right.first = image_pointer->header.stamp.toSec();

    //ds set image
    is_set_image_right = true;
  }
  catch (cv_bridge::Exception& exception_) {
    std::cerr << "callbackImageRight|exception: " << exception_.what() << std::endl;
    return;
  }
}

//ds don't allow any windoof compilation attempt!
int32_t main(int32_t argc, char ** argv) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(0);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds obtain configuration
  int32_t count_added_arguments = 1;
  bool show_top_viewer          = false;
  bool equalize_histogram       = false;
  while(count_added_arguments < argc){
    if (!std::strcmp(argv[count_added_arguments], "-camera-left-image-topic")){
      count_added_arguments++;
      topic_camera_image_left = argv[count_added_arguments];
    } else if (!std::strcmp(argv[count_added_arguments], "-camera-right-image-topic")){
      count_added_arguments++;
      topic_camera_image_right = argv[count_added_arguments];
    } else if (!std::strcmp(argv[count_added_arguments], "-h")) {
      printBanner(banner);
      return 0;
    } else if (!std::strcmp(argv[count_added_arguments], "-use-gui")) {
      use_gui = true;
    } else if (!std::strcmp(argv[count_added_arguments], "-open")) {
      use_relocalization = false;
    } else if (!std::strcmp(argv[count_added_arguments], "-show-top")) {
      show_top_viewer = true;
    } else if (!std::strcmp(argv[count_added_arguments], "-equalize-histogram")) {
      equalize_histogram = true;
    }
    count_added_arguments++;
  }

  //ds log configuration
  std::cerr << "main|-------------------------------------------------------------------------" << std::endl;
  std::cerr << "main|running with params: " << std::endl;
  std::cerr << "main|-camera-left-topic  " << topic_camera_image_left << std::endl;
  std::cerr << "main|-camera-right-topic " << topic_camera_image_right << std::endl;
  std::cerr << "main|-use-gui            " << use_gui << std::endl;
  std::cerr << "main|-open               " << !use_relocalization << std::endl;
  std::cerr << "main|-show-top           " << show_top_viewer << std::endl;
  std::cerr << "main|-equalize-histogram " << equalize_histogram << std::endl;
  std::cerr << "main|-------------------------------------------------------------------------" << std::endl;

  //ds initialize roscpp
  ros::init(argc, argv, "srrg_proslam_node");

  //ds start node
  ros::NodeHandle node;

  //ds subscribe to camera info topics
  ros::Subscriber subscriber_camera_info_left  = node.subscribe(topic_camera_info_left, 1, callbackCameraInfoLeft);
  ros::Subscriber subscriber_camera_info_right = node.subscribe(topic_camera_info_right, 1, callbackCameraInfoRight);

  //ds buffer camera info
  std::cerr << "main|acquiring stereo camera configuration from ROS topics" << std::endl;
  while (ros::ok()) {

    //ds trigger callbacks
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    //ds as soon as both cameras are set
    if (camera_left != 0 && camera_right != 0) {
      break;
    }
  }
  std::cerr << "main|loaded cameras" << std::endl;
  std::cerr << "main|camera left  - resolution: " << camera_left->imageCols() << " x " << camera_left->imageRows() << std::endl;
  std::cerr << "main|camera right - resolution: " << camera_right->imageCols() << " x " << camera_right->imageRows() << std::endl;

  //ds compute undistorted and rectified mappings
  cv::initUndistortRectifyMap(toCv(camera_left->cameraMatrix()),
                              toCv(camera_left->distortionCoefficients()),
                              toCv(camera_left->rectificationMatrix()),
                              toCv(camera_left->projectionMatrix()),
                              cv::Size(camera_left->imageCols(), camera_left->imageRows()),
                              CV_16SC2,
                              undistort_rectify_maps_left[0],
                              undistort_rectify_maps_left[1]);
  cv::initUndistortRectifyMap(toCv(camera_right->cameraMatrix()),
                              toCv(camera_right->distortionCoefficients()),
                              toCv(camera_right->rectificationMatrix()),
                              toCv(camera_right->projectionMatrix()),
                              cv::Size(camera_right->imageCols(), camera_right->imageRows()),
                              CV_16SC2,
                              undistort_rectify_maps_right[0],
                              undistort_rectify_maps_right[1]);

  //ds unsubscribe
  subscriber_camera_info_left.shutdown();
  subscriber_camera_info_right.shutdown();

  //ds allocate SLAM modules
  WorldMap* world_map      = new WorldMap();
  GraphOptimizer* mapper   = new GraphOptimizer();
  Relocalizer* relocalizer = new Relocalizer();
  Tracker* tracker         = new Tracker(camera_left, camera_right);

  //ds configure SLAM modules
  tracker->setPixelDistanceTrackingMinimum(50);
  tracker->setPixelDistanceTrackingMaximum(50);
  tracker->aligner()->setMaximumErrorKernel(25);
  tracker->preprocessor()->setTargetNumberOfPoints(500);
  relocalizer->aligner()->setMaximumErrorKernel(0.5);
  relocalizer->aligner()->setMinimumNumberOfInliers(25);
  relocalizer->aligner()->setMinimumInlierRatio(0.5);
  relocalizer->setMinimumAbsoluteNumberOfMatchesPointwise(25);

  //ds initialize gui
  if (use_gui) {
    ui_server      = new QApplication(argc, argv);
    tracker_viewer = new ViewerInputImages(world_map);
    tracker_viewer->switchMode();
    context_viewer_bird = new ViewerOutputMap(world_map, 0.1, "output: map (bird view)");
    context_viewer_bird->show();

    //ds orientation flip for proper camera following
    TransformMatrix3D orientation_correction;
    orientation_correction.matrix() << 0, -1, 0, 0,
                                       -1, 0, 0, 0,
                                       0, 0, -1, 0,
                                       0, 0, 0, 1;
    context_viewer_bird->setRotationRobotView(orientation_correction);
  }

  //ds subscribe to camera image topics
  ros::Subscriber subscriber_camera_image_left  = node.subscribe(topic_camera_image_left, 1, callbackImageLeft);
  ros::Subscriber subscriber_camera_image_right = node.subscribe(topic_camera_image_right, 1, callbackImageRight);

  //ds start processing loop
  std::cerr << "main|starting processing loop" << std::endl;
  bool is_running = true;
  while (ros::ok() && is_running) {

    //ds trigger callbacks
    ros::spinOnce();

    //ds if we got a valid stereo image pair - brutal without buffers
    if (is_set_image_left && is_set_image_right) {

      //ds preprocess the images if desired
      if (equalize_histogram) {
        cv::equalizeHist(image_left.second, image_left.second);
        cv::equalizeHist(image_right.second, image_right.second);
      }

      //ds check large time delta (more than 100ms -> 10Hz)
      if (std::fabs(image_left.first-image_right.first) > 0.1) {
        std::cerr << "main|WARNING: received large timestamp delta between stereo images" << std::endl;
      }

      //ds process images
      process(world_map,
              tracker,
              mapper,
              relocalizer,
              image_left.second,
              image_right.second);

      //ds update ui
      if (use_gui) {
        if (mapper->numberOfOptimizations() > 0) {
          context_viewer_bird->setIsOpen(false);
        }
        tracker_viewer->initDrawing();
        tracker_viewer->drawFeatureTracking();
        tracker_viewer->drawFeatures();
        is_running = context_viewer_bird->isVisible() && tracker_viewer->updateGUI();
        context_viewer_bird->updateGL();
        ui_server->processEvents();
      }

      //ds reset images
      is_set_image_left = false;
      is_set_image_right = false;
    }
  }

  //ds stop node
  return 0;
}

//ds process a pair of rectified and undistorted stereo images
void process(WorldMap* world_map_,
             Tracker* tracker_,
             GraphOptimizer* mapper_,
             Relocalizer* relocalizer_,
             const cv::Mat& intensity_image_left_,
             const cv::Mat& intensity_image_right_) {

  //ds call the tracker
  tracker_->compute(world_map_, intensity_image_left_, intensity_image_right_);

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
            world_map_->addLoopClosure(world_map_->currentLocalMap(),
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
