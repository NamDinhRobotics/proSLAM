#include "qapplication.h"

#include <ros/ros.h>
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
  "srrg_proslam_app: simple SLAM application",
  "",
  "usage: srrg_proslam_app [options] <message_file> | <calibration_file>",
  "<message_file>: path to a SRRG txt_io message file",
  "<calibration_file>: path to a calibration file containing raw input information (see calibration_example.txt) TBD",
  "[options]:",
  "-camera-left-topic <string>",
  "-camera-right-topic <string>",
  "-use-gui:  displays GUI elements",
  "-open: disables relocalization",
  0
};

//ds playback modules - lazy globals
static SystemUsageCounter system_usage;
bool use_gui                                = false;
bool use_relocalization                     = true;
TransformMatrix3D world_previous_to_current = TransformMatrix3D::Identity();
std::string topic_image_left        = "/wide_stereo/left/image_raw";
std::string topic_image_right       = "/wide_stereo/right/image_raw";
std::string topic_camera_info_left  = "/wide_stereo/left/camera_info";
std::string topic_camera_info_right = "/wide_stereo/right/camera_info";
Camera* camera_left  = 0;
Camera* camera_right = 0;

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

void callbackImageLeft(const sensor_msgs::ImageConstPtr& message_) {

}

void callbackImageRight(const sensor_msgs::ImageConstPtr& message_) {

}

//ds don't allow any windoof compilation attempt!
int32_t main(int32_t argc, char ** argv) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(0);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds obtain configuration
  int32_t count_added_arguments = 1;
  while(count_added_arguments < argc){
    if (!std::strcmp(argv[count_added_arguments], "-camera-left-topic")){
      count_added_arguments++;
      topic_image_left = argv[count_added_arguments];
    } else if (!std::strcmp(argv[count_added_arguments], "-camera-right-topic")){
      count_added_arguments++;
      topic_image_right = argv[count_added_arguments];
    } else if (!std::strcmp(argv[count_added_arguments], "-h")) {
      printBanner(banner);
      return 0;
    } else if (!std::strcmp(argv[count_added_arguments], "-use-gui")) {
      use_gui = true;
    } else if (!std::strcmp(argv[count_added_arguments], "-open")) {
      use_relocalization = false;
    }
    count_added_arguments++;
  }

  //ds log configuration
  std::cerr << "main|running with params:      " << std::endl;
  std::cerr << "main|-camera-left-info-topic   " << topic_camera_info_left << std::endl;
  std::cerr << "main|-camera-left-image-topic  " << topic_image_left << std::endl;
  std::cerr << "main|-camera-right-info-topic  " << topic_camera_info_right << std::endl;
  std::cerr << "main|-camera-right-image-topic " << topic_image_right << std::endl;
  std::cerr << "main|-use-gui                  " << use_gui << std::endl;
  std::cerr << "main|-open                     " << !use_relocalization << std::endl;

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

  //ds compute undistortion and rectification maps
  cv::Mat undistort_rectify_maps_left[2];
  cv::Mat undistort_rectify_maps_right[2];

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

  //ds stop node
  return 0;
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
