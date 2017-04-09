#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include "parameter_server.h"
#include "slam_assembly.h"

//ds lazy globals
proslam::Camera* camera_left  = 0;
proslam::Camera* camera_right = 0;

//ds ground truth handle (set if available)
proslam::TransformMatrix3D ground_truth(proslam::TransformMatrix3D::Identity());

//ds active image buffer handles
cv::Mat image_left;
cv::Mat image_right;
bool found_image_pair = false;

//ds ros synchronization
void callbackCameraInfoLeft(const sensor_msgs::CameraInfoConstPtr& message_) {

  //ds if not set yet
  if (camera_left == 0) {

    //ds obtain eigen formatted data
    const proslam::Matrix3 camera_matrix_transposed(message_->K.elems);
    const proslam::Matrix4_3 projection_matrix_transposed(message_->P.elems);
    const proslam::Matrix3 rectification_matrix_transposed(message_->R.elems);
    const proslam::Vector5 distortion_coefficients(message_->D.data());

    //ds allocate a new camera
    camera_left = new proslam::Camera(message_->height,
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
    const proslam::Matrix3 camera_matrix_transposed(message_->K.elems);
    const proslam::Matrix4_3 projection_matrix_transposed(message_->P.elems);
    const proslam::Matrix3 rectification_matrix_transposed(message_->R.elems);
    const proslam::Vector5 distortion_coefficients(message_->D.data());

    //ds allocate a new camera
    camera_right = new proslam::Camera(message_->height,
                                       message_->width,
                                       camera_matrix_transposed.transpose());
    camera_right->setProjectionMatrix(projection_matrix_transposed.transpose());
    camera_right->setDistortionCoefficients(distortion_coefficients);
    camera_right->setRectificationMatrix(rectification_matrix_transposed.transpose());
  }
}

//ds image retrieval
void callbackStereoImage(const sensor_msgs::ImageConstPtr& image_left_, const sensor_msgs::ImageConstPtr& image_right_){
  found_image_pair = false;

  try {

    //ds obtain cv image pointer for the left image and set it
    cv_bridge::CvImagePtr image_pointer = cv_bridge::toCvCopy(image_left_, sensor_msgs::image_encodings::MONO8);
    image_left = image_pointer->image;
  }
  catch (const cv_bridge::Exception& exception_) {
    std::cerr << "callbackStereoImage|exception: " << exception_.what() << " (image left)" << std::endl;
    return;
  }

  try {

    //ds obtain cv image pointer
    cv_bridge::CvImagePtr image_pointer = cv_bridge::toCvCopy(image_right_, sensor_msgs::image_encodings::MONO8);
    image_right = image_pointer->image;
  }
  catch (const cv_bridge::Exception& exception_) {
    std::cerr << "callbackStereoImage|exception: " << exception_.what() << " (image right)" << std::endl;
    return;
  }

  //ds enable access
  found_image_pair = true;
}

//ds ground truth sources
void callbackGroundTruth(const nav_msgs::OdometryConstPtr& message_) {

  //ds update ground truth: translation
  ground_truth.translation().x() = message_->pose.pose.position.x;
  ground_truth.translation().y() = message_->pose.pose.position.y;
  ground_truth.translation().z() = message_->pose.pose.position.z;

  //ds update ground truth: orientation
  ground_truth.linear() = proslam::Quaternion(message_->pose.pose.orientation.w,
                                              message_->pose.pose.orientation.x,
                                              message_->pose.pose.orientation.y,
                                              message_->pose.pose.orientation.z).toRotationMatrix();
}

//ds don't allow any windoof compilation attempt!
int32_t main(int32_t argc, char ** argv) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(0);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds obtain configuration
  proslam::ParameterServer::parseParametersFromCommandLine(argc, argv);

  //ds check camera info topics - required for the node
  if (proslam::ParameterServer::topicCameraInfoLeft().length() == 0) {
    std::cerr << "ERROR: empty value entered for parameter: -topic-camera-info-left (-cl) (enter -h for help)" << std::endl;
    exit(0);
  }
  if (proslam::ParameterServer::topicCameraInfoRight().length() == 0) {
    std::cerr << "ERROR: empty value entered for parameter: -topic-camera-info-right (-cr) (enter -h for help)" << std::endl;
    exit(0);
  }

  //ds log configuration
  proslam::ParameterServer::printParameters();

  //ds initialize roscpp
  ros::init(argc, argv, "srrg_proslam_node");

  //ds start node
  ros::NodeHandle node;

  //ds subscribe to camera info topics
  ros::Subscriber subscriber_camera_info_left  = node.subscribe(proslam::ParameterServer::topicCameraInfoLeft(), 1, callbackCameraInfoLeft);
  ros::Subscriber subscriber_camera_info_right = node.subscribe(proslam::ParameterServer::topicCameraInfoRight(), 1, callbackCameraInfoRight);

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

  //ds validate if cameras are set (rose node might be interrupted)
  if (camera_left == 0 || camera_right == 0) {
    std::cerr << std::endl;
    std::cerr << "main|caught termination signal, aborting" << std::endl;
    return 0;
  }

  std::cerr << "main|loaded cameras" << std::endl;
  std::cerr << "main|camera left  - resolution: " << camera_left->imageCols() << " x " << camera_left->imageRows() << std::endl;
  std::cerr << "main|camera right - resolution: " << camera_right->imageCols() << " x " << camera_right->imageRows() << std::endl;

  //ds undistortion/rectification maps
  cv::Mat undistort_rectify_maps_left[2];
  cv::Mat undistort_rectify_maps_right[2];

  //ds compute undistorted and rectified mappings
  cv::initUndistortRectifyMap(srrg_core::toCv(camera_left->cameraMatrix()),
                              srrg_core::toCv(camera_left->distortionCoefficients()),
                              srrg_core::toCv(camera_left->rectificationMatrix()),
                              srrg_core::toCv(camera_left->projectionMatrix()),
                              cv::Size(camera_left->imageCols(), camera_left->imageRows()),
                              CV_16SC2,
                              undistort_rectify_maps_left[0],
                              undistort_rectify_maps_left[1]);
  cv::initUndistortRectifyMap(srrg_core::toCv(camera_right->cameraMatrix()),
                              srrg_core::toCv(camera_right->distortionCoefficients()),
                              srrg_core::toCv(camera_right->rectificationMatrix()),
                              srrg_core::toCv(camera_right->projectionMatrix()),
                              cv::Size(camera_right->imageCols(), camera_right->imageRows()),
                              CV_16SC2,
                              undistort_rectify_maps_right[0],
                              undistort_rectify_maps_right[1]);

  //ds unsubscribe
  subscriber_camera_info_left.shutdown();
  subscriber_camera_info_right.shutdown();

  //ds allocate SLAM modules
  proslam::SLAMAssembly slam_system;

  //ds set cameras
  slam_system.loadCameras(camera_left, camera_right);

  //ds configure SLAM modules
  slam_system.tracker()->setPixelDistanceTrackingMinimum(25);
  slam_system.tracker()->setPixelDistanceTrackingMaximum(50);
  slam_system.tracker()->aligner()->setMaximumErrorKernel(25);
  slam_system.tracker()->framepointGenerator()->setTargetNumberOfPoints(500);
  slam_system.tracker()->framepointGenerator()->setMaximumMatchingDistanceTriangulation(50);
  slam_system.tracker()->framepointGenerator()->setMatchingDistanceTrackingThresholdMaximum(50);
  slam_system.tracker()->framepointGenerator()->setMatchingDistanceTrackingThresholdMinimum(50);
  slam_system.relocalizer()->aligner()->setMaximumErrorKernel(0.05);
  slam_system.relocalizer()->aligner()->setMinimumNumberOfInliers(50);
  slam_system.relocalizer()->aligner()->setMinimumInlierRatio(0.5);
  slam_system.relocalizer()->setMinimumNumberOfMatchesPerLandmark(50);

  //ds allocate a qt UI server in the main scope (required)
  QApplication* ui_server = new QApplication(argc, argv);

  //ds initialize gui
  slam_system.initializeGUI(ui_server);
  if (slam_system.viewerInputImages()) slam_system.viewerInputImages()->switchMode();

  //ds set up subscribers
  message_filters::Subscriber<sensor_msgs::Image> subscriber_image_left(node, proslam::ParameterServer::topicImageLeft(), 5);
  message_filters::Subscriber<sensor_msgs::Image> subscriber_image_right(node, proslam::ParameterServer::topicImageRight(), 5);

  //ds define policy and initialize synchronizer
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> StereoImagePolicy;
  message_filters::Synchronizer<StereoImagePolicy> stereo_image_synchronizer(StereoImagePolicy(5), subscriber_image_left, subscriber_image_right);
  stereo_image_synchronizer.registerCallback(boost::bind(&callbackStereoImage, _1, _2));

//  //ds subscribe to ground truth topic if available
//  ros::Subscriber subscriber_ground_truth = node.subscribe("/odom", 1, callbackGroundTruth);

  //ds ground truth transform to robot (current is for the QUT dataset)
  proslam::TransformMatrix3D orientation_correction(proslam::TransformMatrix3D::Identity());
  orientation_correction.matrix() << 0, -1, 0, 0,
                                     0, 0, -1, 0,
                                     1, 0, 0, 0,
                                     0, 0, 0, 1;

  //ds loop control
  proslam::Count number_of_frames_current_window = 0;
  double start_time_current_window_seconds       = srrg_core::getTime();
  const double measurement_interval_seconds      = 5;

  //ds start processing loop
  std::cerr << "main|starting processing loop" << std::endl;
  while (ros::ok() && slam_system.isGUIRunning()) {

    //ds trigger callbacks
    ros::spinOnce();

    //ds if we got a valid stereo image pair (enabled only in callback)
    if (found_image_pair) {

      //ds preprocess the images if desired: rectification
      if (proslam::ParameterServer::optionRectifyAndUndistort()) {
        cv::remap(image_left, image_left, undistort_rectify_maps_left[0], undistort_rectify_maps_left[1], cv::INTER_LINEAR);
        cv::remap(image_right, image_right, undistort_rectify_maps_right[0], undistort_rectify_maps_right[1], cv::INTER_LINEAR);
      }

      //ds preprocess the images if desired: histogram equalization
      if (proslam::ParameterServer::optionEqualizeHistogram()) {
        cv::equalizeHist(image_left, image_left);
        cv::equalizeHist(image_right, image_right);
      }

      //ds process images
      slam_system.process(image_left, image_right);

      //ds add ground truth if available
      slam_system.addGroundTruthMeasurement(orientation_correction*ground_truth);
      ++number_of_frames_current_window;
      found_image_pair = false;
    }

    //ds display stats after each interval
    const double total_duration_seconds_current = srrg_core::getTime()-start_time_current_window_seconds;
    if (total_duration_seconds_current > measurement_interval_seconds) {

      //ds runtime info - depending on set modes
      if (proslam::ParameterServer::optionUseRelocalization()) {
        std::printf("processed frames: %5lu|landmarks: %6lu|local maps: %4lu (%3.2f)|closures: %3lu (%3.2f)|current fps: %5.2f (%3lu/%3.2fs)\n",
                    slam_system.worldMap()->frames().size(),
                    slam_system.worldMap()->landmarks().size(),
                    slam_system.worldMap()->localMaps().size(),
                    slam_system.worldMap()->localMaps().size()/static_cast<proslam::real>(slam_system.worldMap()->frames().size()),
                    slam_system.worldMap()->numberOfClosures(),
                    slam_system.worldMap()->numberOfClosures()/static_cast<proslam::real>(slam_system.worldMap()->localMaps().size()),
                    number_of_frames_current_window/total_duration_seconds_current,
                    number_of_frames_current_window,
                    total_duration_seconds_current);
      } else {
        std::printf("processed frames: %5lu|landmarks: %6lu|current fps: %5.2f (%3lu/%3.2fs)\n",
                    slam_system.worldMap()->frames().size(),
                    slam_system.worldMap()->landmarks().size(),
                    number_of_frames_current_window/total_duration_seconds_current,
                    number_of_frames_current_window,
                    total_duration_seconds_current);
      }

      number_of_frames_current_window   = 0;
      start_time_current_window_seconds = srrg_core::getTime();
    }

    //ds update ui
    slam_system.updateGUI();
  }

  //ds print full report
  slam_system.printReport();

  //ds save trajectory to disk
  slam_system.worldMap()->writeTrajectory("trajectory.txt");

  //ds exit in GUI
  return slam_system.closeGUI(ros::ok());
}
