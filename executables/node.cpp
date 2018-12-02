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

#include "types/parameters.h"
#include "system/slam_assembly.h"

//ds lazy and ugly, ugly globals
proslam::Camera* camera_left;
proslam::Camera* camera_right;

//ds ground truth handle (set if available)
proslam::TransformMatrix3D ground_truth(proslam::TransformMatrix3D::Identity());

//ds active image buffer handles
cv::Mat image_left;
cv::Mat image_right;
bool found_image_pair = false;



void callbackCameraInfoLeft(const sensor_msgs::CameraInfoConstPtr& message_) {

  //ds if not set yet
  if (camera_left == 0) {

    //ds obtain eigen formatted data
    const proslam::Matrix3 camera_matrix_transposed(message_->K.elems);
    const proslam::Matrix4_3 projection_matrix_transposed(message_->P.elems);
    const proslam::Matrix3 rectification_matrix_transposed(message_->R.elems);
    const proslam::Vector5 distortion_coefficients(message_->D.data());

    //ds allocate a new camera
    camera_left = new proslam::Camera(message_->height, message_->width, camera_matrix_transposed.transpose());
    camera_left->setProjectionMatrix(projection_matrix_transposed.transpose());
    camera_left->setDistortionCoefficients(distortion_coefficients);
    camera_left->setRectificationMatrix(rectification_matrix_transposed.transpose());
    std::cerr << BAR << std::endl;
    std::cerr << "LEFT camera configuration" << std::endl;
    camera_left->writeConfiguration(std::cerr);
    std::cerr << BAR << std::endl;
  }
}

void callbackCameraInfoRight(const sensor_msgs::CameraInfoConstPtr& message_) {

  //ds if not set yet
  if (camera_right == 0) {

    //ds obtain eigen formatted data
    const proslam::Matrix3 camera_matrix_transposed(message_->K.elems);
    proslam::Matrix4_3 projection_matrix_transposed(message_->P.elems);
    const proslam::Matrix3 rectification_matrix_transposed(message_->R.elems);
    const proslam::Vector5 distortion_coefficients(message_->D.data());

    //ds check if camera is upside-down
    if(projection_matrix_transposed(3,0) > 0) {
      std::cerr << "WARNING: detected upside-down stereo camera configuration - check if this is desired" << std::endl;
      projection_matrix_transposed(3,0) = -projection_matrix_transposed(3,0);
    }

    //ds allocate a new camera
    camera_right = new proslam::Camera(message_->height, message_->width, camera_matrix_transposed.transpose());
    camera_right->setProjectionMatrix(projection_matrix_transposed.transpose());
    camera_right->setBaselineHomogeneous(camera_right->projectionMatrix().col(3));
    camera_right->setDistortionCoefficients(distortion_coefficients);
    camera_right->setRectificationMatrix(rectification_matrix_transposed.transpose());
    std::cerr << BAR << std::endl;
    std::cerr << "RIGHT camera configuration" << std::endl;
    camera_right->writeConfiguration(std::cerr);
    std::cerr << BAR << std::endl;
  }
}

//ds stereo image acquisition
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

//mc monocular camera and depth image acquisition
void callbackDepthImage(const sensor_msgs::ImageConstPtr& image_left_, const sensor_msgs::ImageConstPtr& image_right_) {
  found_image_pair = false;

  try {
    //ds obtain cv image pointer for the left image and set it
    cv_bridge::CvImagePtr image_pointer = cv_bridge::toCvCopy(image_left_, sensor_msgs::image_encodings::MONO8);
    image_left = image_pointer->image;
  }
  catch (const cv_bridge::Exception& exception_) {
    std::cerr << "callbackDepthImage|exception: " << exception_.what() << " (image left)" << std::endl;
    return;
  }

  try {
    //ds obtain cv image pointer
    cv_bridge::CvImagePtr image_pointer = cv_bridge::toCvCopy(image_right_, sensor_msgs::image_encodings::TYPE_32FC1);
    image_pointer->image.convertTo(image_right, CV_16UC1);
  }
  catch (const cv_bridge::Exception& exception_) {
    std::cerr << "callbackDepthImage|exception: " << exception_.what() << " (image right)" << std::endl;
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
int32_t main(int32_t argc_, char** argv_) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(0);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds allocate the complete parameter collection with default values
  proslam::ParameterCollection* parameters = new proslam::ParameterCollection(proslam::LoggingLevel::Debug);

  try {

    //ds obtain configuration
    parameters->parseFromCommandLine(argc_, argv_);
  } catch (const std::runtime_error& exception_) {
    std::cerr << "main|caught exception '" << exception_.what() << "'" << std::endl;
    delete parameters;
    return 0;
  }

  //ds check camera info topics - required for the node
  if (parameters->command_line_parameters->topic_camera_info_left.length() == 0) {
    std::cerr << "main|empty value entered for parameter: -topic-camera-info-left (-cl) (enter -h for help)" << std::endl;
    delete parameters;
    return 0;
  }
  if (parameters->command_line_parameters->topic_camera_info_right.length() == 0) {
    std::cerr << "main|empty value entered for parameter: -topic-camera-info-right (-cr) (enter -h for help)" << std::endl;
    delete parameters;
    return 0;
  }

  //ds log configuration
  parameters->command_line_parameters->print();

  //ds initialize roscpp
  ros::init(argc_, argv_, "srrg_proslam_node");

  //ds start node
  ros::NodeHandle node;

  //ds subscribe to camera info topics
  ros::Subscriber subscriber_camera_info_left  = node.subscribe(parameters->command_line_parameters->topic_camera_info_left, 1, callbackCameraInfoLeft);
  ros::Subscriber subscriber_camera_info_right = node.subscribe(parameters->command_line_parameters->topic_camera_info_right, 1, callbackCameraInfoRight);

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
    std::cerr << "main|cameras not set" << std::endl;
    delete parameters;
    return 0;
  }

  //ds undistortion/rectification maps
  cv::Mat undistort_rectify_maps_left[2];
  cv::Mat undistort_rectify_maps_right[2];

  //ds always compute undistorted and rectified mappings
  cv::initUndistortRectifyMap(srrg_core::toCv(camera_left->cameraMatrix()),
                              srrg_core::toCv(camera_left->distortionCoefficients()),
                              srrg_core::toCv(camera_left->rectificationMatrix()),
                              srrg_core::toCv(camera_left->projectionMatrix()),
                              cv::Size(camera_left->numberOfImageCols(), camera_left->numberOfImageRows()),
                              CV_16SC2,
                              undistort_rectify_maps_left[0],
                              undistort_rectify_maps_left[1]);
  cv::initUndistortRectifyMap(srrg_core::toCv(camera_right->cameraMatrix()),
                              srrg_core::toCv(camera_right->distortionCoefficients()),
                              srrg_core::toCv(camera_right->rectificationMatrix()),
                              srrg_core::toCv(camera_right->projectionMatrix()),
                              cv::Size(camera_right->numberOfImageCols(), camera_right->numberOfImageRows()),
                              CV_16SC2,
                              undistort_rectify_maps_right[0],
                              undistort_rectify_maps_right[1]);

  //ds unsubscribe
  subscriber_camera_info_left.shutdown();
  subscriber_camera_info_right.shutdown();

  //ds allocate SLAM modules
  proslam::SLAMAssembly slam_system(parameters);

  //ds set cameras TODO clean interface
  slam_system.loadCameras(camera_left, camera_right);

  //ds allocate a qt UI server in the main scope (required)
  std::shared_ptr<QApplication> ui_server(new QApplication(argc_, argv_));

  //ds initialize gui
  slam_system.initializeGUI(ui_server);

  //ds set up subscribers
  message_filters::Subscriber<sensor_msgs::Image> subscriber_image_left(node, parameters->command_line_parameters->topic_image_left, 5);
  message_filters::Subscriber<sensor_msgs::Image> subscriber_image_right(node, parameters->command_line_parameters->topic_image_right, 5);

  //ds define policy and initialize synchronizer
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> StereoImagePolicy;
  message_filters::Synchronizer<StereoImagePolicy> stereo_image_synchronizer(StereoImagePolicy(5), subscriber_image_left, subscriber_image_right);
  if (parameters->command_line_parameters->tracker_mode == proslam::CommandLineParameters::TrackerMode::RGB_STEREO) {
    stereo_image_synchronizer.registerCallback(boost::bind(&callbackStereoImage, _1, _2));
  } else {
    stereo_image_synchronizer.registerCallback(boost::bind(&callbackDepthImage, _1, _2));
  }
//  //ds subscribe to ground truth topic if available
//  ros::Subscriber subscriber_ground_truth = node.subscribe("/odom", 1, callbackGroundTruth);

//  //ds ground truth transform to robot (current is for the QUT dataset)
//  proslam::TransformMatrix3D orientation_correction(proslam::TransformMatrix3D::Identity());
//  orientation_correction.matrix() << 0, -1, 0, 0,
//                                     0, 0, -1, 0,
//                                     1, 0, 0, 0,
//                                     0, 0, 0, 1;

  //ds loop control
  proslam::Count number_of_frames_current_window = 0;
  double start_time_current_window_seconds       = srrg_core::getTime();
  const double measurement_interval_seconds      = 5;
  try {

    //ds start processing loop
    std::cerr << "main|starting processing loop" << std::endl;
    while (ros::ok()) {

      //ds trigger callbacks
      ros::spinOnce();

      //ds if we got a valid stereo image pair (enabled only in callback)
      if (found_image_pair) {

        //ds preprocess the images if desired: histogram equalization
        if (parameters->command_line_parameters->option_equalize_histogram) {
          cv::equalizeHist(image_left, image_left);
          cv::equalizeHist(image_right, image_right);
        }

        //ds process images
        slam_system.process(image_left, image_right);
	
        //ds add ground truth if available
//      slam_system.addGroundTruthMeasurement(orientation_correction*ground_truth);
        ++number_of_frames_current_window;

        //ds update ui
        slam_system.updateGUI();
        slam_system.draw();
        found_image_pair = false;
      } else {
        continue;
      }

      //ds display stats after each interval
      const double total_duration_seconds_current = srrg_core::getTime()-start_time_current_window_seconds;
      if (total_duration_seconds_current > measurement_interval_seconds) {

        //ds runtime info - depending on set modes
//      std::cerr << "fps: " << number_of_frames_current_window/total_duration_seconds_current << std::endl;

        number_of_frames_current_window   = 0;
        start_time_current_window_seconds = srrg_core::getTime();
      }
    }
  } catch (const std::runtime_error& exception_) {
    std::cerr << "main|caught exception '" << exception_.what() << "'" << std::endl;
    delete parameters;
    return 0;
  }

//  //ds print full report TODO merge into assembly
//  slam_system.printReport();

//  //ds save trajectory to disk
//  slam_system.worldMap()->writeTrajectory("trajectory.txt");

  //ds clean up parameters (since not used in GUI)
  delete parameters;

  //ds exit in GUI
  return ros::ok();
}
