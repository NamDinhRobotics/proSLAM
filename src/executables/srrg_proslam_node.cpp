#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "parameter_server.h"
#include "slam_assembly.h"

//ds lazy globals
proslam::Camera* camera_left  = 0;
proslam::Camera* camera_right = 0;

//ds image handling globals
cv::Mat undistort_rectify_maps_left[2];
cv::Mat undistort_rectify_maps_right[2];
std::pair<double, cv::Mat> image_left;
std::pair<double, cv::Mat> image_right;
bool is_set_image_left  = false;
bool is_set_image_right = false;

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

//ds ground truth sources
void callbackGroundTruth() {

}

//ds don't allow any windoof compilation attempt!
int32_t main(int32_t argc, char ** argv) {

  //ds lock opencv to really use only 1 thread
  cv::setNumThreads(0);

  //ds enable opencv2 optimization
  cv::setUseOptimized(true);

  //ds obtain configuration
  proslam::ParameterServer::parseParametersFromCommandLine(argc, argv);

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
  slam_system.tracker()->setPixelDistanceTrackingMinimum(50);
  slam_system.tracker()->setPixelDistanceTrackingMaximum(50);
  slam_system.tracker()->aligner()->setMaximumErrorKernel(25);
  slam_system.tracker()->framepointGenerator()->setTargetNumberOfPoints(500);
  slam_system.relocalizer()->aligner()->setMaximumErrorKernel(0.5);
  slam_system.relocalizer()->aligner()->setMinimumNumberOfInliers(25);
  slam_system.relocalizer()->aligner()->setMinimumInlierRatio(0.5);
  slam_system.relocalizer()->setMinimumAbsoluteNumberOfMatchesPointwise(25);

  //ds allocate a qt UI server in the main scope (required)
  QApplication* ui_server = new QApplication(argc, argv);

  //ds initialize gui
  slam_system.initializeGUI(ui_server);

  //ds subscribe to camera image topics
  ros::Subscriber subscriber_camera_image_left  = node.subscribe(proslam::ParameterServer::topicCameraImageLeft(), 1, callbackImageLeft);
  ros::Subscriber subscriber_camera_image_right = node.subscribe(proslam::ParameterServer::topicCameraInfoRight(), 1, callbackImageRight);

  //ds start processing loop
  std::cerr << "main|starting processing loop" << std::endl;
  bool is_running = true;
  while (ros::ok() && is_running) {

    //ds trigger callbacks
    ros::spinOnce();

    //ds if we got a valid stereo image pair - brutal without buffers
    if (is_set_image_left && is_set_image_right) {

      //ds preprocess the images if desired
      if (proslam::ParameterServer::optionEqualizeHistogram()) {
        cv::equalizeHist(image_left.second, image_left.second);
        cv::equalizeHist(image_right.second, image_right.second);
      }

      //ds check large time delta (more than 100ms -> 10Hz)
      if (std::fabs(image_left.first-image_right.first) > 0.1) {
        std::cerr << "main|WARNING: received large timestamp delta between stereo images" << std::endl;
      }

      //ds process images
      slam_system.process(image_left.second, image_right.second);

      //ds add ground truth if available
      slam_system.addGroundTruthMeasurement(proslam::TransformMatrix3D::Identity());

      //ds update ui
      slam_system.updateGUI();

      //ds reset images
      is_set_image_left  = false;
      is_set_image_right = false;
    }
  }

  //ds print full report
  slam_system.printReport();

  //ds save trajectory to disk
  slam_system.worldMap()->writeTrajectory("trajectory.txt");

  //ds exit in GUI
  return slam_system.closeGUI();
}
