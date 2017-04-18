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

#include "../framepoint_generation/stereo_framepoint_generator.h"
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

//ds image preprocessing
void translate(cv::Mat &image_, const int32_t& offsetx_, const int32_t& offsety_){
  cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx_, 0, 1, offsety_);
  warpAffine(image_, image_, trans_mat,image_.size());
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
  slam_system.tracker()->framepointGenerator()->setDetectorThreshold(10);
  slam_system.tracker()->framepointGenerator()->setDetectorThresholdMinimum(10);
  slam_system.tracker()->framepointGenerator()->setDetectorThresholdMaximum(100);
  slam_system.tracker()->framepointGenerator()->setTargetNumberOfPoints(1000);
  slam_system.tracker()->framepointGenerator()->setMaximumMatchingDistanceTriangulation(50);
  slam_system.tracker()->framepointGenerator()->setMatchingDistanceTrackingThresholdMaximum(50);
  slam_system.tracker()->framepointGenerator()->setMatchingDistanceTrackingThresholdMinimum(50);
  slam_system.relocalizer()->aligner()->setMaximumErrorKernel(0.5);
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

  const cv::Point2f point_offset(camera_left->imageCols(), 0);

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

      //ds shift images, correcting invalid rectification
      translate(image_left, 0, 0);

//      //ds get a dummy frame
//      proslam::Frame* frame = slam_system.worldMap()->createFrame(proslam::TransformMatrix3D::Identity(), slam_system.tracker()->framepointGenerator()->maximumDepthNearMeters());
//      frame->setCameraLeft(camera_left);
//      frame->setCameraRight(camera_right);
//
//      //ds compute keypoints
//      std::vector<cv::KeyPoint> keypoints_left;
//      std::vector<cv::KeyPoint> keypoints_right;
//      slam_system.tracker()->framepointGenerator()->detectKeypoints(image_left, keypoints_left);
//      slam_system.tracker()->framepointGenerator()->detectKeypoints(image_right, keypoints_right);
//
//      const proslam::Count number_of_keypoints_left  = keypoints_left.size();
//      const proslam::Count number_of_keypoints_right = keypoints_right.size();
//
//      //ds compute descriptors
//      cv::Mat descriptors_left;
//      cv::Mat descriptors_right;
//
//      proslam::StereoFramePointGenerator* triangulator = dynamic_cast<proslam::StereoFramePointGenerator*>(slam_system.tracker()->framepointGenerator());
//      assert(triangulator);
//
//      triangulator->extractDescriptors(image_left, keypoints_left, descriptors_left);
//      triangulator->extractDescriptors(image_right, keypoints_right, descriptors_right);
//      triangulator->initialize(keypoints_left, keypoints_right, descriptors_left, descriptors_right);
//
//          //ds triangulate points and fill the frame
//      triangulator->findStereoKeypoints(frame);
//
//      //ds show the images
//      cv::Mat image_display;
//      cv::hconcat(image_left, image_right, image_display);
//      cv::cvtColor(image_display, image_display, CV_GRAY2RGB);
//
////      //ds draw keypoints on image
////      for (const cv::KeyPoint& keypoint_left: keypoints_left) {
////        cv::circle(image_display, keypoint_left.pt, 2, cv::Scalar(0, 255, 0));
////      }
////      for (const cv::KeyPoint& keypoint_right: keypoints_right) {
////        cv::circle(image_display, keypoint_right.pt+point_offset, 2, cv::Scalar(0, 255, 0));
////      }
//
//      //ds visualize triangulated points
//      for (proslam::Index row = 0; row < triangulator->numberOfRowsImage(); ++row) {
//        for (proslam::Index col = 0; col < triangulator->numberOfColsImage(); ++col) {
//          if (triangulator->framepointsInImage()[row][col]) {
//            const proslam::FramePoint* point = triangulator->framepointsInImage()[row][col];
//
//            //ds determine colorization
//            cv::Scalar color (0, 0, 0);
//            if (point->isNear()) {
//              color = cv::Scalar(255, 0, 0);
//            } else {
//              color = cv::Scalar(255, 0, 255);
//            }
//
//            //ds draw left and right detection
//            cv::circle(image_display, point->keypointLeft().pt, 2, color, -1);
//            cv::circle(image_display, point->keypointRight().pt+point_offset, 2, color, -1);
//            triangulator->framepointsInImage()[row][col] = 0;
//          }
//        }
//      }
//
//      std::cerr << "L keypoints: " << number_of_keypoints_left << " descriptors: " << descriptors_left.rows
//                << " R keypoints: " << number_of_keypoints_right << " descriptors: " << descriptors_right.rows
//                << " framepoints: " << triangulator->numberOfAvailablePoints() << std::endl;
//
//      cv::imshow("FAST detection", image_display);
//      cv::waitKey(1);





      //ds process images
      slam_system.process(image_left, image_right);

      //ds add ground truth if available
//      slam_system.addGroundTruthMeasurement(orientation_correction*ground_truth);
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

//      std::cerr << "fps: " << number_of_frames_current_window/total_duration_seconds_current << std::endl;

      number_of_frames_current_window   = 0;
      start_time_current_window_seconds = srrg_core::getTime();
    }

    //ds update ui
    slam_system.updateGUI();
  }

//  //ds print full report TODO merge into assembly
//  slam_system.printReport();

//  //ds save trajectory to disk
//  slam_system.worldMap()->writeTrajectory("trajectory.txt");

  //ds exit in GUI
  return slam_system.closeGUI(ros::ok());
  return 0;
}
