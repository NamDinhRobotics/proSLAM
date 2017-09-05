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

#include "system/slam_assembly.h"

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

  //ds set cameras
  slam_system.loadCameras(camera_left, camera_right);

  //ds allocate a qt UI server in the main scope (required)
  QApplication* ui_server = new QApplication(argc_, argv_);

  //ds initialize gui
  slam_system.initializeGUI(ui_server);

  //ds set up subscribers
  message_filters::Subscriber<sensor_msgs::Image> subscriber_image_left(node, parameters->command_line_parameters->topic_image_left, 5);
  message_filters::Subscriber<sensor_msgs::Image> subscriber_image_right(node, parameters->command_line_parameters->topic_image_right, 5);

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
  try {

    //ds start processing loop
    std::cerr << "main|starting processing loop" << std::endl;
    while (ros::ok()) {

      //ds trigger callbacks
      ros::spinOnce();

      //ds if we got a valid stereo image pair (enabled only in callback)
      if (found_image_pair) {

        //ds preprocess the images if desired: rectification
        if (parameters->command_line_parameters->option_undistort_and_rectify) {
          cv::remap(image_left, image_left, undistort_rectify_maps_left[0], undistort_rectify_maps_left[1], cv::INTER_LINEAR);
          cv::remap(image_right, image_right, undistort_rectify_maps_right[0], undistort_rectify_maps_right[1], cv::INTER_LINEAR);
        }

        //ds preprocess the images if desired: histogram equalization
        if (parameters->command_line_parameters->option_equalize_histogram) {
          cv::equalizeHist(image_left, image_left);
          cv::equalizeHist(image_right, image_right);
        }

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

  //      std::cerr << "fps: " << number_of_frames_current_window/total_duration_seconds_current << std::endl;

        number_of_frames_current_window   = 0;
        start_time_current_window_seconds = srrg_core::getTime();
      }

      //ds update ui
      slam_system.updateGUI();
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
