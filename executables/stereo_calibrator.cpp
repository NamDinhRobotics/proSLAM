#include <iostream>

#include "srrg_messages/message_reader.h"
#include "srrg_messages/message_timestamp_synchronizer.h"
#include "srrg_messages/pinhole_image_message.h"
#include "framepoint_generation/stereo_framepoint_generator.h"

const bool measure(const cv::Mat& image_,
                   const cv::Size& board_size_,
                   const double& square_width_meters_,
                   std::vector<cv::Point3f>& object_points_,
                   std::vector<std::vector<cv::Point3f>>& object_points_per_image_,
                   std::vector<std::vector<cv::Point2f>>& image_points_per_image_,
                   cv::Mat& image_display_);

const double calibrate(const cv::Size image_size_,
                       std::vector<std::vector<cv::Point3f>>& object_points_per_image_,
                       std::vector<std::vector<cv::Point2f>>& image_points_per_image_,
                       cv::Mat& camera_calibration_matrix_,
                       cv::Mat& distortion_coefficients_);

int32_t main (int32_t argc, char** argv) {
  if (argc < 2) {
    std::cerr << "use: ./stereo_calibrator <BOSS/TXTIO_message_file> [-use-gui]" << std::endl;
    return 0;
  }

  //ds configuration
  std::string file_name_messages = argv[1];
  std::string topic_image_left   = "/camera_left/image_raw";
  std::string topic_image_right  = "/camera_right/image_raw";
  bool option_use_gui            = false;

  //ds parse parameters
  if (argc == 3 && !std::strcmp(argv[2], "-use-gui")) {
    option_use_gui = true;
  }

  //ds board configuration
  const uint32_t cols = 6;
  const uint32_t rows = 7;
  const cv::Size board_size(cols, rows);
  const double square_width_meters = 0.06;
  cv::Size image_size(0, 0);

  //ds log configuration
  std::cerr << "file_name_messages: " << file_name_messages << std::endl;
  std::cerr << "topic_image_left: " << topic_image_left << std::endl;
  std::cerr << "topic_image_right: " << topic_image_right << std::endl;
  std::cerr << "option_use_gui: " << option_use_gui << std::endl;

  //ds configure message synchronizer
  srrg_core::MessageTimestampSynchronizer synchronizer;
  std::vector<std::string> camera_topics_synchronized(0);
  camera_topics_synchronized.push_back(topic_image_left);
  camera_topics_synchronized.push_back(topic_image_right);
  synchronizer.setTimeInterval(0.001);
  synchronizer.setTopics(camera_topics_synchronized);

  //ds configure message reader
  srrg_core::MessageReader message_reader;
  message_reader.open(file_name_messages);
  if (!message_reader.good()) {
    std::cerr << "ERROR: unable to open message file: '" << file_name_messages << "'" << std::endl;
    return 0;
  }

  //ds calcuate outer corner positions (constant)
  std::vector<cv::Point3f> object_points;
  for (int32_t i = 0; i < board_size.height; ++i) {
    for (int32_t j = 0; j < board_size.width; ++j) {
      object_points.push_back(cv::Point3f(j*square_width_meters, i*square_width_meters, 0));
    }
  }

  //ds calibration data
  std::vector<std::vector<cv::Point3f>> object_points_per_image_left(0);
  std::vector<std::vector<cv::Point3f>> object_points_per_image_right(0);
  std::vector<std::vector<cv::Point2f>> image_points_per_image_left(0);
  std::vector<std::vector<cv::Point2f>> image_points_per_image_right(0);

  //ds info
  uint64_t number_of_processed_stereo_images  = 0;
  uint64_t number_of_measurements_stereo      = 0;
  const uint32_t measurement_image_interspace = 10;

  //ds start playback
  srrg_core::BaseMessage* message = 0;
  while ((message = message_reader.readMessage())) {
    srrg_core::BaseSensorMessage* sensor_message = dynamic_cast<srrg_core::BaseSensorMessage*>(message);
    sensor_message->untaint();

    //ds add to synchronizer
    if (sensor_message->topic() == topic_image_left) {
      synchronizer.putMessage(sensor_message);
    } else if (sensor_message->topic() == topic_image_right) {
      synchronizer.putMessage(sensor_message);
    } else {
      delete sensor_message;
    }

    //ds if we have a synchronized package of sensor messages ready
    if (synchronizer.messagesReady()) {

      //ds buffer sensor data
      srrg_core::PinholeImageMessage* image_message_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(synchronizer.messages()[0].get());
      srrg_core::PinholeImageMessage* image_message_right = dynamic_cast<srrg_core::PinholeImageMessage*>(synchronizer.messages()[1].get());

      //ds grab opencv image data
      cv::Mat image_left  = image_message_left->image();
      cv::Mat image_right = image_message_right->image();
      cv::Mat image_display_left, image_display_right;
      cv::cvtColor(image_left, image_display_left, CV_GRAY2RGB);
      cv::cvtColor(image_right, image_display_right, CV_GRAY2RGB);
      image_size.height = image_left.rows;
      image_size.width  = image_left.cols;

      //ds reduce number of measurements
      if (number_of_processed_stereo_images % measurement_image_interspace == 0) {

        //ds locate chessboard in the left and right image
        const bool measured_left  = measure(image_left, board_size, square_width_meters, object_points,
                                            object_points_per_image_left, image_points_per_image_left, image_display_left);
        const bool measured_right = measure(image_right, board_size, square_width_meters, object_points,
                                            object_points_per_image_right, image_points_per_image_right, image_display_right);

        //ds if both images contained the pattern
        if (measured_left && measured_right) {
          ++number_of_measurements_stereo;
        }

        //ds visual info
        if (option_use_gui) {
          cv::Mat image_stereo;
          cv::hconcat(image_display_left, image_display_right, image_stereo);
          cv::imshow(topic_image_left + ", " + topic_image_right, image_stereo);
          cv::waitKey(1);
        }

        //ds status
        std::printf("%06lu|L: %f|R: %f|CL: %i CR: %i|"
                    "L: %6lu R: %6lu S: %6lu\n", number_of_processed_stereo_images,
                                                 image_message_left->timestamp(), image_message_right->timestamp(),
                                                 measured_left, measured_right,
                                                 object_points_per_image_left.size(), object_points_per_image_right.size(), number_of_measurements_stereo);
      }

      //ds release processed data
      image_message_left->release();
      image_message_right->release();
      image_left.release();
      image_right.release();
      synchronizer.reset();
      ++number_of_processed_stereo_images;
    }
  }

  //ds done
  message_reader.close();
  synchronizer.reset();

  //ds info
  std::cerr << "obtained measurements   LEFT: " << object_points_per_image_left.size() << std::endl;
  std::cerr << "obtained measurements  RIGHT: " << object_points_per_image_right.size() << std::endl;
  std::cerr << "obtained measurements STEREO: " << number_of_measurements_stereo << std::endl;

  //ds objectives
  cv::Mat camera_calibration_matrix_left(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat camera_calibration_matrix_right(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat distortion_coefficients_left(cv::Mat::zeros(4, 1, CV_64F));
  cv::Mat distortion_coefficients_right(cv::Mat::zeros(4, 1, CV_64F));

  //ds set camera calibration matrix estimates (eth)
  camera_calibration_matrix_left.at<double>(0,0)  = 458.654;
  camera_calibration_matrix_left.at<double>(0,2)  = 367.215;
  camera_calibration_matrix_left.at<double>(1,1)  = 457.296;
  camera_calibration_matrix_left.at<double>(1,2)  = 248.375;
  camera_calibration_matrix_right.at<double>(0,0) = 457.587;
  camera_calibration_matrix_right.at<double>(0,2) = 379.999;
  camera_calibration_matrix_right.at<double>(1,1) = 456.134;
  camera_calibration_matrix_right.at<double>(1,2) = 255.238;

  //ds set distortion estimates (eth)
  distortion_coefficients_left.at<double>(0)  = -0.28340811;
  distortion_coefficients_left.at<double>(1)  = 0.07395907;
  distortion_coefficients_left.at<double>(2)  = 0.00019359;
  distortion_coefficients_left.at<double>(3)  = 1.76187114e-05;
  distortion_coefficients_right.at<double>(0) = -0.28368365;
  distortion_coefficients_right.at<double>(1) = 0.07451284;
  distortion_coefficients_right.at<double>(2) = -0.00010473;
  distortion_coefficients_right.at<double>(3) = -3.55590700e-05;

  //ds calibrate on all measurements
  std::cerr << "calibrating  LEFT .. ";
  const double reprojection_error_left  = calibrate(image_size, object_points_per_image_left, image_points_per_image_left,
                                                    camera_calibration_matrix_left, distortion_coefficients_left);
  std::cerr << " reprojection error: " << reprojection_error_left << std::endl;
  std::cerr << "calibrating RIGHT .. ";
  const double reprojection_error_right = calibrate(image_size, object_points_per_image_right, image_points_per_image_right,
                                                    camera_calibration_matrix_right, distortion_coefficients_right);
  std::cerr << " reprojection error: " << reprojection_error_right << std::endl;

  //ds display the obtained camera calibration matrices and distortion coefficients
  std::cerr << "\ncamera matrices: \n" << std::endl;
  std::cerr << "             LEFT            -             RIGHT" << std::endl;
  std::printf("%8.3f %8.3f %8.3f   |   %8.3f %8.3f %8.3f\n", camera_calibration_matrix_left.at<double>(0,0),
                                                             camera_calibration_matrix_left.at<double>(0,1),
                                                             camera_calibration_matrix_left.at<double>(0,2),
                                                             camera_calibration_matrix_right.at<double>(0,0),
                                                             camera_calibration_matrix_right.at<double>(0,1),
                                                             camera_calibration_matrix_right.at<double>(0,2));
  std::printf("%8.3f %8.3f %8.3f   |   %8.3f %8.3f %8.3f\n", camera_calibration_matrix_left.at<double>(1,0),
                                                             camera_calibration_matrix_left.at<double>(1,1),
                                                             camera_calibration_matrix_left.at<double>(1,2),
                                                             camera_calibration_matrix_right.at<double>(1,0),
                                                             camera_calibration_matrix_right.at<double>(1,1),
                                                             camera_calibration_matrix_right.at<double>(1,2));
  std::printf("%8.3f %8.3f %8.3f   |   %8.3f %8.3f %8.3f\n", camera_calibration_matrix_left.at<double>(2,0),
                                                             camera_calibration_matrix_left.at<double>(2,1),
                                                             camera_calibration_matrix_left.at<double>(2,2),
                                                             camera_calibration_matrix_right.at<double>(2,0),
                                                             camera_calibration_matrix_right.at<double>(2,1),
                                                             camera_calibration_matrix_right.at<double>(2,2));

  std::cerr << "\ndistortion coefficients: \n" << std::endl;
  std::printf(" LEFT: %8.3f, %8.3f, %8.3f, %8.3f\n",
              distortion_coefficients_left.at<double>(0),
              distortion_coefficients_left.at<double>(1),
              distortion_coefficients_left.at<double>(2),
              distortion_coefficients_left.at<double>(3));
  std::printf("RIGHT: %8.3f, %8.3f, %8.3f, %8.3f\n",
              distortion_coefficients_right.at<double>(0),
              distortion_coefficients_right.at<double>(1),
              distortion_coefficients_right.at<double>(2),
              distortion_coefficients_right.at<double>(3));

  //ds rectification configuration
  cv::Mat rotation_camera_left_to_right(3, 3, CV_64F, cv::Scalar(0));
  cv::Mat translation_camera_left_to_right(3, 1, CV_64F, cv::Scalar(0));
  cv::Mat depth_mapping(4, 4, CV_64F, cv::Scalar(0));
  cv::Mat projection_matrix_left(3, 4, CV_64F, cv::Scalar(0));
  cv::Mat projection_matrix_right(3, 4, CV_64F, cv::Scalar(0));
  cv::Mat rectification_matrix_left(3, 3, CV_64F, cv::Scalar(0));
  cv::Mat rectification_matrix_right(3, 3, CV_64F, cv::Scalar(0));

  //ds set transforms (eth)
  Eigen::Isometry3d transform_camera_left_to_body(Eigen::Isometry3d::Identity());
  transform_camera_left_to_body.matrix() << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                                            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                                           -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                                            0.0, 0.0, 0.0, 1.0;
  Eigen::Isometry3d transform_camera_right_to_body(Eigen::Isometry3d::Identity());
  transform_camera_right_to_body.matrix() << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
                                             0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
                                            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
                                             0.0, 0.0, 0.0, 1.0;

  //ds compute relative transform between cameras and convert it to opencv
  const Eigen::Isometry3d transform_camera_left_to_right = transform_camera_right_to_body.inverse()*transform_camera_left_to_body;
  for (int32_t u = 0; u < 3; ++u) {
    for (int32_t v = 0; v < 3; ++v) {
      rotation_camera_left_to_right.at<double>(u, v) = transform_camera_left_to_right.linear()(u,v);
    }
    translation_camera_left_to_right.at<double>(u) = transform_camera_left_to_right.translation()(u);
  }

  std::cerr << "\ntransform LEFT to RIGHT: \n" << std::endl;
  std::cerr << transform_camera_left_to_right.matrix() << std::endl;

  //ds compute rectification parameters
  cv::stereoRectify(camera_calibration_matrix_left,
                    distortion_coefficients_left,
                    camera_calibration_matrix_right,
                    distortion_coefficients_right,
                    image_size,
                    rotation_camera_left_to_right,
                    translation_camera_left_to_right,
                    rectification_matrix_left,
                    rectification_matrix_right,
                    projection_matrix_left,
                    projection_matrix_right,
                    depth_mapping,
                    CV_CALIB_ZERO_DISPARITY,
                    0);

  //ds display the obtained projection matrices
  std::cerr << "\nprojection matrices: \n" << std::endl;
  std::cerr << "               LEFT               -               RIGHT" << std::endl;
  std::printf("%8.3f %8.3f %8.3f %8.3f   |   %8.3f %8.3f %8.3f %8.3f\n", projection_matrix_left.at<double>(0,0),
                                                                         projection_matrix_left.at<double>(0,1),
                                                                         projection_matrix_left.at<double>(0,2),
                                                                         projection_matrix_left.at<double>(0,3),
                                                                         projection_matrix_right.at<double>(0,0),
                                                                         projection_matrix_right.at<double>(0,1),
                                                                         projection_matrix_right.at<double>(0,2),
                                                                         projection_matrix_right.at<double>(0,3));
  std::printf("%8.3f %8.3f %8.3f %8.3f   |   %8.3f %8.3f %8.3f %8.3f\n", projection_matrix_left.at<double>(1,0),
                                                                         projection_matrix_left.at<double>(1,1),
                                                                         projection_matrix_left.at<double>(1,2),
                                                                         projection_matrix_left.at<double>(1,3),
                                                                         projection_matrix_right.at<double>(1,0),
                                                                         projection_matrix_right.at<double>(1,1),
                                                                         projection_matrix_right.at<double>(1,2),
                                                                         projection_matrix_right.at<double>(1,3));
  std::printf("%8.3f %8.3f %8.3f %8.3f   |   %8.3f %8.3f %8.3f %8.3f\n", projection_matrix_left.at<double>(2,0),
                                                                         projection_matrix_left.at<double>(2,1),
                                                                         projection_matrix_left.at<double>(2,2),
                                                                         projection_matrix_left.at<double>(2,3),
                                                                         projection_matrix_right.at<double>(2,0),
                                                                         projection_matrix_right.at<double>(2,1),
                                                                         projection_matrix_right.at<double>(2,2),
                                                                         projection_matrix_right.at<double>(2,3));

  //ds compute undistortion and rectification mappings
  cv::Mat undistort_rectify_maps_left[2];
  cv::Mat undistort_rectify_maps_right[2];

  //ds compute undistorted and rectified mappings
  cv::initUndistortRectifyMap(camera_calibration_matrix_left,
                              distortion_coefficients_left,
                              rectification_matrix_left,
                              projection_matrix_left,
                              image_size,
                              CV_16SC2,
                              undistort_rectify_maps_left[0],
                              undistort_rectify_maps_left[1]);
  cv::initUndistortRectifyMap(camera_calibration_matrix_right,
                              distortion_coefficients_right,
                              rectification_matrix_right,
                              projection_matrix_right,
                              image_size,
                              CV_16SC2,
                              undistort_rectify_maps_right[0],
                              undistort_rectify_maps_right[1]);

  std::cerr << "\ncalibration completed - benchmarking epipolar matching on all images .." << std::endl;

  //ds epipolar structure generation
#if CV_MAJOR_VERSION == 2
  cv::FeatureDetector* keypoint_detector        = new cv::FastFeatureDetector();
  cv::DescriptorExtractor* descriptor_extractor = new cv::BriefDescriptorExtractor();
#elif CV_MAJOR_VERSION == 3
  cv::Ptr<cv::FastFeatureDetector> keypoint_detector    = cv::FastFeatureDetector::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
#else
  #error OpenCV version not supported
#endif

  //ds restart the stream to check the found parameters
  message_reader.open(file_name_messages);
  message = 0;
  number_of_processed_stereo_images         = 0;
  uint64_t total_number_of_epipolar_matches = 0;
  while ((message = message_reader.readMessage())) {
    srrg_core::BaseSensorMessage* sensor_message = dynamic_cast<srrg_core::BaseSensorMessage*>(message);
    sensor_message->untaint();

    //ds add to synchronizer
    if (sensor_message->topic() == topic_image_left) {
      synchronizer.putMessage(sensor_message);
    } else if (sensor_message->topic() == topic_image_right) {
      synchronizer.putMessage(sensor_message);
    } else {
      delete sensor_message;
    }

    //ds if we have a synchronized package of sensor messages ready
    if (synchronizer.messagesReady()) {

      //ds buffer sensor data
      srrg_core::PinholeImageMessage* image_message_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(synchronizer.messages()[0].get());
      srrg_core::PinholeImageMessage* image_message_right = dynamic_cast<srrg_core::PinholeImageMessage*>(synchronizer.messages()[1].get());

      //ds grab opencv image data
      cv::Mat image_left  = image_message_left->image();
      cv::Mat image_right = image_message_right->image();
      cv::Mat image_display_left, image_display_right;

      //ds undistort and rectify
      cv::Mat image_left_undistorted_rectified;
      cv::Mat image_right_undistorted_rectified;
      cv::remap(image_left, image_left_undistorted_rectified, undistort_rectify_maps_left[0], undistort_rectify_maps_left[1], cv::INTER_LINEAR);
      cv::remap(image_left, image_right_undistorted_rectified, undistort_rectify_maps_right[0], undistort_rectify_maps_right[1], cv::INTER_LINEAR);
      cv::cvtColor(image_left_undistorted_rectified, image_display_left, CV_GRAY2RGB);
      cv::cvtColor(image_right_undistorted_rectified, image_display_right, CV_GRAY2RGB);

      //ds check epipolar matching - detect keypoints
      std::vector<cv::KeyPoint> keypoints_left(0);
      std::vector<cv::KeyPoint> keypoints_right(0);
      keypoint_detector->detect(image_left_undistorted_rectified, keypoints_left);
      keypoint_detector->detect(image_right_undistorted_rectified, keypoints_right);

      //ds compute descriptors
      cv::Mat descriptors_left;
      cv::Mat descriptors_right;
      descriptor_extractor->compute(image_left_undistorted_rectified, keypoints_left, descriptors_left);
      descriptor_extractor->compute(image_right_undistorted_rectified, keypoints_right, descriptors_right);

      //ds assemble feature vectors
      std::vector<proslam::BaseFramePointGenerator::KeypointWithDescriptor> features_left(keypoints_left.size());
      std::vector<proslam::BaseFramePointGenerator::KeypointWithDescriptor> features_right(keypoints_right.size());
      for (uint64_t index = 0; index < features_left.size(); ++index) {
        features_left[index].available  = true;
        features_left[index].keypoint   = keypoints_left[index];
        features_left[index].descriptor = descriptors_left.row(index);
      }
      for (uint64_t index = 0; index < features_right.size(); ++index) {
        features_right[index].available  = true;
        features_right[index].keypoint   = keypoints_right[index];
        features_right[index].descriptor = descriptors_right.row(index);
      }

      //ds sort all input vectors by ascending row positions
      std::sort(features_left.begin(), features_left.end(),
                [](const proslam::BaseFramePointGenerator::KeypointWithDescriptor& a_,
                   const proslam::BaseFramePointGenerator::KeypointWithDescriptor& b_){return ((a_.keypoint.pt.y < b_.keypoint.pt.y) ||
                                                                                               (a_.keypoint.pt.y == b_.keypoint.pt.y && a_.keypoint.pt.x < b_.keypoint.pt.x));});
      std::sort(features_right.begin(), features_right.end(),
                [](const proslam::BaseFramePointGenerator::KeypointWithDescriptor& a_,
                   const proslam::BaseFramePointGenerator::KeypointWithDescriptor& b_){return ((a_.keypoint.pt.y < b_.keypoint.pt.y) ||
                                                                                               (a_.keypoint.pt.y == b_.keypoint.pt.y && a_.keypoint.pt.x < b_.keypoint.pt.x));});

      //ds running variables
      uint64_t index_R = 0;
      uint64_t number_of_epipolar_matches = 0;
      const double maximum_matching_distance = 25;

      //ds loop over all left keypoints
      for (uint64_t idx_L = 0; idx_L < features_left.size(); idx_L++) {

        //ds if the point is not yet matched
        if (features_left[idx_L].available) {

          //ds if there are no more points on the right to match against - stop
          if (index_R == features_right.size()) {break;}
          //the right keypoints are on an lower row - skip left
          while (features_left[idx_L].keypoint.pt.y < features_right[index_R].keypoint.pt.y) {
            idx_L++; if (idx_L == features_left.size()) {break;}
          }
          if (idx_L == features_left.size()) {break;}
          //the right keypoints are on an upper row - skip right
          while (features_left[idx_L].keypoint.pt.y > features_right[index_R].keypoint.pt.y) {
            index_R++; if (index_R == features_right.size()) {break;}
          }
          if (index_R == features_right.size()) {break;}
          //search bookkeeping
          uint64_t index_search_R = index_R;
          double distance_best    = maximum_matching_distance;
          uint64_t index_best_R   = 0;
          //scan epipolar line for current keypoint at idx_L
          while (features_left[idx_L].keypoint.pt.y == features_right[index_search_R].keypoint.pt.y) {
            //zero disparity stop condition
            if (features_right[index_search_R].keypoint.pt.x >= features_left[idx_L].keypoint.pt.x) {break;}

            //ds if the point is not yet matched
            if (features_right[index_search_R].available) {

              //ds compute descriptor distance for the stereo match candidates
              const double distance_hamming = cv::norm(features_left[idx_L].descriptor, features_right[index_search_R].descriptor, SRRG_PROSLAM_DESCRIPTOR_NORM);
              if(distance_hamming < distance_best) {
                distance_best = distance_hamming;
                index_best_R  = index_search_R;
              }
            }
            index_search_R++; if (index_search_R == features_right.size()) {break;}
          }
          //check if something was found
          if (distance_best < maximum_matching_distance) {

            //ds reduce search space
            index_R = index_best_R+1;

            //ds set as matched (required for multi-line stereo matching)
            features_left[idx_L].available         = false;
            features_right[index_best_R].available = false;
            ++number_of_epipolar_matches;
          }
        }
      }

      //ds visual info
      if (option_use_gui) {
        for (const proslam::BaseFramePointGenerator::KeypointWithDescriptor& feature: features_left) {
          if (feature.available) {
            cv::circle(image_display_left, feature.keypoint.pt, 2, cv::Scalar(255, 0, 0), -1);
          } else {
            cv::circle(image_display_left, feature.keypoint.pt, 2, cv::Scalar(0, 255, 0), -1);
          }
        }
        for (const proslam::BaseFramePointGenerator::KeypointWithDescriptor& feature: features_right) {
          if (feature.available) {
            cv::circle(image_display_right, feature.keypoint.pt, 2, cv::Scalar(255, 0, 0), -1);
          } else {
            cv::circle(image_display_right, feature.keypoint.pt, 2, cv::Scalar(0, 255, 0), -1);
          }
        }

        cv::Mat image_stereo;
        cv::hconcat(image_display_left, image_display_right, image_stereo);
        cv::imshow(topic_image_left + ", " + topic_image_right, image_stereo);
        cv::waitKey(1);
      }
//
//      //ds status
//      std::printf("%06lu|L: %f|R: %f|keypoints L: %5lu keypoints R: %5lu "
//                  "STEREO MATCHES: %5lu (%5.3f)\n", number_of_processed_stereo_images,
//                                                    image_message_left->timestamp(), image_message_right->timestamp(),
//                                                    keypoints_left.size(), keypoints_right.size(),
//                                                    number_of_epipolar_matches, static_cast<double>(number_of_epipolar_matches)/keypoints_left.size());
      std::cerr << "x";
      ++number_of_processed_stereo_images;
      total_number_of_epipolar_matches += number_of_epipolar_matches;
    }
  }

#if CV_MAJOR_VERSION == 2
  delete keypoint_detector;
  delete descriptor_extractor;
#endif

  std::cerr << "\nbenchmark completed - average number of epipolar matches per stereo image pair: "
            << static_cast<double>(total_number_of_epipolar_matches)/number_of_processed_stereo_images << std::endl;

  //ds done
  message_reader.close();
  synchronizer.reset();
  return 0;
}

const bool measure(const cv::Mat& image_,
                   const cv::Size& board_size_,
                   const double& square_width_meters_,
                   std::vector<cv::Point3f>& object_points_,
                   std::vector<std::vector<cv::Point3f>>& object_points_per_image_,
                   std::vector<std::vector<cv::Point2f>>& image_points_per_image_,
                   cv::Mat& image_display_) {

  //ds inner corners
  std::vector<cv::Point2f> image_points(0);

  //ds locate chessboard
  bool found_chessboard = cv::findChessboardCorners(image_, board_size_, image_points,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

  //ds return on failure
  if (!found_chessboard) {
    return false;
  }

  //ds visual info
  for (cv::Point2f point: image_points) {
    cv::circle(image_display_, point, 5, cv::Scalar(0, 255, 0), 1);
  }

  //ds update object and image points
  object_points_per_image_.push_back(object_points_);
  image_points_per_image_.push_back(image_points);

  //ds success
  return true;
}

const double calibrate(const cv::Size image_size_,
                       std::vector<std::vector<cv::Point3f>>& object_points_per_image_,
                       std::vector<std::vector<cv::Point2f>>& image_points_per_image_,
                       cv::Mat& camera_calibration_matrix_,
                       cv::Mat& distortion_coefficients_) {

  //ds additional output
  std::vector<cv::Mat> rotations_per_image;
  std::vector<cv::Mat> translations_per_image;

  //ds compute camera calibration matrix and distortion coefficients
  const double reprojection_error_pixels = cv::calibrateCamera(object_points_per_image_,
                                                               image_points_per_image_,
                                                               image_size_,
                                                               camera_calibration_matrix_,
                                                               distortion_coefficients_,
                                                               rotations_per_image,
                                                               translations_per_image,
                                                               CV_CALIB_USE_INTRINSIC_GUESS);

  //ds done
  return reprojection_error_pixels;
}
