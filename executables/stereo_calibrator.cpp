#include <iostream>
#include <fstream>
#include <iomanip>

#include "framepoint_generation/stereo_framepoint_generator.h"

struct Image {
    Image(const double& timestamp_seconds_, const std::string file_name_image_): timestamp_seconds(timestamp_seconds_),
                                                                                 file_name_image(file_name_image_) {}
    double timestamp_seconds;
    std::string file_name_image;
};

const std::vector<Image> getImagesASL(const std::string& folder_images_);

const bool measure(const cv::Mat& image_,
                   const cv::Size& board_size_,
                   const double& square_width_meters_,
                   std::vector<cv::Point2f>& image_points_,
                   cv::Mat& image_display_);

const double calibrate(const cv::Size image_size_,
                       std::vector<std::vector<cv::Point3f>>& object_points_per_image_,
                       std::vector<std::vector<cv::Point2f>>& image_points_per_image_,
                       cv::Mat& camera_calibration_matrix_,
                       cv::Mat& distortion_coefficients_);

int32_t main (int32_t argc_, char** argv_) {
  if (argc_ < 6) {
    std::cerr << "use: ./stereo_calibrator -asl <folder_images_left> <folder_images_right> -o <calibration.txt>"<< std::endl;
    std::cerr << "                        [-use-gui -interspace <integer> -use-eth -check-orb -test-asl <folder_images_left_test> <folder_images_right_test>]" << std::endl;
    std::cerr << "                              <folder_images_left/right> should each contain: data, data.csv" << std::endl;
    return 0;
  }

  //ds configuration
  std::string input_format                          = "";
  std::string folder_images_left                    = "";
  std::string folder_images_right                   = "";
  std::string folder_images_left_test               = "";
  std::string folder_images_right_test              = "";
  bool option_use_gui                               = false;
  uint32_t measurement_image_interspace             = 10;
  bool option_use_eth_estimates                     = false;
  const double maximum_timestamp_difference_seconds = 0.01;
  std::string output_file_name                      = "";
  bool option_test_orb_slam_calibration             = false;

  //ds parse parameters
  int32_t number_of_checked_parameters = 1;
  while (number_of_checked_parameters < argc_) {
    if (!std::strcmp(argv_[number_of_checked_parameters], "-asl")) {
      input_format = "asl";
      ++number_of_checked_parameters;
      if (number_of_checked_parameters == argc_) {break;}
      folder_images_left = argv_[number_of_checked_parameters];
      ++number_of_checked_parameters;
      if (number_of_checked_parameters == argc_) {break;}
      folder_images_right = argv_[number_of_checked_parameters];
    } else if (!std::strcmp(argv_[number_of_checked_parameters], "-use-gui")) {
      option_use_gui = true;
    } else if (!std::strcmp(argv_[number_of_checked_parameters], "-interspace")) {
      ++number_of_checked_parameters;
      if (number_of_checked_parameters == argc_) {break;}
      measurement_image_interspace = std::stoi(argv_[number_of_checked_parameters]);
    } else if (!std::strcmp(argv_[number_of_checked_parameters], "-use-eth")) {
      option_use_eth_estimates = true;
    } else if (!std::strcmp(argv_[number_of_checked_parameters], "-o")) {
      ++number_of_checked_parameters;
      if (number_of_checked_parameters == argc_) {break;}
      output_file_name = argv_[number_of_checked_parameters];
    } else if (!std::strcmp(argv_[number_of_checked_parameters], "-check-orb")) {
      option_test_orb_slam_calibration = true;
    } else if (!std::strcmp(argv_[number_of_checked_parameters], "-test-asl")) {
      input_format = "asl";
      ++number_of_checked_parameters;
      if (number_of_checked_parameters == argc_) {break;}
      folder_images_left_test = argv_[number_of_checked_parameters];
      ++number_of_checked_parameters;
      if (number_of_checked_parameters == argc_) {break;}
      folder_images_right_test = argv_[number_of_checked_parameters];
    }
    ++number_of_checked_parameters;
  }

  //ds input validation
  if (input_format.empty()) {
    std::cerr << "ERROR: specify input_format (e.g. -asl)" << std::endl;
    return 0;
  }
  if (folder_images_left.empty()) {
    std::cerr << "ERROR: specify folder_images_left" << std::endl;
    return 0;
  }
  if (folder_images_right.empty()) {
    std::cerr << "ERROR: specify folder_images_right" << std::endl;
    return 0;
  }
  if (output_file_name.empty()) {
    std::cerr << "ERROR: specify output_file_name (e.g. -o <calibration.txt>)" << std::endl;
    return 0;
  }

  //ds if no test images are specified - use calibration ones
  if (folder_images_left_test.empty() || folder_images_right_test.empty()) {
    folder_images_left_test  = folder_images_left;
    folder_images_right_test = folder_images_right;
  }

  //ds board configuration
  const uint32_t cols = 6;
  const uint32_t rows = 7;
  const cv::Size board_size(cols, rows);
  const double square_width_meters = 0.06;
  cv::Size image_size(0, 0);

  //ds log configuration
  std::cerr << "input_format: " << input_format << std::endl;
  std::cerr << "folder_images_left: " << folder_images_left << std::endl;
  std::cerr << "folder_images_right: " << folder_images_right << std::endl;
  std::cerr << "option_use_gui: " << option_use_gui << std::endl;
  std::cerr << "measurement_image_interspace: " << measurement_image_interspace << std::endl;
  std::cerr << "option_use_eth_estimates: " << (option_use_eth_estimates || option_test_orb_slam_calibration) << std::endl;
  std::cerr << "output_file_name: " << output_file_name << std::endl;
  std::cerr << "option_test_orb_slam_calibration: " << option_test_orb_slam_calibration << std::endl;
  std::cerr << "folder_images_left_test: " << folder_images_left_test << std::endl;
  std::cerr << "folder_images_right_test: " << folder_images_right_test << std::endl;

  //ds load images for left and right - assuming to be synchronized
  const std::vector<Image> images_left(getImagesASL(folder_images_left));
  const std::vector<Image> images_right(getImagesASL(folder_images_right));
  if (images_left.size() == 0) {
    std::cerr << "ERROR: no left images loaded" << std::endl;
    return 0;
  }
  if (images_right.size() == 0) {
    std::cerr << "ERROR: no right images loaded" << std::endl;
    return 0;
  }
  if (images_left.size() != images_right.size()) {
    std::cerr << "ERROR: unequal numbers of left and right images" << std::endl;
    return 0;
  }
  const uint32_t number_of_images = images_left.size();

  //ds calcuate outer corner positions (constant)
  std::vector<cv::Point3f> object_points;
  for (int32_t i = 0; i < board_size.height; ++i) {
    for (int32_t j = 0; j < board_size.width; ++j) {
      object_points.push_back(cv::Point3f(j*square_width_meters, i*square_width_meters, 0));
    }
  }

  //ds calibration data
  std::vector<std::vector<cv::Point3f>> object_points_per_image(0);
  std::vector<std::vector<cv::Point2f>> image_points_per_image_left(0);
  std::vector<std::vector<cv::Point2f>> image_points_per_image_right(0);

  //ds info
  uint64_t number_of_processed_stereo_images = 0;
  for (uint32_t image_number = 0; image_number < number_of_images; ++image_number) {

    //ds compute timestamp delta
    const double timestamp_difference_seconds = std::fabs(images_left[image_number].timestamp_seconds-images_right[image_number].timestamp_seconds);

    //ds if we have a synchronized package of sensor messages ready
    if (timestamp_difference_seconds < maximum_timestamp_difference_seconds) {

      //ds grab opencv image data
      cv::Mat image_left  = cv::imread(images_left[image_number].file_name_image, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat image_right = cv::imread(images_right[image_number].file_name_image, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat image_display_left, image_display_right;
      cv::cvtColor(image_left, image_display_left, CV_GRAY2RGB);
      cv::cvtColor(image_right, image_display_right, CV_GRAY2RGB);
      image_size.height = image_left.rows;
      image_size.width  = image_left.cols;

      //ds reduce number of measurements
      if (number_of_processed_stereo_images % measurement_image_interspace == 0) {

        //ds locate chessboard in the left and right image
        std::vector<cv::Point2f> image_points_left;
        std::vector<cv::Point2f> image_points_right;
        const bool measured_left  = measure(image_left, board_size, square_width_meters, image_points_left, image_display_left);
        const bool measured_right = measure(image_right, board_size, square_width_meters, image_points_right, image_display_right);

        //ds if both images contained the pattern
        if (measured_left && measured_right) {

          //ds update object and image points
          object_points_per_image.push_back(object_points);
          image_points_per_image_left.push_back(image_points_left);
          image_points_per_image_right.push_back(image_points_right);
        }

        //ds visual info
        if (option_use_gui) {
          cv::Mat image_stereo;
          cv::hconcat(image_display_left, image_display_right, image_stereo);
          cv::imshow("calibration", image_stereo);
          cv::waitKey(1);
        }

        //ds status
        std::printf("%06lu|L: %f|R: %f|CL: %i CR: %i|measurements: %6lu\n", number_of_processed_stereo_images,
                                                                            images_left[image_number].timestamp_seconds, images_right[image_number].timestamp_seconds,
                                                                            measured_left, measured_right,
                                                                            object_points_per_image.size());
      }

      //ds release processed data
      ++number_of_processed_stereo_images;
    } else {
      std::cerr << "WARNING: received high timestamp difference: " << timestamp_difference_seconds << " skipping the measurements" << std::endl;
    }
  }
  cv::destroyAllWindows();

  //ds info
  std::cerr << "obtained measurements: " << object_points_per_image.size() << std::endl;

  //ds objectives
  cv::Mat camera_calibration_matrix_left(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat camera_calibration_matrix_right(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat distortion_coefficients_left(cv::Mat::zeros(4, 1, CV_64F));
  cv::Mat distortion_coefficients_right(cv::Mat::zeros(4, 1, CV_64F));

  //ds check estimate initialization
  if (option_use_eth_estimates || option_test_orb_slam_calibration) {
    std::cerr << "using ETH estimates for camera calibration matrices and distortion coefficients" << std::endl;

    //ds set camera calibration matrix estimates
    camera_calibration_matrix_left.at<double>(0,0)  = 458.654;
    camera_calibration_matrix_left.at<double>(0,2)  = 367.215;
    camera_calibration_matrix_left.at<double>(1,1)  = 457.296;
    camera_calibration_matrix_left.at<double>(1,2)  = 248.375;
    camera_calibration_matrix_right.at<double>(0,0) = 457.587;
    camera_calibration_matrix_right.at<double>(0,2) = 379.999;
    camera_calibration_matrix_right.at<double>(1,1) = 456.134;
    camera_calibration_matrix_right.at<double>(1,2) = 255.238;

    //ds set distortion estimates
    distortion_coefficients_left.at<double>(0)  = -0.28340811;
    distortion_coefficients_left.at<double>(1)  = 0.07395907;
    distortion_coefficients_left.at<double>(2)  = 0.00019359;
    distortion_coefficients_left.at<double>(3)  = 1.76187114e-05;
    distortion_coefficients_right.at<double>(0) = -0.28368365;
    distortion_coefficients_right.at<double>(1) = 0.07451284;
    distortion_coefficients_right.at<double>(2) = -0.00010473;
    distortion_coefficients_right.at<double>(3) = -3.55590700e-05;
  } else {
    std::cerr << "calibrating camera LEFT .. ";
    camera_calibration_matrix_left  = cv::initCameraMatrix2D(object_points_per_image, image_points_per_image_left, image_size, 0);
    std::cerr << "done" << std::endl;
    std::cerr << "calibrating camera RIGHT .. ";
    camera_calibration_matrix_right = cv::initCameraMatrix2D(object_points_per_image, image_points_per_image_right, image_size, 0);
    std::cerr << "done" << std::endl;
  }

  //ds rectification configuration
  cv::Mat rotation_camera_left_to_right(3, 3, CV_64F, cv::Scalar(0));
  cv::Mat translation_camera_left_to_right(3, 1, CV_64F, cv::Scalar(0));
  cv::Mat depth_mapping(4, 4, CV_64F, cv::Scalar(0));
  cv::Mat projection_matrix_left(cv::Mat::eye(3, 4, CV_64F));
  cv::Mat projection_matrix_right(cv::Mat::eye(3, 4, CV_64F));
  cv::Mat rectification_matrix_left(3, 3, CV_64F, cv::Scalar(0));
  cv::Mat rectification_matrix_right(3, 3, CV_64F, cv::Scalar(0));
  cv::Mat essential_matrix;
  cv::Mat fundamental_matrix;
  double reprojection_error_pixels = 0;

  if (option_test_orb_slam_calibration) {

    //ds update values
    rectification_matrix_left.at<double>(0,0) = 0.999966347530033;
    rectification_matrix_left.at<double>(0,1) = -0.001422739138722922;
    rectification_matrix_left.at<double>(0,2) = 0.008079580483432283;
    rectification_matrix_left.at<double>(1,0) = 0.001365741834644127;
    rectification_matrix_left.at<double>(1,1) = 0.9999741760894847;
    rectification_matrix_left.at<double>(1,2) = 0.007055629199258132;
    rectification_matrix_left.at<double>(2,0) = -0.008089410156878961;
    rectification_matrix_left.at<double>(2,1) = -0.007044357138835809;
    rectification_matrix_left.at<double>(2,2) = 0.9999424675829176;

    rectification_matrix_right.at<double>(0,0) = 0.9999633526194376;
    rectification_matrix_right.at<double>(0,1) = -0.003625811871560086;
    rectification_matrix_right.at<double>(0,2) = 0.007755443660172947;
    rectification_matrix_right.at<double>(1,0) = 0.003680398547259526;
    rectification_matrix_right.at<double>(1,1) = 0.9999684752771629;
    rectification_matrix_right.at<double>(1,2) = -0.007035845251224894;
    rectification_matrix_right.at<double>(2,0) = -0.007729688520722713;
    rectification_matrix_right.at<double>(2,1) = 0.007064130529506649;
    rectification_matrix_right.at<double>(2,2) = 0.999945173484644;

    projection_matrix_left.at<double>(0,0) = 435.2046959714599;
    projection_matrix_left.at<double>(0,2) = 367.4517211914062;
    projection_matrix_left.at<double>(1,1) = 435.2046959714599;
    projection_matrix_left.at<double>(1,2) = 252.2008514404297;

    projection_matrix_right.at<double>(0,0) = 435.2046959714599;
    projection_matrix_right.at<double>(0,2) = 367.4517211914062;
    projection_matrix_right.at<double>(0,3) = -47.90639384423901;
    projection_matrix_right.at<double>(1,1) = 435.2046959714599;
    projection_matrix_right.at<double>(1,2) = 252.2008514404297;
  } else {
    std::cerr << "\ncalibrating stereo camera .. ";
#if CV_MAJOR_VERSION == 2
    reprojection_error_pixels = cv::stereoCalibrate(object_points_per_image,
                                                    image_points_per_image_left,
                                                    image_points_per_image_right,
                                                    camera_calibration_matrix_left,
                                                    distortion_coefficients_left,
                                                    camera_calibration_matrix_right,
                                                    distortion_coefficients_right,
                                                    image_size,
                                                    rotation_camera_left_to_right,
                                                    translation_camera_left_to_right,
                                                    essential_matrix,
                                                    fundamental_matrix,
                                                    cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6),
                                                    CV_CALIB_USE_INTRINSIC_GUESS);
#elif CV_MAJOR_VERSION == 3
  reprojection_error_pixels = cv::stereoCalibrate(object_points_per_image,
                                                  image_points_per_image_left,
                                                  image_points_per_image_right,
                                                  camera_calibration_matrix_left,
                                                  distortion_coefficients_left,
                                                  camera_calibration_matrix_right,
                                                  distortion_coefficients_right,
                                                  image_size,
                                                  rotation_camera_left_to_right,
                                                  translation_camera_left_to_right,
                                                  essential_matrix,
                                                  fundamental_matrix,
                                                  CV_CALIB_USE_INTRINSIC_GUESS);
#else
  #error OpenCV version not supported
#endif

    std::cerr << "reprojection error (Pixels): " << reprojection_error_pixels << std::endl;
    std::cerr << "\nrotation camera LEFT to RIGHT: \n" << std::endl;
    std::cerr << rotation_camera_left_to_right << std::endl;
    std::cerr << "\ntranslation camera LEFT to RIGHT: \n" << std::endl;
    std::cerr << translation_camera_left_to_right << std::endl;

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
    std::printf(" LEFT: %9.6f, %9.6f, %9.6f, %9.6f\n",
                distortion_coefficients_left.at<double>(0),
                distortion_coefficients_left.at<double>(1),
                distortion_coefficients_left.at<double>(2),
                distortion_coefficients_left.at<double>(3));
    std::printf("RIGHT: %9.6f, %9.6f, %9.6f, %9.6f\n",
                distortion_coefficients_right.at<double>(0),
                distortion_coefficients_right.at<double>(1),
                distortion_coefficients_right.at<double>(2),
                distortion_coefficients_right.at<double>(3));

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
  }

  //ds display the obtained projection matrices
  std::cerr << "\nprojection matrices: \n" << std::endl;
  std::cerr << "                 LEFT                 -                 RIGHT" << std::endl;
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

  //ds feature handling
#if CV_MAJOR_VERSION == 2
  cv::FeatureDetector* keypoint_detector        = new cv::FastFeatureDetector();
  cv::DescriptorExtractor* descriptor_extractor = new cv::BriefDescriptorExtractor();
#elif CV_MAJOR_VERSION == 3
  cv::Ptr<cv::FastFeatureDetector> keypoint_detector    = cv::FastFeatureDetector::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
#else
  #error OpenCV version not supported
#endif

  //ds load test images for left and right - assuming to be synchronized
  const std::vector<Image> images_left_test(getImagesASL(folder_images_left_test));
  const std::vector<Image> images_right_test(getImagesASL(folder_images_right_test));

  //ds restart the stream to check the found parameters
  double accumulated_relative_epipolar_matches = 0;
  number_of_processed_stereo_images            = 0;
  for (uint32_t image_number = 0; image_number < number_of_images; ++image_number) {

    //ds compute timestamp delta
    const double timestamp_difference_seconds = std::fabs(images_left_test[image_number].timestamp_seconds-images_right_test[image_number].timestamp_seconds);

    //ds if we have a synchronized package of sensor messages ready
    if (timestamp_difference_seconds < maximum_timestamp_difference_seconds) {

      //ds grab opencv image data
      cv::Mat image_left  = cv::imread(images_left_test[image_number].file_name_image, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat image_right = cv::imread(images_right_test[image_number].file_name_image, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat image_display_left, image_display_right;

      //ds undistort and rectify
      cv::Mat image_left_undistorted_rectified;
      cv::Mat image_right_undistorted_rectified;
      cv::remap(image_left, image_left_undistorted_rectified, undistort_rectify_maps_left[0], undistort_rectify_maps_left[1], cv::INTER_LINEAR);
      cv::remap(image_right, image_right_undistorted_rectified, undistort_rectify_maps_right[0], undistort_rectify_maps_right[1], cv::INTER_LINEAR);
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

        //ds draw some horizontal lines for qualtiative analysis of the rectification
        for (uint32_t u = 0; u < 10; ++u) {
          const int32_t row = u*image_size.height/10.0;
          cv::line(image_stereo, cv::Point2f(0, row), cv::Point2f(2*image_size.width, row), cv::Scalar(0, 0, 255), 1);
        }
        cv::imshow("benchmarking", image_stereo);
        cv::waitKey(0);
      }

      //ds status
      std::printf("%06lu|L: %f|R: %f|keypoints L: %5lu keypoints R: %5lu "
                  "STEREO MATCHES: %5lu (%5.3f)\n", number_of_processed_stereo_images,
                                                    images_left_test[image_number].timestamp_seconds, images_right_test[image_number].timestamp_seconds,
                                                    keypoints_left.size(), keypoints_right.size(),
                                                    number_of_epipolar_matches, static_cast<double>(number_of_epipolar_matches)/keypoints_left.size());
      ++number_of_processed_stereo_images;
      accumulated_relative_epipolar_matches += static_cast<double>(number_of_epipolar_matches)/keypoints_left.size();
    }
  }
  cv::destroyAllWindows();

  const double average_number_of_relative_epipolar_matches = static_cast<double>(accumulated_relative_epipolar_matches)/number_of_processed_stereo_images;
  std::cerr << "\nbenchmark completed - average number of relative epipolar matches per stereo image pair: " << average_number_of_relative_epipolar_matches << std::endl;
  std::cerr << "saving results to: " << output_file_name << std::endl;

  //ds save calibration to file
  std::ofstream calibration_file(output_file_name);
  calibration_file << std::fixed;
  calibration_file << std::setprecision(9);
  calibration_file << "stereo camera calibration" << std::endl;
  calibration_file << "\nnumber_of_processed_stereo_measurements: " << object_points_per_image.size() << std::endl;
  calibration_file << "reprojection_error_pixels: " << reprojection_error_pixels << std::endl;
  calibration_file << "average_number_of_relative_epipolar_matches: " << average_number_of_relative_epipolar_matches << std::endl;
  calibration_file << "\nimage_size: " << std::endl;
  calibration_file << image_size.width << " " << image_size.height << std::endl;
  calibration_file << "\ncamera_calibration_matrix_left: " << std::endl;
  for (uint32_t r = 0; r < 3; ++r) {
    for (uint32_t c = 0; c < 3; ++c) {
      calibration_file << camera_calibration_matrix_left.at<double>(r,c) << " ";
    }
    calibration_file << std::endl;
  }
  calibration_file << "\ncamera_calibration_matrix_right: " << std::endl;
  for (uint32_t r = 0; r < 3; ++r) {
    for (uint32_t c = 0; c < 3; ++c) {
      calibration_file << camera_calibration_matrix_right.at<double>(r,c) << " ";
    }
    calibration_file << std::endl;
  }
  calibration_file << "\ndistortion_coefficients_left: " << std::endl;
  for (uint32_t r = 0; r < 4; ++r) {
    calibration_file << distortion_coefficients_left.at<double>(r) << " ";
  }
  calibration_file << std::endl;
  calibration_file << "\ndistortion_coefficients_right: " << std::endl;
  for (uint32_t r = 0; r < 4; ++r) {
    calibration_file << distortion_coefficients_right.at<double>(r) << " ";
  }
  calibration_file << std::endl;
  calibration_file << "\nprojection_matrix_left: " << std::endl;
  for (uint32_t r = 0; r < 3; ++r) {
    for (uint32_t c = 0; c < 4; ++c) {
      calibration_file << projection_matrix_left.at<double>(r,c) << " ";
    }
    calibration_file << std::endl;
  }
  calibration_file << "\nprojection_matrix_right: " << std::endl;
  for (uint32_t r = 0; r < 3; ++r) {
    for (uint32_t c = 0; c < 4; ++c) {
      calibration_file << projection_matrix_right.at<double>(r,c) << " ";
    }
    calibration_file << std::endl;
  }
  calibration_file << "\nrectification_matrix_left: " << std::endl;
  for (uint32_t r = 0; r < 3; ++r) {
    for (uint32_t c = 0; c < 3; ++c) {
      calibration_file << rectification_matrix_left.at<double>(r,c) << " ";
    }
    calibration_file << std::endl;
  }
  calibration_file << "\nrectification_matrix_right: " << std::endl;
  for (uint32_t r = 0; r < 3; ++r) {
    for (uint32_t c = 0; c < 3; ++c) {
      calibration_file << rectification_matrix_right.at<double>(r,c) << " ";
    }
    calibration_file << std::endl;
  }
  calibration_file << std::endl;
  calibration_file.close();

#if CV_MAJOR_VERSION == 2
  delete keypoint_detector;
  delete descriptor_extractor;
#endif

  return 0;
}

const std::vector<Image> getImagesASL(const std::string& folder_images_) {

  //ds stamped images
  std::vector<Image> images;

  //ds load data file (comma separated values)
  std::ifstream file_images((folder_images_+"/data.csv").c_str());

  //ds image path
  const std::string image_path(folder_images_+"/data/");

  //ds read line by line
  std::string buffer_line;
  while (std::getline(file_images, buffer_line)) {

    //ds skip comment lines
    if (buffer_line[0] == '#') {
      continue;
    }

    //ds parse control
    std::string::size_type index_begin_item = 0;
    std::string::size_type index_end_item   = 0;

    //ds parse timestamp - cutting off the 6 last digits (from nanoseconds to milliseconds)
    index_end_item = buffer_line.find(",", index_begin_item);
    const uint64_t timestamp_milliseconds = std::atol(buffer_line.substr(index_begin_item, index_end_item-6).c_str());
    index_begin_item = index_end_item+1;

    //ds parse image name and store it
    images.push_back(Image(timestamp_milliseconds, image_path+buffer_line.substr(index_begin_item, buffer_line.length()-index_begin_item-1)));
  }
  return images;
}

const bool measure(const cv::Mat& image_,
                   const cv::Size& board_size_,
                   const double& square_width_meters_,
                   std::vector<cv::Point2f>& image_points_,
                   cv::Mat& image_display_) {

  //ds locate chessboard
  bool found_chessboard = cv::findChessboardCorners(image_, board_size_, image_points_,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

  //ds return on failure
  if (!found_chessboard) {
    return false;
  }

  //ds refine corner locations
  cv::cornerSubPix(image_, image_points_, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1));

  //ds visual info
  for (const cv::Point2f& point: image_points_) {
    cv::circle(image_display_, point, 5, cv::Scalar(0, 255, 0), 2);
  }

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
