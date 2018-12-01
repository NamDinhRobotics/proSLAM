#include <fstream>
#include "framepoint_generation/stereo_framepoint_generator.h"
using namespace proslam;



//ds helpers
Eigen::Matrix3d getCameraCalibrationMatrixKITTI(const std::string& file_name_calibration_, Eigen::Vector3d& baseline_pixels_);



int32_t main(int32_t argc_, char** argv_) {

  //ds validate input
  if (argc_ < 5) {
    std::cerr << "ERROR: invalid call - please use: ./test_stereo_framepoint_generator <file_name_initial_image_LEFT> <file_name_initial_image_RIGHT> "
                 "<calib.txt> <fast_threshold> [<gt.txt>]" << std::endl;
    return 0;
  }

  //ds configuration - TODO proper input validation
  const std::string file_name_initial_image_left  = argv_[1];
  const std::string file_name_initial_image_right = argv_[2];
  const std::string file_name_calibration         = argv_[3];
  const uint32_t fast_detector_threshold          = std::stoi(argv_[4]);
  std::cerr << BAR << std::endl;
  std::cerr << "initial image LEFT: " << file_name_initial_image_left << std::endl;
  std::cerr << "initial image RIGHT: " << file_name_initial_image_right << std::endl;
  std::cerr << "calibration file (KITTI): " << file_name_calibration << std::endl;
  std::cerr << "fast detector threshold: " << fast_detector_threshold << std::endl;

  //ds check optional poses
  int32_t range_point_tracking_pixels = 50; //ds maximum projection regional tracking range
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses_left_camera_in_world(0);
  if (argc_ > 5) {
    const std::string file_name_poses = argv_[5];
    std::cerr << "trajectory file (KITTI): " << file_name_poses << std::endl;

    //ds load poses_left_camera_in_world
    std::ifstream ground_truth(file_name_poses);
    std::string line_buffer("");
    while(std::getline(ground_truth, line_buffer)) {
      std::istringstream stream(line_buffer);
      if (line_buffer.empty()) {
        break;
      }

      //ds parse pose matrix directly from file (KITTI format)
      Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
      for(uint8_t u = 0; u < 3; ++u) {
        for(uint8_t v = 0; v < 4; ++v) {
          stream >> pose(u,v);
        }
      }
      poses_left_camera_in_world.push_back(pose);
    }
    std::cerr << "loaded poses: " << poses_left_camera_in_world.size() << std::endl;
    range_point_tracking_pixels /= 5;
    std::cerr << "using tracking range: " << range_point_tracking_pixels << std::endl;
  }
  std::cerr << BAR << std::endl;

  //ds allocate stereo framepoint generator parameters
  StereoFramePointGeneratorParameters* parameters = new StereoFramePointGeneratorParameters(LoggingLevel::Debug);

  //ds allocate and configure a stereo framepoint generator unit
  StereoFramePointGenerator* framepoint_generator = new StereoFramePointGenerator(parameters);

  //ds load camera matrix and stereo configuration (offset to camera right)
  Eigen::Vector3d baseline_pixels_(Eigen::Vector3d::Zero());
  const Eigen::Matrix3d camera_calibration_matrix(getCameraCalibrationMatrixKITTI(file_name_calibration, baseline_pixels_));

  //ds parse image extension from left image
  const std::size_t index_delimiter_extension = file_name_initial_image_left.find_last_of('.');
  const std::string extension                 = file_name_initial_image_left.substr(index_delimiter_extension, file_name_initial_image_left.length()-index_delimiter_extension);
  std::cerr << "using image extension: '" << extension << "'" << std::endl;

  //ds parse folders - UNIX only
  const std::size_t index_delimiter_directory_left = file_name_initial_image_left.find_last_of('/');
  const std::string directory_name_images_left     = file_name_initial_image_left.substr(0, index_delimiter_directory_left+1);
  std::cerr << "reading images from directory: '" << directory_name_images_left << "'" << std::endl;
  const std::size_t index_delimiter_directory_right = file_name_initial_image_right.find_last_of('/');
  const std::string directory_name_images_right     = file_name_initial_image_right.substr(0, index_delimiter_directory_right+1);
  std::cerr << "reading images from directory: '" << directory_name_images_right << "'" << std::endl;

  //ds parse enumeration characters range (as file name of images)
  const uint32_t number_of_enumeration_characters = index_delimiter_extension-index_delimiter_directory_left-1;
  std::cerr << "using enumeration pattern: '";
  for (uint32_t u = 0; u < number_of_enumeration_characters; ++u) {
    std::cerr << u;
  }
  std::cerr << extension << "'" << std::endl;

  //ds configure stereo framepoint generator
  parameters->enable_keypoint_binning                 = true; //ds enable keypoint binning
  parameters->number_of_detectors_horizontal          = 1;
  parameters->number_of_detectors_vertical            = 1;
  parameters->detector_threshold_minimum              = fast_detector_threshold;
  parameters->detector_threshold_maximum              = fast_detector_threshold;
  parameters->minimum_projection_tracking_distance_pixels = range_point_tracking_pixels;
  parameters->maximum_projection_tracking_distance_pixels = range_point_tracking_pixels;
  parameters->matching_distance_tracking_threshold    = 25; //ds number of mismatching bits
  parameters->maximum_matching_distance_triangulation = 35; //ds number of mismatching bits
  parameters->maximum_epipolar_search_offset_pixels   = 3;  //ds number of horizontal epipolar lines to check for triangulation

  //ds load initial images in monochrome
  uint32_t number_of_processed_images = 0;
  std::string file_name_image_left  = file_name_initial_image_left;
  std::string file_name_image_right = file_name_initial_image_right;
  cv::Mat image_left                = cv::imread(file_name_image_left, CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat image_right               = cv::imread(file_name_image_right, CV_LOAD_IMAGE_GRAYSCALE);

  //ds allocate cameras and configure stereo point generator
  Camera* camera_left  = new Camera(image_left.rows, image_left.cols, camera_calibration_matrix);
  Camera* camera_right = new Camera(image_right.rows, image_right.cols, camera_calibration_matrix);
  camera_right->setBaselineHomogeneous(baseline_pixels_);
  framepoint_generator->setCameraLeft(camera_left);
  framepoint_generator->setCameraRight(camera_right);
  framepoint_generator->configure();

  //ds structure from previous frame (for tracking test)
  Frame* frame_previous = nullptr;
  cv::Mat image_left_previous;
  bool enable_auto_playback = false;

  //ds process images
  while (image_left.rows > 0 && image_left.cols > 0 &&
         image_left.rows == image_right.rows && image_left.cols == image_right.cols) {
    const int32_t rows = image_left.rows;
    const int32_t cols = image_left.cols;

    //ds allocate a new, empty frame
    Frame* frame = new Frame(nullptr, frame_previous, nullptr, TransformMatrix3D::Identity(), 0);
    frame->setCameraLeft(camera_left);
    frame->setCameraRight(camera_right);
    frame->setIntensityImageLeft(image_left);
    frame->setIntensityImageRight(image_right);

    //ds initialize matchers for current measurements
    framepoint_generator->initialize(frame);

    //ds if we can track (i.e. have a previous image with points)
    if (frame_previous) {

      //ds retrieve relative motion (usually obtained through ICP) - identity if not available
      Eigen::Isometry3d camera_left_previous_in_current(Eigen::Isometry3d::Identity());
      if (poses_left_camera_in_world.size() > number_of_processed_images) {
        camera_left_previous_in_current = poses_left_camera_in_world[number_of_processed_images].inverse()*
                                          poses_left_camera_in_world[number_of_processed_images-1];
      }

      FramePointPointerVector lost_points(0);
      framepoint_generator->track(frame,
                                  frame_previous,
                                  camera_left_previous_in_current,
                                  lost_points);

      //ds remove matched indices from candidate pools
      std::cerr << "found tracks with matches total: " << frame->points().size() << std::endl;
      std::cerr << "lost points: " << lost_points.size() << std::endl;
    }

    //ds find stereo matches and triangulate -> create Framepoints
    framepoint_generator->compute(frame);
    std::cerr << "total points: " << frame->points().size() << std::endl;

    //ds display current points
    cv::Mat image_display_stereo;
    cv::hconcat(image_left, image_right, image_display_stereo);
    cv::cvtColor(image_display_stereo, image_display_stereo, CV_GRAY2RGB);
    const cv::Point2f shift_horizontal(cols, 0);
    const cv::Point2f shift_text(shift_horizontal+cv::Point2f(5, 5));
    for (const FramePoint* point: frame->points()) {
      const cv::Scalar color = CV_COLOR_CODE_RANDOM;
      cv::circle(image_display_stereo, point->keypointLeft().pt, 2, color, -1);
      cv::circle(image_display_stereo, point->keypointRight().pt+shift_horizontal, 2, color, -1);
      cv::putText(image_display_stereo, std::to_string(point->identifier())+" | "+std::to_string(point->epipolarOffset()),
                  point->keypointLeft().pt+cv::Point2f(5, 5), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.25, CV_COLOR_CODE_RED);
      cv::putText(image_display_stereo, std::to_string(point->identifier())+" | "+std::to_string(point->epipolarOffset()),
                  point->keypointRight().pt+shift_text, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.25, CV_COLOR_CODE_RED);
    }
    std::cerr << "processed images: " << file_name_image_left << " - " << file_name_image_right << " (" << number_of_processed_images << ")" << std::endl;

    //ds display projected depth
    cv::Mat image_display_depth;
    cv::hconcat(image_left, image_right, image_display_depth);
    cv::cvtColor(image_display_depth, image_display_depth, CV_GRAY2RGB);
    image_display_depth.setTo(0);
    for (const FramePoint* point: frame->points()) {

      //ds project in left and right camera
      Eigen::Vector3d uv_L(camera_calibration_matrix*point->cameraCoordinatesLeft());
      Eigen::Vector3d uv_R(uv_L+baseline_pixels_);
      uv_R /= uv_R.z();
      uv_L /= uv_L.z();

      //ds brighten color the closer to the camera the measurement is (based on maximum depth)
      const cv::Scalar color = cv::Scalar(0, 0, 255*(1+point->depthMeters()/baseline_pixels_.x()));
      cv::circle(image_display_depth, cv::Point2f(uv_L(0), uv_L(1)), 2, color, -1);
      cv::circle(image_display_depth, cv::Point2f(uv_R(0), uv_R(1))+shift_horizontal, 2, color, -1);
    }
    if (frame_previous) {
      cv::Mat image_display_tracks;
      cv::vconcat(image_left, image_left_previous, image_display_tracks);
      cv::cvtColor(image_display_tracks, image_display_tracks, CV_GRAY2RGB);
      const cv::Point2f shift_vertical(0, rows);
      const cv::Point2f shift_horizontal(cols, 0);

      //ds for each point
      for (const FramePoint* point: frame->points()) {

        //ds that is tracked (i.e. has a previous point)
        const FramePoint* point_previous = point->previous();
        if (point_previous) {

          //ds highlight tracks
          const cv::Point2f projection_offset(point->keypointLeft().pt-point_previous->keypointLeft().pt);
          cv::line(image_display_tracks, point->keypointLeft().pt, point_previous->keypointLeft().pt+shift_vertical, CV_COLOR_CODE_BLUE);
          cv::line(image_display_tracks, point->keypointLeft().pt, point->keypointLeft().pt+projection_offset, CV_COLOR_CODE_RED);

          //ds highlight stereo match prediction and projection error compensation
          cv::circle(image_display_depth, point->projectionEstimateLeft(), 4, CV_COLOR_CODE_BLUE, 1);
          cv::circle(image_display_depth, point->projectionEstimateRight()+shift_horizontal, 4, CV_COLOR_CODE_BLUE, 1);
          cv::circle(image_display_depth, point->projectionEstimateRightCorrected()+shift_horizontal, 4, CV_COLOR_CODE_GREEN, 1);
          cv::line(image_display_depth, point->projectionEstimateRight()+shift_horizontal, point->projectionEstimateRightCorrected()+shift_horizontal, CV_COLOR_CODE_RED);
        }
      }
      cv::imshow("left image tracks [top: current, bot: previous]", image_display_tracks);
    }
    cv::imshow("feature extraction", image_display_stereo);
    cv::imshow("stereo depth", image_display_depth);

    //ds check if we have to switch to playback mode or escape
    int key = cv::waitKey(1);
    if (!enable_auto_playback) {
      key = cv::waitKey(0);
    }
    if (key == 32) {
      enable_auto_playback = !enable_auto_playback;
    }

    //ds release previous
    delete frame_previous;

    //ds if termination is requested
    if (key == 27) {
      std::cerr << "termination requested" << std::endl;
      delete frame;
      break;
    }

    //ds update previous with current
    frame_previous      = frame;
    image_left_previous = image_left;

    //ds compute file name for next images
    ++number_of_processed_images;
    std::string file_name_tail             = std::to_string(number_of_processed_images);
    const uint32_t number_of_padding_zeros = number_of_enumeration_characters-file_name_tail.length();
    for (uint32_t u = 0; u < number_of_padding_zeros; ++u) {
      file_name_tail = "0"+file_name_tail;
    }
    file_name_image_left  = directory_name_images_left+file_name_tail+extension;
    file_name_image_right = directory_name_images_right+file_name_tail+extension;

    //ds read next images
    image_left.release();
    image_right.release();
    image_left  = cv::imread(file_name_image_left, CV_LOAD_IMAGE_GRAYSCALE);
    image_right = cv::imread(file_name_image_right, CV_LOAD_IMAGE_GRAYSCALE);
  }

  //ds clean up
  delete camera_left;
  delete camera_right;
  delete framepoint_generator;
  delete parameters;
  return 0;
}

Eigen::Matrix3d getCameraCalibrationMatrixKITTI(const std::string& file_name_calibration_, Eigen::Vector3d& baseline_pixels_) {

  //ds load camera matrix - for now only KITTI parsing
  std::ifstream file_calibration(file_name_calibration_, std::ifstream::in);
  std::string line_buffer("");
  std::getline(file_calibration, line_buffer);
  if (line_buffer.empty()) {
    throw std::runtime_error("invalid camera calibration file provided");
  }
  std::istringstream stream_left(line_buffer);
  Eigen::Matrix3d camera_calibration_matrix(Eigen::Matrix3d::Identity());
  baseline_pixels_.setZero();

  //ds parse in fixed order
  std::string filler(""); stream_left >> filler;
  stream_left >> camera_calibration_matrix(0,0);
  stream_left >> filler;
  stream_left >> camera_calibration_matrix(0,2);
  stream_left >> filler; stream_left >> filler;
  stream_left >> camera_calibration_matrix(1,1);
  stream_left >> camera_calibration_matrix(1,2);

  //ds read second projection matrix to obtain the horizontal offset
  std::getline(file_calibration, line_buffer);
  std::istringstream stream_right(line_buffer);
  stream_right >> filler; stream_right >> filler; stream_right >> filler; stream_right >> filler;
  stream_right >> baseline_pixels_(0);
  file_calibration.close();
  std::cerr << "loaded camera calibration matrix: \n" << camera_calibration_matrix << std::endl;
  std::cerr << "with baseline (pixels): " << baseline_pixels_.transpose() << std::endl;
  return camera_calibration_matrix;
}
