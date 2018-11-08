#include "types/definitions.h"



//ds map objects
struct KeypointWithDescriptor {
  KeypointWithDescriptor(const cv::KeyPoint& keypoint_,
                         const cv::Mat& descriptor_,
                         const uint32_t& index_in_vector_): keypoint(keypoint_),
                                                            descriptor(descriptor_),
                                                            row(keypoint_.pt.y),
                                                            col(keypoint_.pt.x),
                                                            index_in_vector(index_in_vector_) {}
  cv::KeyPoint keypoint;    //ds geometric: feature location in 2D
  cv::Mat descriptor;       //ds appearance: feature descriptor
  int32_t row;              //ds pixel column coordinate (v)
  int32_t col;              //ds pixel row coordinate (u)
  uint32_t index_in_vector; //ds inverted index for vector containing this
};

struct KeypointWithDescriptorLattice {
  ~KeypointWithDescriptorLattice() {
    for (uint32_t r = 0; r < number_of_rows; ++r) {
      delete[] feature_lattice[r];
    }
    delete[] feature_lattice;
  }

  void setLattice(const int32_t& rows_, const int32_t& cols_) {
    if (rows_ <= 0 || cols_ <= 0) {
      throw std::runtime_error("setStereoMatchCandidateGrid|invalid image dimensions");
    }
    if (feature_lattice) {
      throw std::runtime_error("setStereoMatchCandidateGrid|lattice already allocated");
    }

    //ds initialize empty lattice
    feature_lattice = new KeypointWithDescriptor**[rows_];
    for (int32_t r = 0; r < rows_; ++r) {
      feature_lattice[r] = new KeypointWithDescriptor*[cols_];
      for (int32_t c = 0; c < cols_; ++c) {
        feature_lattice[r][c] = nullptr;
      }
    }
    number_of_rows = rows_;
    number_of_cols = cols_;
  }

  void setFeatures(const std::vector<cv::KeyPoint>& keypoints_, const cv::Mat& descriptors_) {
    if (keypoints_.size() != static_cast<size_t>(descriptors_.rows)) {
      throw std::runtime_error("setFeatures|mismatching keypoints and descriptor numbers");
    }

    //ds clear the lattice
    for (uint32_t r = 0; r < number_of_rows; ++r) {
      for (uint32_t c = 0; c < number_of_cols; ++c) {
        feature_lattice[r][c] = nullptr;
      }
    }

    //ds fill in features
    number_of_features = 0;
    feature_vector.resize(keypoints_.size());
    for (uint32_t index = 0; index < keypoints_.size(); ++index) {
      KeypointWithDescriptor* feature = new KeypointWithDescriptor(keypoints_[index], descriptors_.row(index), index);
      feature_vector[index] = feature;
      feature_lattice[feature->row][feature->col] = feature;
      ++number_of_features;
    }

    //ds sort all input vectors by ascending row positions
    std::sort(feature_vector.begin(), feature_vector.end(),  [](const KeypointWithDescriptor* a_, const KeypointWithDescriptor* b_){
      return ((a_->row < b_->row) || (a_->row == b_->row && a_->col < b_->col));
    });
  }

  uint32_t number_of_rows = 0;
  uint32_t number_of_cols = 0;
  std::vector<KeypointWithDescriptor*> feature_vector;
  KeypointWithDescriptor*** feature_lattice = nullptr;
  uint32_t number_of_features               = 0;
};

struct Framepoint {
  Framepoint(const uint32_t& identifer_,
             const KeypointWithDescriptor* feature_left_,
             const KeypointWithDescriptor* feature_right_,
             const int32_t& epipolar_offset_,
             const Eigen::Vector3d& position_in_camera_left_): identifer(identifer_),
                                                               feature_left(feature_left_),
                                                               feature_right(feature_right_),
                                                               epipolar_offset(epipolar_offset_),
                                                               position_in_camera_left(position_in_camera_left_) {}

  ~Framepoint() {
    delete feature_left;
    delete feature_right;
  }

  uint32_t identifer;
  const KeypointWithDescriptor* feature_left;
  const KeypointWithDescriptor* feature_right;
  int32_t epipolar_offset;

  Eigen::Vector3d position_in_camera_left;
};
typedef std::vector<Framepoint*> FramepointVector;

//ds helpers
Eigen::Matrix3d getCameraCalibrationMatrixKITTI(const std::string& file_name_calibration_, Eigen::Vector3d& baseline_pixels_);
void stereoMatchExhaustive(const KeypointWithDescriptorLattice& candidates_left_,
                           const KeypointWithDescriptorLattice& candidates_right_,
                           FramepointVector& stereo_framepoints_,
                           uint32_t& number_of_stereo_matches_,
                           const double& descriptor_distance_maximum_,
                           const uint32_t& epipolar_search_height_,
                           const double& f_x, const double& f_y,
                           const double& c_x, const double& c_y,
                           const double& b_x_);
void prune(std::vector<KeypointWithDescriptor*>& elements_, const std::set<uint32_t>& matched_indices_);
KeypointWithDescriptor* getMatchingFeatureInRegion(const int32_t& row_reference_,
                                                   const int32_t& col_reference_,
                                                   const cv::Mat& descriptor_reference_,
                                                   const int32_t& range_point_tracking_pixels_rows_,
                                                   const int32_t& range_point_tracking_pixels_cols_,
                                                   const int32_t& image_rows_,
                                                   const int32_t& image_cols_,
                                                   KeypointWithDescriptor*** candidates_,
                                                   const double& maximum_descriptor_distance_tracking_);
Eigen::Vector3d getPointInLeftCamera(const double& u_L, const double v_L,
                                     const double& u_R, const double v_R,
                                     const double& f_x, const double& f_y,
                                     const double& c_x, const double& c_y,
                                     const double& b_x_);



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

  //ds miscellaneous configuration
  const uint32_t maximum_descriptor_distance_tracking      = 25; //ds number of mismatching bits
  const uint32_t maximum_descriptor_distance_triangulation = 35; //ds number of mismatching bits
  const uint32_t epipolar_search_height                    = 3;  //ds number of horizontal epipolar lines to check for triangulation

  //ds feature handling
#if CV_MAJOR_VERSION == 2
  cv::Ptr<cv::FeatureDetector> keypoint_detector        = new cv::FastFeatureDetector(fast_detector_threshold);
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor = new cv::ORB();
#elif CV_MAJOR_VERSION == 3
  cv::Ptr<cv::FeatureDetector> keypoint_detector        = cv::FastFeatureDetector::create(fast_detector_threshold);
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor = cv::ORB::create();
#endif

  //ds load initial images in monochrome
  uint32_t number_of_processed_images = 0;
  std::string file_name_image_left  = file_name_initial_image_left;
  std::string file_name_image_right = file_name_initial_image_right;
  cv::Mat image_left                = cv::imread(file_name_image_left, CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat image_right               = cv::imread(file_name_image_right, CV_LOAD_IMAGE_GRAYSCALE);

  //ds helper structure: lattice of keypoints with descriptors (for convenience)
  KeypointWithDescriptorLattice candidates_left;
  candidates_left.setLattice(image_left.rows, image_left.cols);
  KeypointWithDescriptorLattice candidates_right;
  candidates_right.setLattice(image_right.rows, image_right.cols);

  //ds structure from previous frame (for tracking test)
  FramepointVector stereo_points_previous;
  cv::Mat image_left_previous;
  bool enable_auto_playback = false;

  //ds process images
  while (image_left.rows > 0 && image_left.cols > 0 &&
         image_left.rows == image_right.rows && image_left.cols == image_right.cols) {
    const int32_t rows = image_left.rows;
    const int32_t cols = image_left.cols;

    //ds detect FAST keypoints
    std::vector<cv::KeyPoint> keypoints_left;
    std::vector<cv::KeyPoint> keypoints_right;
    keypoint_detector->detect(image_left, keypoints_left);
    keypoint_detector->detect(image_right, keypoints_right);

    //ds compute BRIEF descriptors
    cv::Mat descriptors_left;
    cv::Mat descriptors_right;
    descriptor_extractor->compute(image_left, keypoints_left, descriptors_left);
    descriptor_extractor->compute(image_right, keypoints_right, descriptors_right);

    //ds connect keypoints with descriptors in a feature object and store it in our working lattice
    candidates_left.setFeatures(keypoints_left, descriptors_left);
    candidates_right.setFeatures(keypoints_right, descriptors_right);

    //ds grab next pose from ground truth
    Eigen::Isometry3d pose_current(Eigen::Isometry3f::Identity());

    //ds stereo points of current stereo frames
    FramepointVector stereo_points(keypoints_left.size());
    uint32_t number_of_stereo_matches = 0;

    //ds if we can track (i.e. have a previous image with points)
    cv::Mat image_display_tracks;
    cv::Mat image_display_depth;
    cv::hconcat(image_left, image_right, image_display_depth);
    cv::cvtColor(image_display_depth, image_display_depth, CV_GRAY2RGB);
    image_display_depth.setTo(0);
    if (!stereo_points_previous.empty()) {
      std::cerr << BAR << std::endl;
      cv::vconcat(image_left, image_left_previous, image_display_tracks);
      cv::cvtColor(image_display_tracks, image_display_tracks, CV_GRAY2RGB);
      const cv::Point2f shift_vertical(0, rows);
      const cv::Point2f shift_horizontal(cols, 0);

      //ds retrieve relative motion (usually obtained through ICP) - identity if not available
      Eigen::Isometry3d camera_left_previous_in_current(Eigen::Isometry3d::Identity());
      if (poses_left_camera_in_world.size() > number_of_processed_images) {
        camera_left_previous_in_current = poses_left_camera_in_world[number_of_processed_images].inverse()*
                                          poses_left_camera_in_world[number_of_processed_images-1];
      }

      //ds tracked and triangulated features (to not consider them in the exhaustive stereo matching)
      std::set<uint32_t> matched_indices_left;
      std::set<uint32_t> matched_indices_right;

      //ds for each previous point
      for (Framepoint* point_previous: stereo_points_previous) {

        //ds transform the point into the current camera frame
        const Eigen::Vector3d point_in_camera_left(camera_left_previous_in_current*point_previous->position_in_camera_left);

        //ds project the point into the current left image plane
        const Eigen::Vector3d point_in_image_left(camera_calibration_matrix*point_in_camera_left);
        const int32_t col_projection_left = point_in_image_left.x()/point_in_image_left.z();
        const int32_t row_projection_left = point_in_image_left.y()/point_in_image_left.z();

        //ds skip point if not in image plane
        if (col_projection_left < 0 || col_projection_left > cols || row_projection_left < 0 || row_projection_left > rows) {
          continue;
        }

        //ds TRACKING obtain matching feature in left image (if any)
        KeypointWithDescriptor* feature_left = getMatchingFeatureInRegion(row_projection_left,
                                                                          col_projection_left,
                                                                          point_previous->feature_left->descriptor,
                                                                          range_point_tracking_pixels,
                                                                          range_point_tracking_pixels,
                                                                          rows,
                                                                          cols,
                                                                          candidates_left.feature_lattice,
                                                                          maximum_descriptor_distance_tracking);

        //ds if we found a match
        if (feature_left) {

          //ds compute projection offset (i.e. prediction error > optical flow)
          const cv::Point2f projection_offset(col_projection_left-feature_left->keypoint.pt.x, row_projection_left-feature_left->keypoint.pt.y);

          //ds project point into the right image - correcting by the predicition error
          const Eigen::Vector3d point_in_image_right(point_in_image_left+baseline_pixels_);
          const int32_t col_projection_right = point_in_image_right.x()/point_in_image_right.z()-projection_offset.x;
          const int32_t row_projection_right = point_in_image_right.y()/point_in_image_right.z()-projection_offset.y;

          //ds skip point if not in image plane
          if (col_projection_right < 0 || col_projection_right > cols || row_projection_right < 0 || row_projection_right > rows) {
            continue;
          }

          //ds TRIANGULATION: obtain matching feature in right image (if any)
          //ds we reduce the vertical matching space to the epipolar range
          //ds we increase the matching tolerance (2x) since we have a strong prior on location
          KeypointWithDescriptor* feature_right = getMatchingFeatureInRegion(row_projection_right,
                                                                             col_projection_right,
                                                                             feature_left->descriptor,
                                                                             epipolar_search_height,
                                                                             range_point_tracking_pixels,
                                                                             rows,
                                                                             cols,
                                                                             candidates_right.feature_lattice,
                                                                             2*maximum_descriptor_distance_triangulation);

          //ds if we found a match
          if (feature_right) {

            //ds create a stereo match
            stereo_points[number_of_stereo_matches] = new Framepoint(number_of_stereo_matches,
                                                                     feature_left,
                                                                     feature_right,
                                                                     0,
                                                                     getPointInLeftCamera(feature_left->col, feature_left->row,
                                                                                          feature_right->col, feature_right->row,
                                                                                          camera_calibration_matrix(0,0), camera_calibration_matrix(1,1),
                                                                                          camera_calibration_matrix(0,2), camera_calibration_matrix(1,2),
                                                                                          baseline_pixels_(0)));
            ++number_of_stereo_matches;

            //ds block matching in exhaustive matching (later)
            matched_indices_left.insert(feature_left->index_in_vector);
            matched_indices_right.insert(feature_right->index_in_vector);

            //ds remove feature from lattices
            candidates_left.feature_lattice[feature_left->row][feature_left->col]    = nullptr;
            candidates_right.feature_lattice[feature_right->row][feature_right->col] = nullptr;

            //ds visualization
            cv::line(image_display_tracks, feature_left->keypoint.pt, point_previous->feature_left->keypoint.pt+shift_vertical, CV_COLOR_CODE_BLUE);
            cv::line(image_display_tracks, feature_left->keypoint.pt, feature_left->keypoint.pt+projection_offset, CV_COLOR_CODE_RED);
            cv::circle(image_display_depth, feature_left->keypoint.pt, 4, CV_COLOR_CODE_GREEN, 1);
            cv::circle(image_display_depth, feature_right->keypoint.pt+shift_horizontal, 6, CV_COLOR_CODE_GREEN, 1);

            //ds display left/right point with (green) and without correction (blue)
            cv::circle(image_display_depth, cv::Point2f(col_projection_left, row_projection_left), 4, CV_COLOR_CODE_BLUE, 1);
            cv::circle(image_display_depth, cv::Point2f(col_projection_right, row_projection_right)+shift_horizontal, 4, CV_COLOR_CODE_GREEN, 1);
            cv::circle(image_display_depth, cv::Point2f(point_in_image_right.x()/point_in_image_right.z(), point_in_image_right.y()/point_in_image_right.z())+shift_horizontal, 4, CV_COLOR_CODE_BLUE, 1);
          }
        }
      }

      //ds remove matched indices from candidate pools
      prune(candidates_left.feature_vector, matched_indices_left);
      prune(candidates_right.feature_vector, matched_indices_right);
      std::cerr << "trackAndStereoMatch|found tracks with matches total: " << number_of_stereo_matches << "/" << candidates_left.number_of_features << std::endl;
      std::cerr << "trackAndStereoMatch|number of unmatched features L: "
                << candidates_left.feature_vector.size() << " R: " << candidates_right.feature_vector.size() << std::endl;
      std::cerr << BAR << std::endl;
    }

    //ds find stereo matches and triangulate -> create Framepoints
    stereoMatchExhaustive(candidates_left,
                          candidates_right,
                          stereo_points,
                          number_of_stereo_matches,
                          maximum_descriptor_distance_triangulation,
                          epipolar_search_height,
                          camera_calibration_matrix(0,0), camera_calibration_matrix(1,1),
                          camera_calibration_matrix(0,2), camera_calibration_matrix(1,2),
                          baseline_pixels_(0));

    //ds display current points
    cv::Mat image_display_stereo;
    cv::hconcat(image_left, image_right, image_display_stereo);
    cv::cvtColor(image_display_stereo, image_display_stereo, CV_GRAY2RGB);
    const cv::Point2f shift_horizontal(cols, 0);
    for (const KeypointWithDescriptor* feature: candidates_left.feature_vector) {
      cv::circle(image_display_stereo, cv::Point2f(feature->col, feature->row), 2, CV_COLOR_CODE_BLUE, -1);
    }
    for (const KeypointWithDescriptor* feature: candidates_right.feature_vector) {
      cv::circle(image_display_stereo, cv::Point2f(feature->col, feature->row)+shift_horizontal, 2, CV_COLOR_CODE_BLUE, -1);
    }
    const cv::Point2f shift_text(shift_horizontal+cv::Point2f(5, 5));
    for (const Framepoint* point: stereo_points) {
      const cv::Scalar color = CV_COLOR_CODE_RANDOM;
      cv::circle(image_display_stereo, point->feature_left->keypoint.pt, 4, color, 1);
      cv::circle(image_display_stereo, point->feature_right->keypoint.pt+shift_horizontal, 4, color, 1);
      cv::putText(image_display_stereo, std::to_string(point->identifer)+" | "+std::to_string(point->epipolar_offset),
                  point->feature_left->keypoint.pt+cv::Point2f(5, 5), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.25, CV_COLOR_CODE_RED);
      cv::putText(image_display_stereo, std::to_string(point->identifer)+" | "+std::to_string(point->epipolar_offset),
                  point->feature_right->keypoint.pt+shift_text, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.25, CV_COLOR_CODE_RED);
    }
    std::cerr << "processed images: " << file_name_image_left << " - " << file_name_image_right << " (" << number_of_processed_images << ")"
              << " features L: " << candidates_left.number_of_features << " R: " << candidates_right.number_of_features << std::endl;

    //ds display projected depth
    for (const Framepoint* point: stereo_points) {

      //ds project in left and right camera
      Eigen::Vector3d uv_L(camera_calibration_matrix*point->position_in_camera_left);
      Eigen::Vector3d uv_R(uv_L+baseline_pixels_);
      uv_R /= uv_R.z();
      uv_L /= uv_L.z();

      //ds brighten color the closer to the camera the measurement is (based on maximum depth)
      const cv::Scalar color = cv::Scalar(0, 0, 255*(1+point->position_in_camera_left.z()/baseline_pixels_.x()));
      cv::circle(image_display_depth, cv::Point2f(uv_L(0), uv_L(1)), 2, color, -1);
      cv::circle(image_display_depth, cv::Point2f(uv_R(0), uv_R(1))+shift_horizontal, 2, color, -1);
    }
    cv::imshow("feature extraction", image_display_stereo);
    cv::imshow("stereo depth", image_display_depth);
    if (!stereo_points_previous.empty()) {
      cv::imshow("left image tracks [top: current, bot: previous]", image_display_tracks);
    }

    //ds check if we have to switch to playback mode or escape
    int key = cv::waitKey(1);
    if (!enable_auto_playback) {
      key = cv::waitKey(0);
    }
    if (key == 32) {
      enable_auto_playback = !enable_auto_playback;
    }

    //ds release previous
    for (Framepoint* point: stereo_points_previous) {
      delete point;
    }

    //ds if termination is requested
    if (key == 27) {
      std::cerr << "termination requested" << std::endl;
      for (Framepoint* point: stereo_points) {
        delete point;
      }
      return 0;
    }

    //ds update previous with current
    stereo_points_previous = stereo_points;
    image_left_previous    = image_left;

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

void stereoMatchExhaustive(const KeypointWithDescriptorLattice& candidates_left_,
                           const KeypointWithDescriptorLattice& candidates_right_,
                           FramepointVector& stereo_framepoints_,
                           uint32_t& number_of_stereo_matches_,
                           const double& descriptor_distance_maximum_,
                           const uint32_t& epipolar_search_height_,
                           const double& f_x, const double& f_y,
                           const double& c_x, const double& c_y,
                           const double& b_x_) {
  std::cerr << BAR << std::endl;
  const uint32_t number_of_stereo_matches_initial = number_of_stereo_matches_;

  //ds working buffers (for each checked epipolar line they shrink as matches are found)
  std::vector<KeypointWithDescriptor*> features_left(candidates_left_.feature_vector);
  std::vector<KeypointWithDescriptor*> features_right(candidates_right_.feature_vector);

  //ds configure epipolar search ranges (in v in the right image)
  std::vector<int32_t> epipolar_offsets = {0};
  for (uint32_t u = 1; u <= epipolar_search_height_; ++u) {
    epipolar_offsets.push_back(u);
    epipolar_offsets.push_back(-u);
  }

  //ds start stereo matching for all epipolar offsets
  for (const int32_t& epipolar_offset: epipolar_offsets) {

    //ds matched features (to not consider them for the next offset search)
    std::set<uint32_t> matched_indices_left;
    std::set<uint32_t> matched_indices_right;

    //ds running variable
    uint32_t index_R = 0;

    //ds loop over all left keypoints
    for (uint32_t index_L = 0; index_L < features_left.size(); index_L++) {

      //ds if there are no more points on the right to match against - stop
      if (index_R == features_right.size()) {break;}

      //ds the right keypoints are on an lower row - skip left
      while (features_left[index_L]->row < features_right[index_R]->row+epipolar_offset) {
        index_L++; if (index_L == features_left.size()) {break;}
      }
      if (index_L == features_left.size()) {break;}
      KeypointWithDescriptor* feature_left = features_left[index_L];

      //ds the right keypoints are on an upper row - skip right
      while (feature_left->row > features_right[index_R]->row+epipolar_offset) {
        index_R++; if (index_R == features_right.size()) {break;}
      }
      if (index_R == features_right.size()) {break;}

      //ds search bookkeeping
      uint32_t index_search_R         = index_R;
      double descriptor_distance_best = descriptor_distance_maximum_;
      uint32_t index_best_R           = 0;

      //ds scan epipolar line for current keypoint at idx_L - exhaustive
      while (feature_left->row == features_right[index_search_R]->row+epipolar_offset) {

        //ds invalid disparity stop condition
        if (feature_left->col-features_right[index_search_R]->col < 0) {break;}

        //ds compute descriptor distance for the stereo match candidates
        const double descriptor_distance = cv::norm(feature_left->descriptor, features_right[index_search_R]->descriptor, SRRG_PROSLAM_DESCRIPTOR_NORM);
        if(descriptor_distance < descriptor_distance_best) {
          descriptor_distance_best = descriptor_distance;
          index_best_R             = index_search_R;
        }
        index_search_R++; if (index_search_R == features_right.size()) {break;}
      }

      //ds check if something was found
      if (descriptor_distance_best < descriptor_distance_maximum_) {
        KeypointWithDescriptor* feature_right = features_right[index_best_R];

        //ds register match
        stereo_framepoints_[number_of_stereo_matches_] = new Framepoint(number_of_stereo_matches_,
                                                                        feature_left,
                                                                        feature_right,
                                                                        epipolar_offset,
                                                                        getPointInLeftCamera(feature_left->col, feature_left->row,
                                                                                             feature_right->col, feature_right->row,
                                                                                             f_x, f_y, c_x, c_y, b_x_));
        ++number_of_stereo_matches_;

        //ds block further matching against features_right[index_best_R] in a search on offset epipolar lines
        matched_indices_left.insert(index_L);
        matched_indices_right.insert(index_best_R);

        //ds reduce search space (this eliminates all structurally conflicting matches)
        index_R = index_best_R+1;
      }
    }

    //ds remove matched indices from candidate pools
    prune(features_left, matched_indices_left);
    prune(features_right, matched_indices_right);
    std::cerr << "stereoMatchExhaustive|epipolar offset: " << epipolar_offset << " number of unmatched features L: "
              << features_left.size() << " R: " << features_right.size() << std::endl;
  }

  stereo_framepoints_.resize(number_of_stereo_matches_);
  std::cerr << "stereoMatchExhaustive|found matches total: " << number_of_stereo_matches_-number_of_stereo_matches_initial << "/" << candidates_left_.number_of_features << std::endl;
  std::cerr << BAR << std::endl;
}

void prune(std::vector<KeypointWithDescriptor*>& elements_, const std::set<uint32_t>& matched_indices_) {

  //ds remove matched indices from candidate pools
  uint32_t number_of_unmatched_elements = 0;
  for (uint32_t index = 0; index < elements_.size(); ++index) {

    //ds if we haven't matched this index yet
    if (matched_indices_.count(index) == 0) {

      //ds keep the element (this operation is not problemenatic since we do not loop reversely here)
      elements_[number_of_unmatched_elements] = elements_[index];
      ++number_of_unmatched_elements;
    }
  }
  elements_.resize(number_of_unmatched_elements);
}

KeypointWithDescriptor* getMatchingFeatureInRegion(const int32_t& row_reference_,
                                                   const int32_t& col_reference_,
                                                   const cv::Mat& descriptor_reference_,
                                                   const int32_t& range_point_tracking_pixels_rows_,
                                                   const int32_t& range_point_tracking_pixels_cols_,
                                                   const int32_t& image_rows_,
                                                   const int32_t& image_cols_,
                                                   KeypointWithDescriptor*** candidates_,
                                                   const double& maximum_descriptor_distance_tracking_) {
  double descriptor_distance_best = maximum_descriptor_distance_tracking_;
  int32_t row_best = -1;
  int32_t col_best = -1;

  //ds current tracking region
  const int32_t row_start_point = std::max(row_reference_-range_point_tracking_pixels_rows_, 0);
  const int32_t row_end_point   = std::min(row_reference_+range_point_tracking_pixels_rows_+1, image_rows_);
  const int32_t col_start_point = std::max(col_reference_-range_point_tracking_pixels_cols_, 0);
  const int32_t col_end_point   = std::min(col_reference_+range_point_tracking_pixels_cols_+1, image_cols_);

  //ds locate best match in appearance
  for (int32_t row = row_start_point; row < row_end_point; ++row) {
    for (int32_t col = col_start_point; col < col_end_point; ++col) {
      if (candidates_[row][col]) {
        const double descriptor_distance = cv::norm(descriptor_reference_,
                                                    candidates_[row][col]->descriptor,
                                                    SRRG_PROSLAM_DESCRIPTOR_NORM);

        if (descriptor_distance < descriptor_distance_best) {
          descriptor_distance_best = descriptor_distance;
          row_best = row;
          col_best = col;
        }
      }
    }
  }

  //ds if we found a match
  if (row_best != -1) {
    return candidates_[row_best][col_best];
  } else {
    return nullptr;
  }
}

Eigen::Vector3d getPointInLeftCamera(const double& u_L, const double v_L,
                                     const double& u_R, const double v_R,
                                     const double& f_x, const double& f_y,
                                     const double& c_x, const double& c_y,
                                     const double& b_x_) {

  //ds point coordinates in camera frame
  Eigen::Vector3d position_in_left_camera(Eigen::Vector3d::Zero());

  //ds check for zero disparity
  if (u_L == u_R) {

    //ds set infinity depth (here maximum baseline)
    position_in_left_camera.z() = -b_x_;
  } else {

    //ds triangulate point normally
    position_in_left_camera.z() = b_x_/(u_R-u_L);
  }

  //ds compute x in camera (regular)
  position_in_left_camera.x() = 1/f_x*(u_L-c_x)*position_in_left_camera.z();

  //ds average in case we have an epipolar offset in v
  position_in_left_camera.y() = 1/f_y*((v_L+v_R)/2.0-c_y)*position_in_left_camera.z();
  return position_in_left_camera;
}
