#include "qapplication.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "ui/gt_tracker_viewer.h"
#include "ui/simple_point_viewer.h"
#include "types/gt_camera.h"

using namespace gslam;
using namespace srrg_core;

//#define LINE_SEARCH_RETARDED
#define LINE_SEARCH_SMART

//#define TRACKING_RETARDED
#define TRACKING_SMART

//ds nasty globals
#if CV_MAJOR_VERSION == 2
  std::shared_ptr<cv::FeatureDetector> feature_detector;
  std::shared_ptr<cv::DescriptorExtractor> descriptor_extractor;
#elif CV_MAJOR_VERSION == 3
  cv::Ptr<cv::Feature2D> feature_detector;
  cv::Ptr<cv::Feature2D> descriptor_extractor;
#else
  #error OpenCV version not supported
#endif

//ds triangulation constants
gt_real triangulation_F           = 0;
gt_real triangulation_Finverse    = 0;
gt_real triangulation_Pu          = 0;
gt_real triangulation_Pv          = 0;
gt_real triangulation_DuR         = 0;
gt_real triangulation_DuR_flipped = 0;

//ds triangulation
const gt_real maximum_matching_distance_triangulation = 50;
const gt_real minimum_disparity   = 2;

//ds tracking
const gt_real maximum_matching_distance_tracking    = 50;
const gt_real maximum_matching_distance_tracking_region   = 50;
const int32_t range_point_tracking  = 2;
const int32_t range_region_tracking = 40;
const int32_t length_epipolar_line  = 60;
const int32_t width_epipolar_line   = 5;

//ds read cameras
StringCameraMap cameras_by_topic;
Count number_of_image_rows = 0;
Count number_of_image_cols = 0;

//ds normalization - multiples of aspect ratio (e.g. 16:9 or 10:3 for kitti)
Count number_of_bins_u = 0;
Count number_of_bins_v = 0;
const Count bin_size   = 4;
cv::KeyPoint** bin_map_left;

//ds ui control
int32_t cv_wait_key_timeout_milliseconds = 0;

struct TriangulatedPoint {
  bool is_set       = false;
  bool is_available = false;
  cv::KeyPoint keypoint_left;
  cv::KeyPoint keypoint_right;
  cv::Mat descriptor_left;
  cv::Mat descriptor_right;
  PointCoordinates camera_coordinates_left;
};

struct Track {
    int32_t row;
    int32_t col;
    cv::Point2f image_coordinates;
    cv::Point2f image_coordinates_previous;
    cv::Mat descriptor;
    PointCoordinates camera_coordinates;
};

struct KeypointWD {
  cv::KeyPoint keypoint;
  uint32_t r;
  uint32_t c;
  cv::Mat descriptor;
};

//ds structures allocated in global heap
#ifdef LINE_SEARCH_RETARDED
int32_t** keypoint_index_map_left;
int32_t** keypoint_index_map_right;
#endif
TriangulatedPoint** triangulation_map;
std::vector<Track> points_tracked_previous;

const bool displayImages(const cv::Mat& image_left_, const cv::Mat& image_right_);
const PointCoordinates _getCoordinatesByTriangulation(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_);
const gt_real _getEpipolarCurveU(const Vector3& coefficients_, const gt_real& coordinate_v_) {return -(coefficients_(1)*coordinate_v_+coefficients_(2))/coefficients_(0);}
const gt_real _getEpipolarCurveV(const Vector3& coefficients_, const gt_real& coordinate_u_) {return -(coefficients_(0)*coordinate_u_+coefficients_(2))/coefficients_(1);}
void binKeypoints(std::vector<cv::KeyPoint>& keypoints_, cv::KeyPoint** bin_map_);

int32_t main(int32_t argc, char** argv) {

  //ds lock opencv to use only 1 thread
  cv::setNumThreads(1);

  //ds enable maximally deterministic mode
  cv::setUseOptimized(false);

  //ds if no datafile provided
  if (argc != 2) {
    std::cerr << "ERROR: No input provided, usage: ./srrg_gslam_feature_tester_svi_app txt_io_file.txt" << std::endl;
    return 0;
  }

  //ds input file
  const std::string filename_messages(argv[1]);

  //ds configure sensor message source
  if (filename_messages.length() == 0) {
    std::cerr << "ERROR: No input provided, usage: ./srrg_gslam_feature_tester_svi_app txt_io_file.txt" << std::endl;
    return 0;
  }

  //ds configure feature handling
#if CV_MAJOR_VERSION == 2
    feature_detector     = std::make_shared<cv::FastFeatureDetector>();
    descriptor_extractor = std::make_shared<cv::BriefDescriptorExtractor>(cv::BriefDescriptorExtractor(DESCRIPTOR_SIZE_BYTES));
#elif CV_MAJOR_VERSION == 3
    feature_detector     = cv::FastFeatureDetector::create();
    descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(DESCRIPTOR_SIZE_BYTES);
#else
  #error OpenCV version not supported
#endif

  //ds ui
  QApplication* ui_server         = new QApplication(argc, argv);
  SimplePointViewer* point_viewer = new SimplePointViewer();
  point_viewer->show();
  std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;

  //ds set up message reader
  MessageReader sensor_message_reader;
  sensor_message_reader.open(filename_messages);

  //ds configure message synchronizer
  MessageTimestampSynchronizer synchronizer;
  synchronizer.setTimeInterval(0.001);
  const std::string topic_image_stereo_left      = "/camera_left/image_raw";
  const std::string topic_image_stereo_right     = "/camera_right/image_raw";
  std::vector<std::string> topics_synchronized;
  topics_synchronized.push_back(topic_image_stereo_left);
  topics_synchronized.push_back(topic_image_stereo_right);
  synchronizer.setTopics(topics_synchronized);
  TransformMatrix3D odometry_robot_to_world_previous = TransformMatrix3D::Identity();

  //ds prepare cameras
  cameras_by_topic.clear();

  //ds quickly read the first messages to buffer camera info
  BaseMessage* base_message = 0;
  while ((base_message = sensor_message_reader.readMessage())) {
    BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(base_message);
    assert(sensor_msg != 0);
    sensor_msg->untaint();

    //ds check for the two set topics
    if (sensor_msg->topic() == topic_image_stereo_left) {
      PinholeImageMessage* message_image_left  = dynamic_cast<PinholeImageMessage*>(sensor_msg);

      //ds allocate a new camera
      Camera* camera_left = new Camera(message_image_left->image().rows,
                                       message_image_left->image().cols,
                                       message_image_left->cameraMatrix().cast<gt_real>(),
                                       message_image_left->offset().cast<gt_real>());
      cameras_by_topic.insert(std::make_pair(message_image_left->topic(), camera_left));

      //ds set triangulation properties
      triangulation_F        = camera_left->projectionMatrix()(0,0);
      triangulation_Finverse = 1/triangulation_F;
      triangulation_Pu       = camera_left->projectionMatrix()(0,2);
      triangulation_Pv       = camera_left->projectionMatrix()(1,2);
      number_of_image_rows = camera_left->imageRows();
      number_of_image_cols = camera_left->imageCols();
    } else if (sensor_msg->topic() == topic_image_stereo_right) {
      PinholeImageMessage* message_image_right  = dynamic_cast<PinholeImageMessage*>(sensor_msg);

      //ds allocate a new camera
      Camera* camera_right = new Camera(message_image_right->image().rows,
                                        message_image_right->image().cols,
                                        message_image_right->cameraMatrix().cast<gt_real>(),
                                        message_image_right->offset().cast<gt_real>());
      cameras_by_topic.insert(std::make_pair(message_image_right->topic(), camera_right));

      //ds set triangulation properties
      triangulation_DuR         = camera_right->projectionMatrix()(0,3);
      triangulation_DuR_flipped = -triangulation_DuR;
      number_of_image_rows = camera_right->imageRows();
      number_of_image_cols = camera_right->imageCols();
    }
    delete sensor_msg;

    //ds if we got all the information we need
    if (cameras_by_topic.size() == topics_synchronized.size()) {
      break;
    }
  }

  std::cerr << "loaded camera settings - rows: " << number_of_image_rows << " cols: " << number_of_image_cols << std::endl;

  //ds compute binning configuration
  number_of_bins_u = std::floor(static_cast<gt_real>(number_of_image_cols)/bin_size);
  number_of_bins_v = std::floor(static_cast<gt_real>(number_of_image_rows)/bin_size);

  std::cerr << "bin sixe (pixel): " << bin_size << std::endl;
  std::cerr << "number of bins u: " << number_of_bins_u << std::endl;
  std::cerr << "number of bins v: " << number_of_bins_v << std::endl;
  std::cerr << "total number of bins: " << number_of_bins_u*number_of_bins_v << std::endl;

  //ds allocate dynamic datastructures: simple maps
#ifdef LINE_SEARCH_RETARDED
  keypoint_index_map_left  = new int32_t*[number_of_image_rows];
  keypoint_index_map_right = new int32_t*[number_of_image_rows];
#endif
  triangulation_map        = new TriangulatedPoint*[number_of_image_rows];
  for (Index row = 0; row < number_of_image_rows; ++row) {
#ifdef LINE_SEARCH_RETARDED
    keypoint_index_map_left[row]  = new int32_t[number_of_image_cols];
    keypoint_index_map_right[row] = new int32_t[number_of_image_cols];
#endif
    triangulation_map[row]        = new TriangulatedPoint[number_of_image_cols];
  }

  //ds allocate dynamic datastructures: bin grid
  bin_map_left  = new cv::KeyPoint*[number_of_bins_v];
  for (Count v = 0; v < number_of_bins_v; ++v) {
    bin_map_left[v]  = new cv::KeyPoint[number_of_bins_u];
    for (Count u = 0; u < number_of_bins_u; ++u) {
      bin_map_left[v][u].response  = 0;
    }
  }

  //ds initialize data structures
  points_tracked_previous.clear();

  //ds statistics
  Count number_of_frames = 0;
  double duration_feature_detection  = 0;
  double duration_binning            = 0;
  double duration_feature_extraction = 0;
  double duration_triangulation      = 0;
  double duration_tracking           = 0;
  const double time_start_total = getTime();
  gt_real total_ratio_triangulation = 0;
  gt_real total_ratio_tracking_point    = 0;
  gt_real total_ratio_tracking_region   = 0;
  gt_real total_ratio_tracking_epipolar = 0;
  Count total_number_of_points_tracked = 0;

  //ds cameras
  Camera* camera_left = cameras_by_topic.at(topic_image_stereo_left);

  //ds start playback
  bool termination_requested = false;
  while ((base_message = sensor_message_reader.readMessage()) && !termination_requested) {
    BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(base_message);
    assert(sensor_msg != 0);
    sensor_msg->untaint();

    //ds add to synchronizer
    if (sensor_msg->topic() == topic_image_stereo_left) {
      synchronizer.putMessage(sensor_msg);
    } else if (sensor_msg->topic() == topic_image_stereo_right) {
      synchronizer.putMessage(sensor_msg);
    } else {
      delete sensor_msg;
    }

    //ds if we have a synchronized package of sensor messages ready
    if (synchronizer.messagesReady()) {

      //ds buffer sensor data
      PinholeImageMessage* message_image_left  = dynamic_cast<PinholeImageMessage*>(synchronizer.messages()[0].get());
      PinholeImageMessage* message_image_right = dynamic_cast<PinholeImageMessage*>(synchronizer.messages()[1].get());

      //ds buffer images and cameras
      cv::Mat intensity_image_left  = message_image_left->image();
      cv::Mat intensity_image_right = message_image_right->image();
      //cv::equalizeHist(message_image_left->image(), intensity_image_left);
      //cv::equalizeHist(message_image_right->image(), intensity_image_right);

      //ds get rgb copies of both images
      cv::Mat display_image_left_rgb;
      cv::Mat display_image_right_rgb;
      cv::cvtColor(intensity_image_left, display_image_left_rgb, CV_GRAY2RGB);
      cv::cvtColor(intensity_image_right, display_image_right_rgb, CV_GRAY2RGB);

      //ds compute odometry change (used for epipolar lines)
      const TransformMatrix3D robot_previous_to_current  = message_image_left->odometry().cast<gt_real>().inverse()*odometry_robot_to_world_previous;
      odometry_robot_to_world_previous                   = message_image_left->odometry().cast<gt_real>();
      const TransformMatrix3D camera_previous_to_current = camera_left->robotToCamera()*robot_previous_to_current*camera_left->cameraToRobot();

      //ds new features buffer
      std::vector<cv::KeyPoint> keypoints_left;
      std::vector<cv::KeyPoint> keypoints_right;

      //ds run detection
      const double time_start_feature_detection = getTime();
      feature_detector->detect(intensity_image_left, keypoints_left);
      feature_detector->detect(intensity_image_right, keypoints_right);
      duration_feature_detection += getTime()-time_start_feature_detection;

      //ds keypoint pruning - preemptively filter keypoints by bin
      const double time_start_binning = getTime();
      binKeypoints(keypoints_left, bin_map_left);
      duration_binning += getTime()-time_start_binning;

      //ds extract descriptors for detected features TODO make it surgical
      cv::Mat descriptors_left;
      cv::Mat descriptors_right;
      const double time_start_feature_extraction = getTime();
      descriptor_extractor->compute(intensity_image_left, keypoints_left, descriptors_left);
      descriptor_extractor->compute(intensity_image_right, keypoints_right, descriptors_right);
      duration_feature_extraction += getTime()-time_start_feature_extraction;

      //ds initialize data structures
      const double time_start_triangulation = getTime();
      for (Count row = 0; row < number_of_image_rows; ++row) {
        for (Count col = 0; col < number_of_image_cols; ++col) {
#ifdef LINE_SEARCH_RETARDED
          keypoint_index_map_left[row][col]  = -1;
          keypoint_index_map_right[row][col] = -1;
#endif
          triangulation_map[row][col].is_set       = false;
          triangulation_map[row][col].is_available = false;
        }
      }

      //ds triangulations: <index, index> (take either one of the sizes)
      std::vector<Track> points_triangulated(keypoints_left.size());
      Index index_point_triangulated = 0;

#ifdef LINE_SEARCH_RETARDED
      //ds set keypoint maps
      for (int32_t index_keypoint = 0; index_keypoint < static_cast<int32_t>(keypoints_left.size()); ++index_keypoint) {
        const cv::KeyPoint& keypoint_left = keypoints_left[index_keypoint];
        const int32_t row = static_cast<int32_t>(keypoint_left.pt.y);
        const int32_t col = static_cast<int32_t>(keypoint_left.pt.x);
        keypoint_index_map_left[row][col] = index_keypoint;
      }
      for (int32_t index_keypoint = 0; index_keypoint < static_cast<int32_t>(keypoints_right.size()); ++index_keypoint) {
        const cv::KeyPoint& keypoint_right = keypoints_right[index_keypoint];
        const int32_t row = static_cast<int32_t>(keypoint_right.pt.y);
        const int32_t col = static_cast<int32_t>(keypoint_right.pt.x);
        keypoint_index_map_right[row][col] = index_keypoint;
      }

      //ds look for triangulation pairs - row wise
      for (uint32_t row = 0; row < number_of_image_rows; ++row) {

        //ds last found on right in this column - blocking the search range to the left
        int32_t col_right_last = 0;

        //ds always starting from the left
        for (uint32_t col_left = 0; col_left < number_of_image_cols; ++col_left) {

          //ds if we have a keypoint at this location
          if (keypoint_index_map_left[row][col_left] != -1) {

            //ds exhaustive search
            gt_real matching_distance_best = maximum_matching_distance_triangulation;
            int32_t col_right_best         = -1;

            //ds check the all potentially matching stereo keypoints in range
            for (int32_t col_right = col_left; col_right > col_right_last; --col_right) {

              //ds if we have a keypoint at this location
              if (keypoint_index_map_right[row][col_right] != -1) {

                //ds compute the distance
                const gt_real matching_distance = cv::norm(descriptors_left.row(keypoint_index_map_left[row][col_left]),
                                                           descriptors_right.row(keypoint_index_map_right[row][col_right]),
                                                           DESCRIPTOR_NORM);

                if (matching_distance < matching_distance_best) {
                  matching_distance_best = matching_distance;
                  col_right_best = col_right;
                }
              }
            }

            //ds if we found a match
            if (matching_distance_best < maximum_matching_distance_triangulation) {
              try {

                //ds directly attempt the triangulation
                const PointCoordinates camera_coordinates(_getCoordinatesByTriangulation(keypoints_left[keypoint_index_map_left[row][col_left]].pt,
                                                                                         keypoints_right[keypoint_index_map_right[row][col_right_best]].pt));

                //ds set descriptor map
                triangulation_map[row][col_left].camera_coordinates_left = camera_coordinates;
                triangulation_map[row][col_left].keypoint_left    = keypoints_left[keypoint_index_map_left[row][col_left]];
                triangulation_map[row][col_left].keypoint_right   = keypoints_right[keypoint_index_map_right[row][col_right_best]];
                triangulation_map[row][col_left].descriptor_left  = descriptors_left.row(keypoint_index_map_left[row][col_left]);
                triangulation_map[row][col_left].descriptor_right = descriptors_right.row(keypoint_index_map_right[row][col_right_best]);
                triangulation_map[row][col_left].is_set           = true;
                triangulation_map[row][col_left].is_available     = true;

                points_triangulated[index_point_triangulated].camera_coordinates         = triangulation_map[row][col_left].camera_coordinates_left;
                points_triangulated[index_point_triangulated].row                        = triangulation_map[row][col_left].keypoint_left.pt.y;
                points_triangulated[index_point_triangulated].col                        = triangulation_map[row][col_left].keypoint_left.pt.x;
                points_triangulated[index_point_triangulated].image_coordinates          = triangulation_map[row][col_left].keypoint_left.pt;
                points_triangulated[index_point_triangulated].image_coordinates_previous = cv::Point2f(0, 0);
                points_triangulated[index_point_triangulated].descriptor                 = triangulation_map[row][col_left].descriptor_left;
                ++index_point_triangulated;

                //ds disable further matching and reduce search time
                col_right_last = col_right_best;
              } catch (const std::runtime_error&) {

              }
            }
          }
        }
      }
#elif defined LINE_SEARCH_SMART

      //ds fused keypoint + descriptor vectors
      std::vector<KeypointWD> K_L(keypoints_left.size());
      std::vector<KeypointWD> K_R(keypoints_right.size());
      for (Index u = 0; u < keypoints_left.size(); ++u) {
        K_L[u].keypoint   = keypoints_left[u];
        K_L[u].descriptor = descriptors_left.row(u);
        K_L[u].r          = std::floor(keypoints_left[u].pt.y);
        K_L[u].c          = std::floor(keypoints_left[u].pt.x);
      }
      for (Index u = 0; u < keypoints_right.size(); ++u) {
        K_R[u].keypoint   = keypoints_right[u];
        K_R[u].descriptor = descriptors_right.row(u);
        K_R[u].r          = std::floor(keypoints_right[u].pt.y);
        K_R[u].c          = std::floor(keypoints_right[u].pt.x);
      }

      //sort all input vectors in the order of the expression
      std::sort(K_L.begin(), K_L.end(), [](const KeypointWD& a_, const KeypointWD& b_){return ((a_.r < b_.r) ||
                                                                                               (a_.r == b_.r && a_.c < b_.c));});
      std::sort(K_R.begin(), K_R.end(), [](const KeypointWD& a_, const KeypointWD& b_){return ((a_.r < b_.r) ||
                                                                                               (a_.r == b_.r && a_.c < b_.c));});

      //configuration
      uint32_t idx_R = 0;
      //loop over all left Keypoints
      for (uint32_t idx_L = 0; idx_L < K_L.size(); idx_L++) {
        //the right Keypoints are a row ahead - skip left
        while (K_L[idx_L].r < K_R[idx_R].r) {
          idx_L++; if (idx_L == K_L.size()) {break;}
        }
        //the right Keypoints are a row behind - skip right
        while (K_L[idx_L].r > K_R[idx_R].r) {
          idx_R++; if (idx_R == K_R.size()) {break;}
        }
        //search bookkeeping
        uint32_t idx_RS     = idx_R;
        float dist_best     = maximum_matching_distance_triangulation;
        uint32_t idx_best_R = 0;
        //scan epipolar line for current Keypoint at idx_L
        while (idx_RS < K_R.size() && K_L[idx_L].r == K_R[idx_RS].r) {
          //ds zero disparity stop condition
          if (K_R[idx_RS].c >= K_L[idx_L].c) {break;}
          const float dist = cv::norm(K_L[idx_L].descriptor, K_R[idx_RS].descriptor, DESCRIPTOR_NORM);
          if(dist < dist_best) {
            dist_best  = dist;
            idx_best_R = idx_RS;
          }
          idx_RS++;
        }
        //check if something was found
        if (dist_best < maximum_matching_distance_triangulation) {
          try {

            //ds directly attempt the triangulation
            const PointCoordinates camera_coordinates(_getCoordinatesByTriangulation(K_L[idx_L].keypoint.pt,
                                                                                     K_R[idx_best_R].keypoint.pt));

            //ds wrapping
            const int row      = K_L[idx_L].r;
            const int col_left = K_L[idx_L].c;

            //ds set descriptor map
            triangulation_map[row][col_left].camera_coordinates_left = camera_coordinates;
            triangulation_map[row][col_left].keypoint_left    = K_L[idx_L].keypoint;
            triangulation_map[row][col_left].keypoint_right   = K_R[idx_best_R].keypoint;
            triangulation_map[row][col_left].descriptor_left  = K_L[idx_L].descriptor;
            triangulation_map[row][col_left].descriptor_right = K_R[idx_best_R].descriptor;
            triangulation_map[row][col_left].is_set           = true;
            triangulation_map[row][col_left].is_available     = true;

            points_triangulated[index_point_triangulated].camera_coordinates         = triangulation_map[row][col_left].camera_coordinates_left;
            points_triangulated[index_point_triangulated].row                        = triangulation_map[row][col_left].keypoint_left.pt.y;
            points_triangulated[index_point_triangulated].col                        = triangulation_map[row][col_left].keypoint_left.pt.x;
            points_triangulated[index_point_triangulated].image_coordinates          = triangulation_map[row][col_left].keypoint_left.pt;
            points_triangulated[index_point_triangulated].image_coordinates_previous = cv::Point2f(0, 0);
            points_triangulated[index_point_triangulated].descriptor                 = triangulation_map[row][col_left].descriptor_left;
            ++index_point_triangulated;

            idx_R = idx_best_R+1; if (idx_R == K_R.size()) {break;}

          } catch (const std::runtime_error&) {}
        }
      }

#endif

      //ds resize result vector
      points_triangulated.resize(index_point_triangulated);
      duration_triangulation += getTime()-time_start_triangulation;






      //ds TRACKING
      std::vector<Track> points_tracked(index_point_triangulated);
      std::vector<PointDrawable> drawables(index_point_triangulated);

      //ds track previous points - automatically updating previous grid
      const double time_start_tracking = getTime();
      Index index_point_tracked = 0;
      Count number_of_points_tracked_point    = 0;
      Count number_of_points_tracked_region   = 0;

#ifdef TRACKING_RETARDED
      for (const Track& track_previous: points_tracked_previous) {

        //ds compute projection (used by epipolar line only as we want to keep maximum tracking difficulty)
        Vector4 point_in_camera_homogeneous(Vector4::Ones());
        point_in_camera_homogeneous.head<3>() = camera_previous_to_current*track_previous.camera_coordinates;
        PointCoordinates point_in_image_left = camera_left->projectionMatrix()*point_in_camera_homogeneous;
        point_in_image_left  /= point_in_image_left.z();
        const cv::Point2f projection_left(std::round(point_in_image_left.x()), std::round(point_in_image_left.y()));

        //ds exhaustive search
        gt_real matching_distance_best = maximum_matching_distance_tracking;
        int32_t row_best = -1;
        int32_t col_best = -1;

//ds ------------------------------------------- STAGE 1: POINT VICINITY TRACKING
        //ds compute borders
        const int32_t row_start_point = std::max(track_previous.row-range_point_tracking, static_cast<int32_t>(0));
        const int32_t row_end_point   = std::min(track_previous.row+range_point_tracking, static_cast<int32_t>(number_of_image_rows));
        const int32_t col_start_point = std::max(track_previous.col-range_point_tracking, static_cast<int32_t>(0));
        const int32_t col_end_point   = std::min(track_previous.col+range_point_tracking, static_cast<int32_t>(number_of_image_cols));

        //ds locate best match
        for (int32_t row_point = row_start_point; row_point < row_end_point; ++row_point) {
          for (int32_t col_point = col_start_point; col_point < col_end_point; ++col_point) {
            if (triangulation_map[row_point][col_point].is_available) {
              const gt_real matching_distance = cv::norm(track_previous.descriptor,
                                                         triangulation_map[row_point][col_point].descriptor_left,
                                                         DESCRIPTOR_NORM);

              if (matching_distance < matching_distance_best) {
                matching_distance_best = matching_distance;
                row_best = row_point;
                col_best = col_point;
              }
            }
          }
        }

        //ds if we found a match
        if (matching_distance_best < maximum_matching_distance_tracking) {
          points_tracked[index_point_tracked].row  = row_best;
          points_tracked[index_point_tracked].col  = col_best;
          points_tracked[index_point_tracked].image_coordinates = cv::Point2f(col_best, row_best);
          points_tracked[index_point_tracked].image_coordinates_previous = cv::Point2f(track_previous.col, track_previous.row);
          points_tracked[index_point_tracked].descriptor = triangulation_map[row_best][col_best].descriptor_left;
          drawables[index_point_tracked] = std::make_pair(track_previous.camera_coordinates, PointColorRGB(0, 1, 0));
          ++index_point_tracked;
          ++number_of_points_tracked_point;

          //ds disable further matching and reduce search time
          triangulation_map[row_best][col_best].is_available = false;
          continue;
        }

//ds ------------------------------------------- STAGE 2: REGIONAL TRACKING
        matching_distance_best = maximum_matching_distance_tracking;

        //ds compute borders
        const int32_t row_start_region = std::max(track_previous.row-range_region_tracking, static_cast<int32_t>(0));
        const int32_t row_end_region   = std::min(track_previous.row+range_region_tracking, static_cast<int32_t>(number_of_image_rows));
        const int32_t col_start_region = std::max(track_previous.col-range_region_tracking, static_cast<int32_t>(0));
        const int32_t col_end_region   = std::min(track_previous.col+range_region_tracking, static_cast<int32_t>(number_of_image_cols));

        //ds locate best match
        for (int32_t row_region = row_start_region; row_region < row_end_region; ++row_region) {
          for (int32_t col_region = col_start_region; col_region < col_end_region; ++col_region) {
            if (triangulation_map[row_region][col_region].is_available) {

              //ds if area has not been already evaluated in previous stage
              if (row_region < row_start_point||
                  row_region >= row_end_point ||
                  col_region < col_start_point||
                  col_region >= col_end_point ) {

                const gt_real matching_distance = cv::norm(track_previous.descriptor,
                                                           triangulation_map[row_region][col_region].descriptor_left,
                                                           DESCRIPTOR_NORM);

                if (matching_distance < matching_distance_best) {
                  matching_distance_best = matching_distance;
                  row_best = row_region;
                  col_best = col_region;
                }
              }
            }
          }
        }

        //ds if we found a match
        if (matching_distance_best < maximum_matching_distance_tracking) {
          points_tracked[index_point_tracked].row  = row_best;
          points_tracked[index_point_tracked].col  = col_best;
          points_tracked[index_point_tracked].image_coordinates = cv::Point2f(col_best, row_best);
          points_tracked[index_point_tracked].image_coordinates_previous = cv::Point2f(track_previous.col, track_previous.row);
          points_tracked[index_point_tracked].descriptor = triangulation_map[row_best][col_best].descriptor_left;
          drawables[index_point_tracked] = std::make_pair(track_previous.camera_coordinates, PointColorRGB(0, 0, 1));
          ++index_point_tracked;
          ++number_of_points_tracked_region;

          //ds disable further matching and reduce search time
          triangulation_map[row_best][col_best].is_available = false;
          continue;
        }
      }

#elif defined TRACKING_SMART

      //ds create KD-tree from left keypoints K_L
      //Tree t

      //ds query tree for previous points
      for (const Track& track_previous: points_tracked_previous) {

        //ds compute projection (used by epipolar line only as we want to keep maximum tracking difficulty)
        Vector4 point_in_camera_homogeneous(Vector4::Ones());
        point_in_camera_homogeneous.head<3>() = camera_previous_to_current*track_previous.camera_coordinates;
        PointCoordinates point_in_image_left = camera_left->projectionMatrix()*point_in_camera_homogeneous;
        point_in_image_left  /= point_in_image_left.z();
        const cv::Point2f projection_left(std::round(point_in_image_left.x()), std::round(point_in_image_left.y()));

        //ds get selection of closest points
        //vector<> closest_points = t.getkNeighbors(.....)

        //ds find closest with sufficient matching distance < maximum_matching_distance_tracking
        //loop over closest_points

      }

#endif

      duration_tracking += getTime()-time_start_tracking;
      points_tracked.resize(index_point_tracked);
      drawables.resize(index_point_tracked);

      //ds display tracks
      for (const Track& track: points_tracked) {

        //ds draw track
        cv::line(display_image_left_rgb, track.image_coordinates_previous, track.image_coordinates, CV_COLOR_CODE_GREEN);
        cv::circle(display_image_left_rgb, track.image_coordinates_previous, 2, CV_COLOR_CODE_RED, -1);
        cv::circle(display_image_left_rgb, track.image_coordinates, 2, CV_COLOR_CODE_BLUE, -1);
      }














      std::cerr << "Keypoints L: " << keypoints_left.size()
                << " R: " << keypoints_right.size()
                << " triangulated points: " << index_point_triangulated
                << " ratio: " << static_cast<gt_real>(2*index_point_triangulated)/(keypoints_left.size()+keypoints_right.size())
                << " points tracked: " << index_point_tracked;

      //ds update stats
      total_ratio_triangulation     += static_cast<gt_real>(2*index_point_triangulated)/(keypoints_left.size()+keypoints_right.size());
      if (points_tracked_previous.size() > 0) {
        total_ratio_tracking_point    += static_cast<gt_real>(number_of_points_tracked_point)/points_tracked_previous.size();
        total_ratio_tracking_region   += static_cast<gt_real>(number_of_points_tracked_region)/points_tracked_previous.size();
        std::cerr << " ratio: " << static_cast<gt_real>(index_point_tracked)/points_tracked_previous.size();
      }
      std::cerr << std::endl;

      total_number_of_points_tracked += index_point_tracked;

      //ds update previous
      points_tracked_previous.resize(index_point_triangulated);
      for (uint32_t u = 0; u < index_point_triangulated; ++u) {
        points_tracked_previous[u] = points_triangulated[u];
      }
















      //ds update ui
      termination_requested = displayImages(display_image_left_rgb, display_image_right_rgb);
      point_viewer->setPoints(drawables);
      point_viewer->updateGL();
      ui_server->processEvents();
      synchronizer.reset();
      ++number_of_frames;
    }
  }
  const double duration_total = getTime()-time_start_total;

  std::cerr << "dataset completed" << std::endl;
  std::cerr << "               total frames: " << number_of_frames << std::endl;
  std::cerr << "                average fps: " << number_of_frames/duration_total << std::endl;
  std::cerr << "                average pps: " << static_cast<gt_real>(total_number_of_points_tracked)/duration_total << std::endl;
  std::cerr << "     average tracked points: " << static_cast<gt_real>(total_number_of_points_tracked)/number_of_frames << std::endl;
  std::cerr << "average ratio triangulation: " << total_ratio_triangulation/number_of_frames << std::endl;
  std::cerr << "    average ratio tracking    point: " << total_ratio_tracking_point/number_of_frames << std::endl;
  std::cerr << "    average ratio tracking   region: " << total_ratio_tracking_region/number_of_frames << std::endl;
  std::cerr << "    average ratio tracking epipolar: " << total_ratio_tracking_epipolar/number_of_frames << std::endl;
  std::cerr << "duration    keypoint detection: " << duration_feature_detection/duration_total << " (" << duration_feature_detection << "s)" << std::endl;
  std::cerr << "duration        regularization: " << duration_binning/duration_total << " (" << duration_binning << "s)" << std::endl;
  std::cerr << "duration descriptor extraction: " << duration_feature_extraction/duration_total << " (" << duration_feature_extraction << "s)" << std::endl;
  std::cerr << "duration         triangulation: " << duration_triangulation/duration_total << " (" << duration_triangulation << "s)" << std::endl;
  std::cerr << "duration              tracking: " << duration_tracking/duration_total << " (" << duration_tracking << "s)" << std::endl;

  return 0;
}

const bool displayImages(const cv::Mat& image_left_, const cv::Mat& image_right_) {

  //ds create stereo image
  cv::Mat image_stereo_rgb;
  cv::hconcat(image_left_, image_right_, image_stereo_rgb);

  //ds update ui
  cv::imshow("OpenCVViewer", image_stereo_rgb);
  switch(cv::waitKey(cv_wait_key_timeout_milliseconds)) {
    case TrackerViewer::KeyStroke::Escape: {
      std::cerr << "termination requested" << std::endl;
      return true;
    }
    case TrackerViewer::KeyStroke::Backspace: {
      if(cv_wait_key_timeout_milliseconds > 0) {
        cv_wait_key_timeout_milliseconds = 0;
        std::cerr << "switched to stepwise mode (press backspace for switch, press space for stepping)" << std::endl;
        return false;
      }
      else {
        cv_wait_key_timeout_milliseconds = 1;
        std::cerr << "switched to benchmark mode (press backspace for switch)" << std::endl;
        return false;
      }
      break;
    }
    default:{
      return false;
    }
  }
}

const PointCoordinates _getCoordinatesByTriangulation(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) {

  //ds check for minimal disparity
  if (image_coordinates_left_.x-image_coordinates_right_.x < minimum_disparity) {
    throw std::runtime_error("disparity value to low");
  }

  //ds input validation
  assert(image_coordinates_right_.x < image_coordinates_left_.x);
  assert(image_coordinates_right_.y == image_coordinates_left_.y);

  //ds first compute depth (z in camera)
  const gt_real depth_meters = triangulation_DuR_flipped/(image_coordinates_left_.x-image_coordinates_right_.x);
  assert(depth_meters >= 0);

  //ds set 3d point
  const PointCoordinates coordinates_in_camera(triangulation_Finverse*depth_meters*(image_coordinates_left_.x-triangulation_Pu),
                                               triangulation_Finverse*depth_meters*(image_coordinates_left_.y-triangulation_Pv),
                                               depth_meters);

  //ds return triangulated point
  return coordinates_in_camera;
}

void binKeypoints(std::vector<cv::KeyPoint>& keypoints_, cv::KeyPoint** bin_map_) {

  //ds sort by position in u
  std::sort(keypoints_.begin(), keypoints_.end(), [](const cv::KeyPoint& a_, const cv::KeyPoint& b_) {return a_.pt.x < b_.pt.x;});

  //ds check all keypoints for this grid
  Count u_current = 0;
  for (const cv::KeyPoint& keypoint: keypoints_) {
    const gt_real& keypoint_u = keypoint.pt.x;
    const gt_real& keypoint_v = keypoint.pt.y;

    //ds if the keypoint still enters the current bin
    if (keypoint_u < u_current*bin_size) {
      assert(u_current < number_of_bins_u);

      //ds check matching bin range in V
      for (Count v = 0; v < number_of_bins_v; ++v) {
        if (keypoint_v < v*bin_size) {

          //ds found matching bin - check if the reponse is higher
          if (keypoint.response > bin_map_[v][u_current].response) {
            bin_map_[v][u_current] = keypoint;
          }
          break;
        }
      }
    } else {
      ++u_current;

      //ds if we reached the end of the grid
      if (u_current == number_of_bins_u) {
        break;
      }
    }
  }

  //ds collect keypoints from all grids into one single vector - and at the same time prepare the data structure for the next binning
  Index index_keypoint  = 0;
  for (Count v = 0; v < number_of_bins_v; ++v) {
    for (Count u = 0; u < number_of_bins_u; ++u) {
      if (bin_map_[v][u].response > 0) {
        keypoints_[index_keypoint] = bin_map_[v][u];
        ++index_keypoint;
        bin_map_[v][u].response = 0;
      }
    }
  }
  keypoints_.resize(index_keypoint);
}
