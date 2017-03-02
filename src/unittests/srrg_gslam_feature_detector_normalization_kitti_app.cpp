#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "ui/gt_tracker_viewer.h"
#include "types/gt_camera.h"

using namespace gslam;
using namespace srrg_core;

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
const gt_real maximum_matching_distance_triangulation = 50;
const gt_real minimum_disparity   = 2;

//ds read cameras
StringCameraMap cameras_by_topic;

//ds normalization - multiples of aspect ratio (e.g. 16:9 or 10:3 for kitti)
const Count number_of_grids  = 1; //1;
const Count number_of_bins_u = 100; //100; //200;
const Count number_of_bins_v = 30; //30; //60;
gt_real bin_size_u = 0;
gt_real bin_size_v = 0;
Count number_of_image_rows = 0;
Count number_of_image_cols = 0;

//ds allocate this baby in the global heap
static cv::KeyPoint bin_map_left[number_of_grids][number_of_bins_v][number_of_bins_u];

//ds ui control
int32_t cv_wait_key_timeout_milliseconds = 0;

const bool displayImages(const cv::Mat& image_left_, const cv::Mat& image_right_);
const PointCoordinates _getCoordinatesByTriangulation(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_);
void regularize(const std::vector<cv::KeyPoint>& keypoints_);

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
  feature_detector = std::make_shared<cv::FastFeatureDetector>(50);
  descriptor_extractor = std::make_shared<cv::BriefDescriptorExtractor>(cv::BriefDescriptorExtractor(DESCRIPTOR_SIZE_BYTES));
#elif CV_MAJOR_VERSION == 3
  feature_detector = cv::FastFeatureDetector::create(10);
  descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(DESCRIPTOR_SIZE_BYTES);
#else
  #error OpenCV version not supported
#endif

  //ds ui
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

  //ds initialize data structures
  bin_size_u = static_cast<gt_real>(number_of_image_cols)/number_of_bins_u;
  bin_size_v = static_cast<gt_real>(number_of_image_rows)/number_of_bins_v;
  for (Count index_grid = 0; index_grid < number_of_grids; ++index_grid) {
    for (Count v = 0; v < number_of_bins_v; ++v) {
      for (Count u = 0; u < number_of_bins_u; ++u) {
        bin_map_left[index_grid][v][u].response  = 0;
      }
    }
  }
  std::cerr << "aspect ratio: " << static_cast<gt_real>(number_of_image_cols)/number_of_image_rows << std::endl;
  std::cerr << "number of bins in U: " << number_of_bins_u << " with size: " << bin_size_u << std::endl;
  std::cerr << "number of bins in V: " << number_of_bins_v << " with size: " << bin_size_v << std::endl;
  std::cerr << "total number of bins: " << number_of_bins_u*number_of_bins_v << std::endl;

  //ds keypoint maps
  int32_t keypoint_map_left[number_of_image_rows][number_of_image_cols];
  int32_t keypoint_map_right[number_of_image_rows][number_of_image_cols];

  //ds statistics
  Count number_of_frames = 0;
  double duration_feature_detection  = 0;
  double duration_feature_extraction = 0;
  double duration_triangulation      = 0;
  const double time_start_total = getTime();
  Count total_number_of_triangulations = 0;

  //ds start full playback
  base_message = 0;
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





      //ds reset maps
      for (uint32_t row = 0; row < number_of_image_rows; ++row) {
        for (uint32_t col = 0; col < number_of_image_cols; ++col) {
          keypoint_map_left[row][col]  = -1;
          keypoint_map_right[row][col] = -1;
        }
      }

      //ds new features buffer
      std::vector<cv::KeyPoint> keypoints_left;
      std::vector<cv::KeyPoint> keypoints_right;

      //ds run detection
      const double time_start_feature_detection = getTime();
      feature_detector->detect(intensity_image_left, keypoints_left);
      feature_detector->detect(intensity_image_right, keypoints_right);
      duration_feature_detection += getTime()-time_start_feature_detection;

      std::cerr << "detected keypoints L: " << keypoints_left.size() << " R: " << keypoints_right.size() << std::endl;

//      //ds draw boxes
//      for (Count v = 0; v < number_of_bins_v; ++v) {
//        for (Count u = 0; u < number_of_bins_u; ++u) {
//          cv::rectangle(display_image_left_rgb, cv::Point2f(u*bin_size_u, v*bin_size_v), cv::Point2f((u+1)*bin_size_u, (v+1)*bin_size_v), CV_COLOR_CODE_ORANGE);
//          cv::rectangle(display_image_right_rgb, cv::Point2f(u*bin_size_u, v*bin_size_v), cv::Point2f((u+1)*bin_size_u, (v+1)*bin_size_v), CV_COLOR_CODE_ORANGE);
//        }
//      }

      //ds clear detection boxes
      for (Count index_grid = 0; index_grid < number_of_grids; ++index_grid) {
        for (Count v = 0; v < number_of_bins_v; ++v) {
          for (Count u = 0; u < number_of_bins_u; ++u) {
            bin_map_left[index_grid][v][u].response  = 0;
          }
        }
      }







      //ds keypoint pruning - sort by position in u
      std::sort(keypoints_left.begin(), keypoints_left.end(), [](const cv::KeyPoint& a_, const cv::KeyPoint& b_) {return a_.pt.x < b_.pt.x;});

      //ds keypoint pruning - preemptively filter keypoints by bin
      regularize(keypoints_left);

      //ds get keypoint from bin maps
      Index index_keypoint_left  = 0;
      keypoints_left.resize(number_of_grids*keypoints_left.size());
      for (Count index_grid = 0; index_grid < number_of_grids; ++index_grid) {
        for (Count v = 0; v < number_of_bins_v; ++v) {
          for (Count u = 0; u < number_of_bins_u; ++u) {
            if (bin_map_left[index_grid][v][u].response > 0) {
              keypoints_left[index_keypoint_left] = bin_map_left[index_grid][v][u];
              ++index_keypoint_left;
            }
          }
        }
      }
      keypoints_left.resize(index_keypoint_left);
      std::sort(keypoints_left.begin(), keypoints_left.end(), [](const cv::KeyPoint& a_, const cv::KeyPoint& b_) {return ((a_.pt.x < b_.pt.x) || ((a_.pt.x == b_.pt.x) && (a_.pt.y < b_.pt.y)));});
      std::unique(keypoints_left.begin(), keypoints_left.end(), [](const cv::KeyPoint& a_, const cv::KeyPoint& b_) {return ((a_.pt.x == b_.pt.x) && (a_.pt.y == b_.pt.y));});




//
//      //ds sort both vectors by v descending
//      std::sort(keypoints_right.begin(), keypoints_right.end(), [](const cv::KeyPoint& a_, const cv::KeyPoint& b_) {return a_.pt.y < b_.pt.y;});
//      std::sort(keypoints_left.begin(), keypoints_left.end(), [](const cv::KeyPoint& a_, const cv::KeyPoint& b_) {return a_.pt.y < b_.pt.y;});
//
//      Index index_keypoint_left_filter  = 0;
//      Index index_keypoint_right_filter = 0;
//      Index index_keypoint_right = 0;
//      std::set<gt_real> empty_rows;
//      empty_rows.clear();
//
//
//      std::cerr << "filtering" << std::endl;
//      float v_last = keypoints_left[0].pt.y;
//      while (index_keypoint_right_filter < keypoints_right.size() && index_keypoint_left_filter < keypoints_left.size()) {
//
//        //ds check if corresponding y is set
//        while (keypoints_right[index_keypoint_right_filter].pt.y == v_last) {
//          keypoints_right[index_keypoint_right] = keypoints_right[index_keypoint_right_filter];
//          ++index_keypoint_right;
//          ++index_keypoint_right_filter;
//        }
//
//        //ds v is not matching anymore - move on until we get a new v
//        ++index_keypoint_left_filter;
//        while (v_last == keypoints_left[index_keypoint_left_filter].pt.y) {
//          ++index_keypoint_left_filter;
//        }
//        v_last = keypoints_left[index_keypoint_left_filter].pt.y;
//
//        //ds while there is a gap - skip all points in between
//        while (keypoints_right[index_keypoint_right_filter].pt.y < v_last) {
//          empty_rows.insert(keypoints_left[index_keypoint_right_filter].pt.y);
//          ++index_keypoint_right_filter;
//        }
//      }
//      keypoints_right.resize(index_keypoint_right);
//
//      std::cerr << "pruned keypoints L: " << keypoints_left.size() << " R: " << keypoints_right.size() << " empty rows: " << empty_rows.size() << std::endl;





      //ds extract descriptors for detected features TODO make it surgical
      cv::Mat descriptors_left;
      cv::Mat descriptors_right;
      const double time_start_feature_extraction = getTime();
      descriptor_extractor->compute(intensity_image_left, keypoints_left, descriptors_left);
      descriptor_extractor->compute(intensity_image_right, keypoints_right, descriptors_right);
      duration_feature_extraction += getTime()-time_start_feature_extraction;

      //ds set keypoint maps
      for (int32_t index_keypoint = 0; index_keypoint < static_cast<int32_t>(keypoints_left.size()); ++index_keypoint) {
        const cv::KeyPoint& keypoint_left = keypoints_left[index_keypoint];
//        cv::circle(display_image_left_rgb, keypoint_left.pt, 2, CV_COLOR_CODE_GREEN, -1);
        keypoint_map_left[static_cast<int32_t>(keypoint_left.pt.y)][static_cast<int32_t>(keypoint_left.pt.x)] = index_keypoint;
      }
      for (int32_t index_keypoint = 0; index_keypoint < static_cast<int32_t>(keypoints_right.size()); ++index_keypoint) {
        const cv::KeyPoint& keypoint_right = keypoints_right[index_keypoint];
//        cv::circle(display_image_right_rgb, keypoint_right.pt, 2, CV_COLOR_CODE_GREEN, -1);
        keypoint_map_right[static_cast<int32_t>(keypoint_right.pt.y)][static_cast<int32_t>(keypoint_right.pt.x)] = index_keypoint;
      }

      //ds triangulations: <index, index> (take either one of the sizes)
      std::vector<std::pair<int32_t, int32_t>> stereo_points(keypoints_left.size());
      Index index_stereo_point = 0;

      //ds look for triangulation pairs - row wise
      const double time_start_triangulation = getTime();
      for (uint32_t row = 0; row < number_of_image_rows; ++row) {

        //ds last found on right in this column - blocking the search range to the left
        int32_t col_right_last = 0;

        //ds always starting from the left
        for (uint32_t col_left = 0; col_left < number_of_image_cols; ++col_left) {

          //ds if we have a keypoint at this location
          if (keypoint_map_left[row][col_left] != -1) {

            //ds exhaustive search
            gt_real matching_distance_best = maximum_matching_distance_triangulation;
            int32_t col_right_best         = 0;

            //ds check the all potentially matching stereo keypoints in range
            for (int32_t col_right = col_left; col_right > col_right_last; --col_right) {

              //ds if we have a keypoint at this location
              if (keypoint_map_right[row][col_right] != -1) {

                //ds compute the distance
                const gt_real matching_distance = cv::norm(descriptors_left.row(keypoint_map_left[row][col_left]),
                                                           descriptors_right.row(keypoint_map_right[row][col_right]),
                                                           DESCRIPTOR_NORM);

                if (matching_distance < matching_distance_best) {
                  matching_distance_best = matching_distance;
                  col_right_best = col_right;
                }
              }
            }

            //ds if we found a match
            if (matching_distance_best < maximum_matching_distance_triangulation) {
              stereo_points[index_stereo_point] = std::make_pair(keypoint_map_left[row][col_left], keypoint_map_right[row][col_right_best]);
              ++index_stereo_point;

              //ds disable further matching and reduce search time
              col_right_last = col_right_best;
            }
          }
        }
      }
      stereo_points.resize(index_stereo_point);
      std::vector<PointCoordinates> points_triangulated(index_stereo_point);

      //ds triangulate all points
      Index index_point_triangulated = 0;
      for (Index u = 0; u < index_stereo_point; ++u) {

        try {

          //ds triangulate the point
          points_triangulated[index_point_triangulated] = _getCoordinatesByTriangulation(keypoints_left[stereo_points[u].first].pt, keypoints_right[stereo_points[u].second].pt);

          //ds draw points
          if (points_triangulated[index_point_triangulated].z() < 25) {
            cv::circle(display_image_left_rgb, keypoints_left[stereo_points[u].first].pt, 2, CV_COLOR_CODE_BLUE, -1);
            cv::circle(display_image_right_rgb, keypoints_right[stereo_points[u].second].pt, 2, CV_COLOR_CODE_BLUE, -1);
          } else {
            cv::circle(display_image_left_rgb, keypoints_left[stereo_points[u].first].pt, 2, CV_COLOR_CODE_VIOLETT, -1);
            cv::circle(display_image_right_rgb, keypoints_right[stereo_points[u].second].pt, 2, CV_COLOR_CODE_VIOLETT, -1);
          }
          ++index_point_triangulated;
        } catch(const std::runtime_error&) {
          //ds point not triangulated
        }

      }
      points_triangulated.resize(index_point_triangulated);
      duration_triangulation += getTime()-time_start_triangulation;

      std::cerr << "Keypoints L: " << keypoints_left.size()
                << " R: " << keypoints_right.size()
                << " stereo points: " << index_stereo_point
                << " triangulated points: " << index_point_triangulated
                << " ratio: " << static_cast<gt_real>(index_point_triangulated)/index_stereo_point
                << std::endl;

      total_number_of_triangulations += index_point_triangulated;
















      //ds update ui
      termination_requested = displayImages(display_image_left_rgb, display_image_right_rgb);
      synchronizer.reset();
      ++number_of_frames;
    }
  }
  const double duration_total = getTime()-time_start_total;

  std::cerr << "------------------- dataset completed -------------------" << std::endl;
  std::cerr << "          total frames: " << number_of_frames << std::endl;
  std::cerr << "           average fps: " << number_of_frames/duration_total << std::endl;
  std::cerr << "average triangulations: " << static_cast<gt_real>(total_number_of_triangulations)/number_of_frames << std::endl;
  std::cerr << "duration  feature detection (s): " << duration_feature_detection << " (" << duration_feature_detection/duration_total << ")" << std::endl;
  std::cerr << "duration feature extraction (s): " << duration_feature_extraction << " (" << duration_feature_extraction/duration_total << ")" << std::endl;
  std::cerr << "duration      triangulation (s): " << duration_triangulation << " (" << duration_triangulation/duration_total << ")" << std::endl;
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

void regularize(const std::vector<cv::KeyPoint>& keypoints_) {

  //ds keypoint pruning - preemptively filter keypoints by bin
  const Count bin_size_u_half = std::round(bin_size_u/2);
  const Count bin_size_v_half = std::round(bin_size_v/2);

  //ds for all grids
  for (Count index_grid = 0; index_grid < number_of_grids; ++index_grid) {

    //ds in all its beauty it allows for direct constant initialization
    const Count u_offset = index_grid&1?0:bin_size_u_half;
    const Count v_offset = (index_grid>>1)?0:bin_size_v_half;

    //ds check all keypoints for this grid
    Count u_current = 0;
    for (const cv::KeyPoint& keypoint: keypoints_) {
      const gt_real& keypoint_u = keypoint.pt.x;
      const gt_real& keypoint_v = keypoint.pt.y;

      //ds if the keypoint still enters the current bin
      if (keypoint_u < u_current*bin_size_u+u_offset) {

        //ds check matching bin range in V
        for (Count v = 0; v < number_of_bins_v; ++v) {
          if (keypoint_v < v*bin_size_v+v_offset) {

            //ds found matching bin - check if the reponse is higher
            if (keypoint.response > bin_map_left[index_grid][v][u_current].response) {
              bin_map_left[index_grid][v][u_current] = keypoint;
            }
            break;
          }
        }
      } else {
        ++u_current;
      }
    }
    assert(u_current < number_of_bins_u);
  }
}
