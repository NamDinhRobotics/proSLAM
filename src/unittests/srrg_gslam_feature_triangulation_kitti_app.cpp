#include "qapplication.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "ui/gt_tracker_viewer.h"
#include "ui/simple_point_viewer.h"
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

//ds triangulation constants - determined by camera intrinsics
gt_real triangulation_F           = 0;
gt_real triangulation_Finverse    = 0;
gt_real triangulation_Pu          = 0;
gt_real triangulation_Pv          = 0;
gt_real triangulation_DuR         = 0;
gt_real triangulation_DuR_flipped = 0;

//ds read cameras
StringCameraMap cameras_by_topic;

//ds ui control
int32_t cv_wait_key_timeout_milliseconds = 0;

const bool displayImages(const cv::Mat& image_left_, const cv::Mat& image_right_);
const PointCoordinates _getCoordinatesByTriangulation(const cv::KeyPoint& keypoint_left, const cv::KeyPoint& keypoint_right_);

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
    feature_detector = std::make_shared<cv::FastFeatureDetector>();
    descriptor_extractor = std::make_shared<cv::BriefDescriptorExtractor>(cv::BriefDescriptorExtractor(DESCRIPTOR_SIZE_BYTES));
#elif CV_MAJOR_VERSION == 3
    feature_detector = cv::FastFeatureDetector::create();
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

  //ds prepare cameras
  cameras_by_topic.clear();

  //ds keypoint maps TODO check resolution constraint
  int32_t keypoint_map_left[376][1241];
  int32_t keypoint_map_right[376][1241];
  const gt_real maximum_matching_distance_triangulation = 50;

  //ds statistics
  Count number_of_frames = 0;
  double duration_feature_detection  = 0;
  double duration_feature_extraction = 0;
  double duration_triangulation      = 0;
  const double time_start_total = getTime();
  Count total_number_of_triangulations = 0;

  //ds start playback
  BaseMessage* base_message = 0;
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

      //ds cameras
      Camera* camera_left  = 0;
      Camera* camera_right = 0;

      //ds attempt to set them - otherwise create new camera
      try {
        camera_left = cameras_by_topic.at(message_image_left->topic());
      } catch(const std::out_of_range& /*exception*/) {

        //ds allocate a new camera
        camera_left = new Camera(message_image_left->image().rows,
                                 message_image_left->image().cols,
                                 message_image_left->cameraMatrix().cast<gt_real>(),
                                 message_image_left->offset().cast<gt_real>());
        cameras_by_topic.insert(std::make_pair(message_image_left->topic(), camera_left));

        //ds set triangulation properties
        triangulation_F        = camera_left->projectionMatrix()(0,0);
        triangulation_Finverse = 1/triangulation_F;
        triangulation_Pu       = camera_left->projectionMatrix()(0,2);
        triangulation_Pv       = camera_left->projectionMatrix()(1,2);

      }
      try {
        camera_right = cameras_by_topic.at(message_image_right->topic());
      } catch(const std::out_of_range& /*exception*/) {

        //ds allocate a new camera
        camera_right = new Camera(message_image_right->image().rows,
                                  message_image_right->image().cols,
                                  message_image_right->cameraMatrix().cast<gt_real>(),
                                  message_image_right->offset().cast<gt_real>());
        cameras_by_topic.insert(std::make_pair(message_image_right->topic(), camera_right));

        //ds set triangulation properties
        triangulation_DuR         = camera_right->projectionMatrix()(0,3);
        triangulation_DuR_flipped = -triangulation_DuR;
      }







      //ds reset maps
      for (uint32_t row = 0; row < 376; ++row) {
        for (uint32_t col = 0; col < 1241; ++col) {
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
      for (uint32_t row = 0; row < 376; ++row) {

        //ds last found on right in this column - blocking the search range to the left
        int32_t col_right_last = 0;

        //ds always starting from the left
        for (uint32_t col_left = 0; col_left < 1241; ++col_left) {

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
          points_triangulated[index_point_triangulated] = _getCoordinatesByTriangulation(keypoints_left[stereo_points[u].first], keypoints_right[stereo_points[u].second]);

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
















      //ds set viewer items
      std::vector<PointDrawable> drawables(index_point_triangulated);
      for (uint32_t u = 0; u < index_point_triangulated; ++u) {
        if (points_triangulated[u].z() < 25) {
          drawables[u] = std::make_pair(points_triangulated[u], PointColorRGB(0, 0, 1));
        } else {
          drawables[u] = std::make_pair(points_triangulated[u], PointColorRGB(1, 0, 1));
        }
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

const PointCoordinates _getCoordinatesByTriangulation(const cv::KeyPoint& keypoint_left, const cv::KeyPoint& keypoint_right_) {

  //ds check for minimal disparity
  if (keypoint_left.pt.x-keypoint_right_.pt.x < 1) {
    throw std::runtime_error("disparity value to low");
  }

  //ds input validation
  assert(keypoint_right_.pt.x < keypoint_left.pt.x);
  assert(keypoint_right_.pt.y == keypoint_left.pt.y);

  //ds first compute depth (z in camera)
  const gt_real depth_meters = triangulation_DuR_flipped/(keypoint_left.pt.x-keypoint_right_.pt.x);
  assert(depth_meters >= 0);

  //ds set 3d point
  const PointCoordinates coordinates_in_camera(triangulation_Finverse*depth_meters*(keypoint_left.pt.x-triangulation_Pu),
                                               triangulation_Finverse*depth_meters*(keypoint_left.pt.y-triangulation_Pv),
                                               depth_meters);

  //ds return triangulated point
  return coordinates_in_camera;
}
