#include <iostream>

#include "srrg_messages/message_reader.h"
#include "srrg_messages/message_timestamp_synchronizer.h"
#include "srrg_messages/pinhole_image_message.h"

const bool measure(const cv::Mat& image_,
                   const cv::Size& board_size_,
                   const double& square_width_meters_,
                   std::vector<cv::Point3f>& object_points_,
                   std::vector<std::vector<cv::Point3f>>& object_points_per_image_,
                   std::vector<std::vector<cv::Point2f>>& image_points_per_image_,
                   cv::Mat& image_display_);

const double calibrate(const cv::Size image_size_,
                       std::vector<std::vector<cv::Point3f>>& object_points_per_image_,
                       std::vector<std::vector<cv::Point2f>>& image_points_per_image_) {

  //ds objectives
  cv::Mat camera_calibration_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distortion_coefficients   = cv::Mat::zeros(8, 1, CV_64F);
  std::vector<cv::Mat> rotations_per_image;
  std::vector<cv::Mat> translations_per_image;

  //ds we have a fixed aspect ratio
  camera_calibration_matrix.at<double>(0,0) = static_cast<float>(image_size_.width)/image_size_.height;

  //ds compute camera calibration matrix and distortion coefficients
  const double reprojection_error_pixels = cv::calibrateCamera(object_points_per_image_,
                                                               image_points_per_image_,
                                                               image_size_,
                                                               camera_calibration_matrix,
                                                               distortion_coefficients,
                                                               rotations_per_image,
                                                               translations_per_image,
                                                               CV_CALIB_FIX_ASPECT_RATIO);

  std::cerr << camera_calibration_matrix << std::endl;
  std::cerr << distortion_coefficients << std::endl;

  //ds done
  return reprojection_error_pixels;
}

int32_t main (int32_t argc, char** argv) {
  if (argc != 2) {
    std::cerr << "use: ./stereo_calibrator <BOSS/TXTIO_message_file>" << std::endl;
  }

  //ds configuration
  std::string file_name_messages = argv[1];
  std::string topic_image_left   = "/camera_left/image_raw";
  std::string topic_image_right  = "/camera_right/image_raw";

  //ds board configuration
  const uint32_t cols = 6;
  const uint32_t rows = 7;
  const cv::Size board_size(cols, rows);
  const double square_width_meters  = 0.06;
  const cv::Size image_size(100,100);

  //ds log configuration
  std::cerr << "file_name_messages: " << file_name_messages << std::endl;
  std::cerr << "topic_image_left: " << topic_image_left << std::endl;
  std::cerr << "topic_image_right: " << topic_image_right << std::endl;

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
  uint64_t number_of_processed_stereo_images      = 0;
  uint64_t number_of_processed_chessboards_stereo = 0;

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

      //ds locate chessboard in the left and right image
      const bool measured_left  = measure(image_left, board_size, square_width_meters, object_points,
                                          object_points_per_image_left, image_points_per_image_left, image_display_left);
      const bool measured_right = measure(image_right, board_size, square_width_meters, object_points,
                                          object_points_per_image_right, image_points_per_image_right, image_display_right);

      //ds if both images contained the pattern
      if (measured_left >= 0 && measured_right >= 0) {
        ++number_of_processed_chessboards_stereo;
      }

      //ds visual info
      cv::Mat image_stereo;
      cv::hconcat(image_display_left, image_display_right, image_stereo);
      cv::imshow(topic_image_left + ", " + topic_image_right, image_stereo);
      cv::waitKey(1);

      //ds status
      std::printf("%06lu|L: %f|R: %f|CL: %i CR: %i|"
                  "L: %6lu R: %6lu S: %6lu\n", number_of_processed_stereo_images,
                                               image_message_left->timestamp(), image_message_right->timestamp(),
                                               measured_left, measured_right,
                                               object_points_per_image_left.size(), object_points_per_image_right.size(), number_of_processed_chessboards_stereo);

      //ds release processed data
      image_message_left->release();
      image_message_right->release();
      image_left.release();
      image_right.release();
      synchronizer.reset();
      ++number_of_processed_stereo_images;
    }
  }

  std::cerr << "obtained measurements LEFT: " << object_points_per_image_left.size() << std::endl;
  std::cerr << "obtained measurements RIGHT: " << object_points_per_image_right.size() << std::endl;
  std::cerr << "obtained measurements STEREO: " << number_of_processed_chessboards_stereo << std::endl;

  //ds calibrate on all measurements
  const double reprojection_error_left  = calibrate(image_size, object_points_per_image_left, image_points_per_image_left);
  const double reprojection_error_right = calibrate(image_size, object_points_per_image_right, image_points_per_image_right);

  std::cerr << "reprojection error LEFT: " << reprojection_error_left << std::endl;
  std::cerr << "reprojection error RIGHT: " << reprojection_error_right << std::endl;

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
