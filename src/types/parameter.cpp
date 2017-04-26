#include "parameter.h"

//ds listed below are the default parameter values of the system - not necessarily supporting each configuration
namespace proslam {

  //ds informative
  std::string Parameter::banner =
  "-------------------------------------------------------------------------\n"
  "srrg_proslam_app: simple SLAM application\n"
  "usage: srrg_proslam_app [options] <dataset>\n"
  "\n"
  "<dataset>: path to a SRRG txt_io dataset file\n"
  "\n"
  "[options]\n"
  "-topic-image-left (-il)        <string>: sets left image topic name (txt_io, ROS)\n"
  "-topic-image-right (-ir)       <string>: sets right image topic name (txt_io, ROS)\n"
  "-topic-camera-info-left (-cl)  <string>: sets left camera info topic (ROS)\n"
  "-topic-camera-info-right (-cr) <string>: sets right camera info topic (ROS)\n"
  "-use-gui (-ug):                          displays GUI elements\n"
  "-use-odometry (-uo):                     uses odometry instead of inner motion model for prediction\n"
  "-depth-mode (-dm):                       depth tracking (-topic-image-left: intensity image, -topic-image-right: depth)\n"
  "-open:                                   disables relocalization (open loop mode)\n"
  "-show-top (-st):                         enable top map viewer\n"
  "-drop-framepoints (-df):                 deallocation of past framepoints at runtime (reduces memory demand)\n"
  "-equalize-histogram (-eh):               equalize stereo image histogram before processing\n"
  "-rectify-and-undistort (-ru):            rectifies and undistorts input images based on camera info\n"
  "-------------------------------------------------------------------------";

  //ds files/topics
  std::string Parameter::topic_image_left        = "/camera_left/image_raw";
  std::string Parameter::topic_image_right       = "/camera_right/image_raw";
  std::string Parameter::topic_camera_info_left  = "";
  std::string Parameter::topic_camera_info_right = "";
  std::string Parameter::filename_dataset        = "";

  //ds options
  bool Parameter::option_use_gui               = false;
  bool Parameter::option_use_odometry          = false;
  bool Parameter::option_use_relocalization    = true;
  bool Parameter::option_show_top_viewer       = false;
  bool Parameter::option_drop_framepoints      = false;
  bool Parameter::option_equalize_histogram    = false;
  bool Parameter::option_rectify_and_undistort = false;
  Parameter::TrackerMode Parameter::tracker_mode = Parameter::TrackerMode::Stereo;



  //ds Frame: this criteria is used for the decision of creating a landmark or not from a track of framepoints
  Count Parameter::Types::minimum_track_length_for_landmark_creation = 3;

  //ds Landmark: minimum number of measurements before optimization is filtering
  Count Parameter::Types::minimum_number_of_forced_updates = 2;

  //ds Landmark: maximum allowed measurement divergence
  real Parameter::Types::maximum_translation_error_to_depth_ratio = 1;

  //ds LocalMap: target minimum number of landmarks for local map creation
  Count Parameter::Types::minimum_number_of_landmarks = 50;

  //ds WorldMap: key frame generation properties
  real Parameter::Types::minimum_distance_traveled_for_local_map = 0.5;
  real Parameter::Types::minimum_degrees_rotated_for_local_map   = 0.5;
  Count Parameter::Types::minimum_number_of_frames_for_local_map = 4;



  //ds dynamic thresholds for feature detection
  real Parameter::FramepointGeneration::target_number_of_keypoints_tolerance = 0.1;
  int32_t Parameter::FramepointGeneration::detector_threshold         = 5;
  int32_t Parameter::FramepointGeneration::detector_threshold_minimum = 5;
  real Parameter::FramepointGeneration::detector_threshold_step_size  = 5;

  //ds dynamic thresholds for descriptor matching
  int32_t Parameter::FramepointGeneration::matching_distance_tracking_threshold         = 50;
  int32_t Parameter::FramepointGeneration::matching_distance_tracking_threshold_maximum = 50;
  int32_t Parameter::FramepointGeneration::matching_distance_tracking_threshold_minimum = 25;
  int32_t Parameter::FramepointGeneration::matching_distance_tracking_step_size         = 1;

  //ds stereo: triangulation
  int32_t Parameter::FramepointGeneration::maximum_matching_distance_triangulation = 50;
  real Parameter::FramepointGeneration::baseline_factor                            = 50;
  real Parameter::FramepointGeneration::minimum_disparity_pixels                   = 1;



  //ds track lost criteria
  Count Parameter::MotionEstimation::minimum_number_of_landmarks_to_track = 5;

  //ds point tracking thresholds
  int32_t Parameter::MotionEstimation::pixel_distance_tracking_threshold_maximum = 7*7; //ds upper limit: 7x7 pixels
  int32_t Parameter::MotionEstimation::pixel_distance_tracking_threshold_minimum = 4*4; //ds lower limit: 4x4 pixels
  int32_t Parameter::MotionEstimation::range_point_tracking                      = 2;           //ds pixel search range width for point vicinity tracking
  int32_t Parameter::MotionEstimation::maximum_flow_pixels_squared               = 150*150;     //ds maximum allowed pixel distance between image coordinates prediction and actual detection

  //ds feature density regularization
  Count Parameter::MotionEstimation::bin_size_pixels        = 20;
  real Parameter::MotionEstimation::ratio_keypoints_to_bins = 1.5;
}
