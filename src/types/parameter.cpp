#include "parameter.h"

namespace proslam {

  //ds CommandLine

    std::string Parameter::CommandLine::banner =
    "-------------------------------------------------------------------------\n"
    "srrg_proslam_app: simple SLAM application\n"
    "usage: srrg_proslam_app [options] <dataset>\n"
    "\n"
    "<dataset>: path to a SRRG txt_io dataset file\n"
    "\n"
    "[options]\n"
    "-configuration (-c)            <string>: path to configuration file to load\n"
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

    std::string Parameter::CommandLine::topic_image_left        = "/camera_left/image_raw";
    std::string Parameter::CommandLine::topic_image_right       = "/camera_right/image_raw";
    std::string Parameter::CommandLine::topic_camera_info_left  = "";
    std::string Parameter::CommandLine::topic_camera_info_right = "";
    std::string Parameter::CommandLine::filename_dataset        = "";
    std::string Parameter::CommandLine::filename_configuration  = "";

    bool Parameter::CommandLine::option_use_gui                 = false;
    bool Parameter::CommandLine::option_use_odometry            = false;
    bool Parameter::CommandLine::option_use_relocalization      = true;
    bool Parameter::CommandLine::option_show_top_viewer         = false;
    bool Parameter::CommandLine::option_drop_framepoints        = false;
    bool Parameter::CommandLine::option_equalize_histogram      = false;
    bool Parameter::CommandLine::option_rectify_and_undistort   = false;
    Parameter::TrackerMode Parameter::CommandLine::tracker_mode = Parameter::TrackerMode::Stereo;

  //Types

    Count Parameter::Types::minimum_track_length_for_landmark_creation = 3;
    Count Parameter::Types::minimum_number_of_forced_updates           = 2;
    real Parameter::Types::maximum_translation_error_to_depth_ratio    = 1;
    Count Parameter::Types::minimum_number_of_landmarks                = 50;

    real Parameter::Types::minimum_distance_traveled_for_local_map = 0.5;
    real Parameter::Types::minimum_degrees_rotated_for_local_map   = 0.5;
    Count Parameter::Types::minimum_number_of_frames_for_local_map = 4;

  //FramepointGeneration

    real Parameter::FramepointGeneration::target_number_of_keypoints_tolerance = 0.1;
    int32_t Parameter::FramepointGeneration::detector_threshold                = 5;
    int32_t Parameter::FramepointGeneration::detector_threshold_minimum        = 5;
    real Parameter::FramepointGeneration::detector_threshold_step_size         = 5;

    int32_t Parameter::FramepointGeneration::matching_distance_tracking_threshold         = 50;
    int32_t Parameter::FramepointGeneration::matching_distance_tracking_threshold_maximum = 50;
    int32_t Parameter::FramepointGeneration::matching_distance_tracking_threshold_minimum = 15;
    int32_t Parameter::FramepointGeneration::matching_distance_tracking_step_size         = 1;

    int32_t Parameter::FramepointGeneration::maximum_matching_distance_triangulation = 50;
    real Parameter::FramepointGeneration::baseline_factor                            = 50;
    real Parameter::FramepointGeneration::minimum_disparity_pixels                   = 1;

  //MotionEstimation

    Count Parameter::MotionEstimation::minimum_number_of_landmarks_to_track = 5;

    int32_t Parameter::MotionEstimation::minimum_threshold_distance_tracking_pixels = 4*4;
    int32_t Parameter::MotionEstimation::maximum_threshold_distance_tracking_pixels = 7*7;
    int32_t Parameter::MotionEstimation::range_point_tracking                       = 2;
    int32_t Parameter::MotionEstimation::maximum_distance_tracking_pixels           = 150*150;

    Count Parameter::MotionEstimation::maximum_number_of_landmark_recoveries = 3;

    Count Parameter::MotionEstimation::bin_size_pixels        = 16;
    real Parameter::MotionEstimation::ratio_keypoints_to_bins = 1;

  //Relocalization

    Count Parameter::Relocalization::preliminary_minimum_interspace_queries = 5;
    real Parameter::Relocalization::preliminary_minimum_matching_ratio      = 0.1;
    Count Parameter::Relocalization::minimum_number_of_matches_per_landmark = 100;
    Count Parameter::Relocalization::minimum_matches_per_correspondence     = 0;

  //MapOptimization

  //Visualization
}
