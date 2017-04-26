#pragma once
#include "definitions.h"

namespace proslam {

  //ds this class serves as a singleton parameter base for the ProSLAM pipeline
  class Parameter {
    
  //ds exported types
  public:

    //ds SLAM system tracker modes
    enum TrackerMode {Stereo, //ds stereo image processing
                      Depth}; //ds rgb + depth image processing

  //ds functionality
  public:

    static void parseParametersFromFile(const std::string& file_) {

    }

    static void parseParametersFromCommandLine(int32_t argc, char ** argv) {
      int32_t number_of_added_parameters = 1;
      while(number_of_added_parameters < argc){
        if (!std::strcmp(argv[number_of_added_parameters], "-topic-image-left") || !std::strcmp(argv[number_of_added_parameters], "-il")){
          number_of_added_parameters++;
          topic_image_left = argv[number_of_added_parameters];
        } else if (!std::strcmp(argv[number_of_added_parameters], "-topic-image-right") || !std::strcmp(argv[number_of_added_parameters], "-ir")){
          number_of_added_parameters++;
          topic_image_right = argv[number_of_added_parameters];
        } else if (!std::strcmp(argv[number_of_added_parameters], "-topic-camera-info-left") || !std::strcmp(argv[number_of_added_parameters], "-cl")){
          number_of_added_parameters++;
          topic_camera_info_left = argv[number_of_added_parameters];
        } else if (!std::strcmp(argv[number_of_added_parameters], "-topic-camera-info-right") || !std::strcmp(argv[number_of_added_parameters], "-cr")){
          number_of_added_parameters++;
          topic_camera_info_right = argv[number_of_added_parameters];
        } else if (!std::strcmp(argv[number_of_added_parameters], "-h") || !std::strcmp(argv[number_of_added_parameters], "--h")) {
          printBanner();
          exit(0);
        } else if (!std::strcmp(argv[number_of_added_parameters], "-use-gui") || !std::strcmp(argv[number_of_added_parameters], "-ug")) {
          option_use_gui = true;
        } else if (!std::strcmp(argv[number_of_added_parameters], "-open")) {
          option_use_relocalization = false;
        } else if (!std::strcmp(argv[number_of_added_parameters], "-show-top") || !std::strcmp(argv[number_of_added_parameters], "-st")) {
          option_show_top_viewer = true;
        } else if (!std::strcmp(argv[number_of_added_parameters], "-drop-framepoints") || !std::strcmp(argv[number_of_added_parameters], "-df")) {
          option_drop_framepoints = true;
        } else if (!std::strcmp(argv[number_of_added_parameters], "-equalize-histogram") || !std::strcmp(argv[number_of_added_parameters], "-eh")) {
          option_equalize_histogram = true;
        } else if (!std::strcmp(argv[number_of_added_parameters], "-rectify-and-undistort") || !std::strcmp(argv[number_of_added_parameters], "-ru")) {
          option_rectify_and_undistort = true;
        } else if (!std::strcmp(argv[number_of_added_parameters], "-depth-mode") || !std::strcmp(argv[number_of_added_parameters], "-dm")) {
          tracker_mode = Depth;
        } else if (!std::strcmp(argv[number_of_added_parameters], "-use-odometry") || !std::strcmp(argv[number_of_added_parameters], "-uo")) {
          option_use_odometry = true;
        } else {
          filename_dataset = argv[number_of_added_parameters];
        }
        number_of_added_parameters++;
      }

      //ds validate input parameters and exit on failure
      validateParameters();
    }

    static void validateParameters() {

      //ds check camera topics
      if (topic_image_left.length() == 0) {
        std::cerr << "ERROR: empty value entered for parameter: -topic-image-left (-il) (enter -h for help)" << std::endl;
        exit(0);
      }
      if (topic_image_right.length() == 0) {
        std::cerr << "ERROR: empty value entered for parameter: -topic-image-right (-ir) (enter -h for help)" << std::endl;
        exit(0);
      }
    }

    static void printCommandLineParameters() {
      std::cerr << "-------------------------------------------------------------------------" << std::endl;
      std::cerr << "running with command line parameters:" << std::endl;
      std::cerr << "-topic-image-left        " << topic_image_left << std::endl;
      std::cerr << "-topic-image-right       " << topic_image_right << std::endl;
      if (topic_camera_info_left.length() > 0) std::cerr << "-topic-camera-left-info  " << topic_camera_info_left << std::endl;
      if (topic_camera_info_right.length() > 0) std::cerr << "-topic-camera-right-info " << topic_camera_info_right << std::endl;
      std::cerr << "-use-gui                 " << option_use_gui << std::endl;
      std::cerr << "-open                    " << !option_use_relocalization << std::endl;
      std::cerr << "-show-top                " << option_show_top_viewer << std::endl;
      std::cerr << "-use-odometry            " << option_use_odometry << std::endl;
      std::cerr << "-depth-mode              " << (tracker_mode==TrackerMode::Depth) << std::endl;
      std::cerr << "-drop-framepoints        " << option_drop_framepoints << std::endl;
      std::cerr << "-equalize-histogram      " << option_equalize_histogram << std::endl;
      std::cerr << "-rectify-and-undistort   " << option_rectify_and_undistort << std::endl;
      if (filename_dataset.length() > 0) std::cerr << "-dataset                 " << filename_dataset << std::endl;
      std::cerr << "-------------------------------------------------------------------------" << std::endl;
    }

    static void printBanner() {std::cerr << banner << std::endl;}

  //ds command line parameters
  public:

    //ds informative
    static std::string banner;

    //ds files/topics
    static std::string topic_image_left;
    static std::string topic_image_right;
    static std::string topic_camera_info_left;
    static std::string topic_camera_info_right;
    static std::string filename_dataset;

    //ds options
    static bool option_use_gui;
    static bool option_use_relocalization;
    static bool option_show_top_viewer;
    static bool option_drop_framepoints;
    static bool option_equalize_histogram;
    static bool option_rectify_and_undistort;
    static bool option_use_odometry;
    static TrackerMode tracker_mode;

  //ds system parameters
  public:

    struct Types {

      //ds Frame: this criteria is used for the decision of creating a landmark or not from a track of framepoints
      static Count minimum_track_length_for_landmark_creation;

      //ds Landmark: minimum number of measurements before optimization is filtering
      static Count minimum_number_of_forced_updates;

      //ds Landmark: maximum allowed measurement divergence
      static real maximum_translation_error_to_depth_ratio;

      //ds LocalMap: target minimum number of landmarks for local map creation
      static Count minimum_number_of_landmarks;

      //ds WorldMap: key frame generation properties
      static real minimum_distance_traveled_for_local_map;
      static real minimum_degrees_rotated_for_local_map;
      static Count minimum_number_of_frames_for_local_map;
    };

    struct FramepointGeneration {

      //ds dynamic thresholds for feature detection
      static real target_number_of_keypoints_tolerance;
      static int32_t detector_threshold;
      static int32_t detector_threshold_minimum;
      static real detector_threshold_step_size;

      //ds dynamic thresholds for descriptor matching
      static int32_t matching_distance_tracking_threshold;
      static int32_t matching_distance_tracking_threshold_maximum;
      static int32_t matching_distance_tracking_threshold_minimum;
      static int32_t matching_distance_tracking_step_size;

      //ds stereo: triangulation
      static int32_t maximum_matching_distance_triangulation;
      static real baseline_factor;
      static real minimum_disparity_pixels;
    };

    struct MotionEstimation {

      //ds track lost criteria
      static Count minimum_number_of_landmarks_to_track;

      //ds point tracking thresholds
      static int32_t pixel_distance_tracking_threshold_maximum; //ds upper limit: 7x7 pixels
      static int32_t pixel_distance_tracking_threshold_minimum; //ds lower limit: 4x4 pixels
      static int32_t range_point_tracking;           //ds pixel search range width for point vicinity tracking
      static int32_t maximum_flow_pixels_squared;     //ds maximum allowed pixel distance between image coordinates prediction and actual detection

      //ds feature density regularization
      static Count bin_size_pixels;
      static real ratio_keypoints_to_bins;
    };

    struct Relocalization {

    };

    struct MapOptimization {

    };

    struct Visualization {

    };
  };
}
