#pragma once
#include "definitions.h"
#include "yaml-cpp/yaml.h"

namespace proslam {

  //! @class Parameter
  //! @brief this class serves as a singleton parameter base for the ProSLAM pipeline
  class Parameter {

    //! @brief macro wrapping the YAML node parsing for a single parameter
    //! @param YAML_NODE the target root YAML node
    //! @param STRUCT_NAME parameter Struct name (e.g. CommandLine)
    //! @param PARAMETER_NAME parameter name (e.g. topic_image_left)
    //! @param PARAMETER_TYPE parameter type (e.g. std::string)
    #define PARSE_PARAMETER(YAML_NODE, STRUCT_NAME, PARAMETER_NAME, PARAMETER_TYPE) \
    try { \
      STRUCT_NAME::PARAMETER_NAME = YAML_NODE[#STRUCT_NAME][#PARAMETER_NAME].as<PARAMETER_TYPE>(); \
      /*std::cerr << "parameter name: '" << #STRUCT_NAME << "::" << #PARAMETER_NAME << "' value: '" << STRUCT_NAME::PARAMETER_NAME << "'" << std::endl;*/ \
    } catch (const YAML::TypedBadConversion<PARAMETER_TYPE>& exception_) { \
      std::cerr << "unable to parse parameter: '" << #STRUCT_NAME << "::" << #PARAMETER_NAME << "' - exception: '" << exception_.what() << "'" << std::endl; \
    }

  //ds exported types
  public:

    //! @brief SLAM system tracker modes
    enum TrackerMode {Stereo, //ds stereo image processing
                      Depth}; //ds rgb + depth image processing

  //ds functionality
  public:

    //! @brief utility parsing parameters from a file (YAML)
    static void parseParametersFromFile(const std::string& filename_) {

      try {

        //ds attempt to open the configuration file and parse it into a YAML node
        YAML::Node configuration = YAML::LoadFile(filename_);

        //CommandLine
        PARSE_PARAMETER(configuration, CommandLine, topic_image_left, std::string)
        PARSE_PARAMETER(configuration, CommandLine, topic_image_right, std::string)
        PARSE_PARAMETER(configuration, CommandLine, topic_camera_info_left, std::string)
        PARSE_PARAMETER(configuration, CommandLine, topic_camera_info_right, std::string)
        PARSE_PARAMETER(configuration, CommandLine, filename_dataset, std::string)
        PARSE_PARAMETER(configuration, CommandLine, option_use_gui, bool)
        PARSE_PARAMETER(configuration, CommandLine, option_use_odometry, bool)
        PARSE_PARAMETER(configuration, CommandLine, option_use_relocalization, bool)
        PARSE_PARAMETER(configuration, CommandLine, option_show_top_viewer, bool)
        PARSE_PARAMETER(configuration, CommandLine, option_drop_framepoints, bool)
        PARSE_PARAMETER(configuration, CommandLine, option_equalize_histogram, bool)
        PARSE_PARAMETER(configuration, CommandLine, option_rectify_and_undistort, bool)

        //Types
        PARSE_PARAMETER(configuration, Types, minimum_track_length_for_landmark_creation, Count)
        PARSE_PARAMETER(configuration, Types, minimum_number_of_forced_updates, Count)
        PARSE_PARAMETER(configuration, Types, maximum_translation_error_to_depth_ratio, real)
        PARSE_PARAMETER(configuration, Types, minimum_number_of_landmarks, Count)
        PARSE_PARAMETER(configuration, Types, minimum_distance_traveled_for_local_map, real)
        PARSE_PARAMETER(configuration, Types, minimum_degrees_rotated_for_local_map, real)
        PARSE_PARAMETER(configuration, Types, minimum_number_of_frames_for_local_map, Count)

        //FramepointGeneration
        PARSE_PARAMETER(configuration, FramepointGeneration, target_number_of_keypoints_tolerance, real)
        PARSE_PARAMETER(configuration, FramepointGeneration, detector_threshold, int32_t)
        PARSE_PARAMETER(configuration, FramepointGeneration, detector_threshold_minimum, int32_t)
        PARSE_PARAMETER(configuration, FramepointGeneration, detector_threshold_step_size, real)
        PARSE_PARAMETER(configuration, FramepointGeneration, matching_distance_tracking_threshold, int32_t)
        PARSE_PARAMETER(configuration, FramepointGeneration, matching_distance_tracking_threshold_maximum, int32_t)
        PARSE_PARAMETER(configuration, FramepointGeneration, matching_distance_tracking_threshold_minimum, int32_t)
        PARSE_PARAMETER(configuration, FramepointGeneration, matching_distance_tracking_step_size, int32_t)
        PARSE_PARAMETER(configuration, FramepointGeneration, maximum_matching_distance_triangulation, int32_t)
        PARSE_PARAMETER(configuration, FramepointGeneration, baseline_factor, real)
        PARSE_PARAMETER(configuration, FramepointGeneration, minimum_disparity_pixels, real)

        //MotionEstimation
        PARSE_PARAMETER(configuration, MotionEstimation, minimum_number_of_landmarks_to_track, Count)
        PARSE_PARAMETER(configuration, MotionEstimation, maximum_threshold_distance_tracking_pixels, int32_t)
        PARSE_PARAMETER(configuration, MotionEstimation, minimum_threshold_distance_tracking_pixels, int32_t)
        PARSE_PARAMETER(configuration, MotionEstimation, range_point_tracking, int32_t)
        PARSE_PARAMETER(configuration, MotionEstimation, maximum_distance_tracking_pixels, int32_t)
        PARSE_PARAMETER(configuration, MotionEstimation, maximum_number_of_landmark_recoveries, Count)
        PARSE_PARAMETER(configuration, MotionEstimation, bin_size_pixels, Count)
        PARSE_PARAMETER(configuration, MotionEstimation, ratio_keypoints_to_bins, real)

        //Relocalization
        PARSE_PARAMETER(configuration, Relocalization, preliminary_minimum_interspace_queries, Count)
        PARSE_PARAMETER(configuration, Relocalization, preliminary_minimum_matching_ratio, real)
        PARSE_PARAMETER(configuration, Relocalization, minimum_number_of_matches_per_landmark, Count)
        PARSE_PARAMETER(configuration, Relocalization, minimum_matches_per_correspondence, Count)

        //ds done
        std::cerr << "Parameter::parseParametersFromFile|successfully loaded configuration from file: " << filename_ << std::endl;
      } catch (const YAML::BadFile& exception_) {
        std::cerr << "Parameter::parseParametersFromFile|ERROR: unable to parse configuration file: " << filename_ << " - exception: '" << exception_.what() << "'" << std::endl;
      }
    }

    //! @brief utility parsing command line parameters
    static void parseParametersFromCommandLine(const int32_t& argc_, char ** argv_) {

      //ds skim the command line for configuration file input
      int32_t number_of_checked_parameters = 1;
      while(number_of_checked_parameters < argc_){
        if (!std::strcmp(argv_[number_of_checked_parameters], "-configuration") || !std::strcmp(argv_[number_of_checked_parameters], "-c")){
          number_of_checked_parameters++;
          if (number_of_checked_parameters == argc_) {break;}
          CommandLine::filename_configuration = argv_[number_of_checked_parameters];
        }
        number_of_checked_parameters++;
      }

      //ds if no configuration file was specified
      if (CommandLine::filename_configuration.length() == 0) {

        //ds check if a default configuration file is present
        if (srrg_core::isAccessible("configuration.yaml")) {
          CommandLine::filename_configuration = "configuration.yaml";
          std::cerr << "Parameter::parseParametersFromCommandLine|loading default configuration file: " << CommandLine::filename_configuration << std::endl;
        } else {
          std::cerr << "Parameter::parseParametersFromCommandLine|WARNING: no configuration file specified and no default configuration present (running with internal configuration)" << std::endl;
        }
      } else {

        //ds check if specified configuration file is not accessible
        if (!srrg_core::isAccessible(CommandLine::filename_configuration)) {
          std::cerr << "Parameter::parseParametersFromCommandLine|ERROR: specified configuration file is not accessible: " << CommandLine::filename_configuration << std::endl;
          exit(0);
        }
      }

      //ds if a valid configuration file is set (otherwise the field must be empty)
      if (CommandLine::filename_configuration.length()) {

        //ds parse parameters from configuration file
        parseParametersFromFile(CommandLine::filename_configuration);
      }

      //ds reset and check for other command line parameters, potentially overwritting the ones set in the configuration file
      number_of_checked_parameters = 1;
      while(number_of_checked_parameters < argc_){
        if (!std::strcmp(argv_[number_of_checked_parameters], "-topic-image-left") || !std::strcmp(argv_[number_of_checked_parameters], "-il")){
          number_of_checked_parameters++;
          if (number_of_checked_parameters == argc_) {break;}
          CommandLine::topic_image_left = argv_[number_of_checked_parameters];
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-topic-image-right") || !std::strcmp(argv_[number_of_checked_parameters], "-ir")){
          number_of_checked_parameters++;
          if (number_of_checked_parameters == argc_) {break;}
          CommandLine::topic_image_right = argv_[number_of_checked_parameters];
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-topic-camera-info-left") || !std::strcmp(argv_[number_of_checked_parameters], "-cl")){
          number_of_checked_parameters++;
          if (number_of_checked_parameters == argc_) {break;}
          CommandLine::topic_camera_info_left = argv_[number_of_checked_parameters];
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-topic-camera-info-right") || !std::strcmp(argv_[number_of_checked_parameters], "-cr")){
          number_of_checked_parameters++;
          if (number_of_checked_parameters == argc_) {break;}
          CommandLine::topic_camera_info_right = argv_[number_of_checked_parameters];
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-h") || !std::strcmp(argv_[number_of_checked_parameters], "--h")) {
          printBanner();
          exit(0);
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-use-gui") || !std::strcmp(argv_[number_of_checked_parameters], "-ug")) {
          CommandLine::option_use_gui = true;
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-open")) {
          CommandLine::option_use_relocalization = false;
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-show-top") || !std::strcmp(argv_[number_of_checked_parameters], "-st")) {
          CommandLine::option_show_top_viewer = true;
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-drop-framepoints") || !std::strcmp(argv_[number_of_checked_parameters], "-df")) {
          CommandLine::option_drop_framepoints = true;
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-equalize-histogram") || !std::strcmp(argv_[number_of_checked_parameters], "-eh")) {
          CommandLine::option_equalize_histogram = true;
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-rectify-and-undistort") || !std::strcmp(argv_[number_of_checked_parameters], "-ru")) {
          CommandLine::option_rectify_and_undistort = true;
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-depth-mode") || !std::strcmp(argv_[number_of_checked_parameters], "-dm")) {
          CommandLine::tracker_mode = Depth;
        } else if (!std::strcmp(argv_[number_of_checked_parameters], "-use-odometry") || !std::strcmp(argv_[number_of_checked_parameters], "-uo")) {
          CommandLine::option_use_odometry = true;
        } else {
          CommandLine::filename_dataset = argv_[number_of_checked_parameters];
        }
        number_of_checked_parameters++;
      }

      //ds validate input parameters and exit on failure
      validateParameters();
    }

    //! @brief validates set parameters
    static void validateParameters() {

      //ds check camera topics
      if (CommandLine::topic_image_left.length() == 0) {
        std::cerr << "ERROR: empty value entered for parameter: -topic-image-left (-il) (enter -h for help)" << std::endl;
        exit(0);
      }
      if (CommandLine::topic_image_right.length() == 0) {
        std::cerr << "ERROR: empty value entered for parameter: -topic-image-right (-ir) (enter -h for help)" << std::endl;
        exit(0);
      }
    }

    //! @brief prints current command line parameter values
    static void printCommandLineParameters() {
      std::cerr << "-------------------------------------------------------------------------" << std::endl;
      std::cerr << "running with command line parameters:" << std::endl;
      if (CommandLine::filename_configuration.length() > 0) { std::cerr << "-config                  " << CommandLine::filename_configuration << std::endl;}
      std::cerr << "-topic-image-left        " << CommandLine::topic_image_left << std::endl;
      std::cerr << "-topic-image-right       " << CommandLine::topic_image_right << std::endl;
      if (CommandLine::topic_camera_info_left.length() > 0) { std::cerr << "-topic-camera-left-info  " << CommandLine::topic_camera_info_left << std::endl;}
      if (CommandLine::topic_camera_info_right.length() > 0) { std::cerr << "-topic-camera-right-info " << CommandLine::topic_camera_info_right << std::endl;}
      std::cerr << "-use-gui                 " << CommandLine::option_use_gui << std::endl;
      std::cerr << "-open                    " << !CommandLine::option_use_relocalization << std::endl;
      std::cerr << "-show-top                " << CommandLine::option_show_top_viewer << std::endl;
      std::cerr << "-use-odometry            " << CommandLine::option_use_odometry << std::endl;
      std::cerr << "-depth-mode              " << (CommandLine::tracker_mode == TrackerMode::Depth) << std::endl;
      std::cerr << "-drop-framepoints        " << CommandLine::option_drop_framepoints << std::endl;
      std::cerr << "-equalize-histogram      " << CommandLine::option_equalize_histogram << std::endl;
      std::cerr << "-rectify-and-undistort   " << CommandLine::option_rectify_and_undistort << std::endl;
      if (CommandLine::filename_dataset.length() > 0) { std::cerr << "-dataset                 " << CommandLine::filename_dataset << std::endl;}
      std::cerr << "-------------------------------------------------------------------------" << std::endl;
    }

    //! @brief prints help banner
    static void printBanner() {std::cerr << CommandLine::banner << std::endl;}

  //ds parameters
  public:

    //! @brief
    struct CommandLine {

      //! @brief informative
      static std::string banner;

      //! @brief files/topics
      static std::string topic_image_left;
      static std::string topic_image_right;
      static std::string topic_camera_info_left;
      static std::string topic_camera_info_right;
      static std::string filename_dataset;
      static std::string filename_configuration;

      //! @brief options
      static bool option_use_gui;
      static bool option_use_relocalization;
      static bool option_show_top_viewer;
      static bool option_drop_framepoints;
      static bool option_equalize_histogram;
      static bool option_rectify_and_undistort;
      static bool option_use_odometry;
      static TrackerMode tracker_mode;
    };

    //! @brief
    struct Types {

      //! @brief Frame: this criteria is used for the decision of creating a landmark or not from a track of framepoints
      static Count minimum_track_length_for_landmark_creation;

      //! @brief Landmark: minimum number of measurements before optimization is filtering
      static Count minimum_number_of_forced_updates;

      //! @brief Landmark: maximum allowed measurement divergence
      static real maximum_translation_error_to_depth_ratio;

      //! @brief LocalMap: target minimum number of landmarks for local map creation
      static Count minimum_number_of_landmarks;

      //! @brief WorldMap: key frame generation properties
      static real minimum_distance_traveled_for_local_map;
      static real minimum_degrees_rotated_for_local_map;
      static Count minimum_number_of_frames_for_local_map;
    };

    //! @brief
    struct FramepointGeneration {

      //! @brief dynamic thresholds for feature detection
      static real target_number_of_keypoints_tolerance;
      static int32_t detector_threshold;
      static int32_t detector_threshold_minimum;
      static real detector_threshold_step_size;

      //! @brief dynamic thresholds for descriptor matching
      static int32_t matching_distance_tracking_threshold;
      static int32_t matching_distance_tracking_threshold_maximum;
      static int32_t matching_distance_tracking_threshold_minimum;
      static int32_t matching_distance_tracking_step_size;

      //! @brief stereo: triangulation
      static int32_t maximum_matching_distance_triangulation;
      static real baseline_factor;
      static real minimum_disparity_pixels;
    };

    //! @brief
    struct MotionEstimation {

      //! @brief track lost criteria
      static Count minimum_number_of_landmarks_to_track;

      //! @brief point tracking thresholds
      static int32_t minimum_threshold_distance_tracking_pixels;
      static int32_t maximum_threshold_distance_tracking_pixels;

      //! @brief pixel search range width for point vicinity tracking
      static int32_t range_point_tracking;

      //! @brief maximum allowed pixel distance between image coordinates prediction and actual detection
      static int32_t maximum_distance_tracking_pixels;

      //! @brief framepoint track recovery
      static Count maximum_number_of_landmark_recoveries;

      //! @brief feature density regularization
      static Count bin_size_pixels;
      static real ratio_keypoints_to_bins;
    };

    //! @brief
    struct Relocalization {

      //! @brief minimum query interspace
      static Count preliminary_minimum_interspace_queries;

      //! @brief minimum relative number of matches
      static real preliminary_minimum_matching_ratio;

      //! @brief minimum absolute number of matches
      static Count minimum_number_of_matches_per_landmark;

      //! @brief correspondence retrieval
      static Count minimum_matches_per_correspondence;
    };

    //! @brief
    struct MapOptimization {

    };

    //! @brief
    struct Visualization {

    };
  };
}
