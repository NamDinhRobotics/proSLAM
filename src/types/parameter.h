#pragma once
#include <fstream>
#include "definitions.h"

namespace proslam {

  //ds this class serves as a singleton parameter base for the ProSLAM pipeline
  class Parameter {

    #define PARSE_PARAMETER(STRING_TO_PARSE, GROUPNAME, PARAMETERNAME) \
    try { \
      std::string search_term_##GROUPNAME_##PARAMETERNAME = " "; \
      search_term_##GROUPNAME_##PARAMETERNAME += #PARAMETERNAME; \
      search_term_##GROUPNAME_##PARAMETERNAME += ":"; \
      const Index index_parameter_name_begin_##GROUPNAME_##PARAMETERNAME  = STRING_TO_PARSE.find(search_term_##GROUPNAME_##PARAMETERNAME); \
      const Index index_parameter_name_end_##GROUPNAME_##PARAMETERNAME    = index_parameter_name_begin_##GROUPNAME_##PARAMETERNAME+search_term_##GROUPNAME_##PARAMETERNAME.length(); \
      const Index index_parameter_value_begin_##GROUPNAME_##PARAMETERNAME = STRING_TO_PARSE.find_first_not_of("\t #", index_parameter_name_end_##GROUPNAME_##PARAMETERNAME); \
      const Index index_parameter_value_end_##GROUPNAME_##PARAMETERNAME   = STRING_TO_PARSE.find_first_of("\t\n #", index_parameter_value_begin_##GROUPNAME_##PARAMETERNAME); \
      const std::string parameter_value_##GROUPNAME_##PARAMETERNAME = STRING_TO_PARSE.substr(index_parameter_value_begin_##GROUPNAME_##PARAMETERNAME, \
                                                                                             index_parameter_value_end_##GROUPNAME_##PARAMETERNAME- \
                                                                                             index_parameter_value_begin_##GROUPNAME_##PARAMETERNAME); \
      setParameterValue(GROUPNAME::PARAMETERNAME, parameter_value_##GROUPNAME_##PARAMETERNAME); \
      /*std::cerr << "parameter name: " << #GROUPNAME << "::" << #PARAMETERNAME << " value: " << GROUPNAME::PARAMETERNAME << std::endl; */ \
    } catch (const std::out_of_range& exception_) { \
      std::cerr << "unable to parse parameter: " << #PARAMETERNAME << " - exception: " << exception_.what() << std::endl; \
    } catch (const std::invalid_argument& exception_) { \
      std::cerr << "unable to parse parameter: " << #PARAMETERNAME << " - exception: " << exception_.what() << std::endl; \
    }

  //ds exported types
  public:

    //ds SLAM system tracker modes
    enum TrackerMode {Stereo, //ds stereo image processing
                      Depth}; //ds rgb + depth image processing

  //ds functionality
  public:

    static void parseParametersFromFile(const std::string& filename_) {

      //ds attempt to open the file (read-only)
      std::ifstream configuration_file(filename_);

      //ds check for failure
      if (!configuration_file.good() || !configuration_file.is_open()) {
        std::cerr << "Parameter::parseParametersFromFile|ERROR: unable to open configuration file: " << filename_ << std::endl;
        exit(0);
      }

      //ds get the complete file content into a standard string - then start parsing
      std::string configuration_string((std::istreambuf_iterator<char>(configuration_file)), std::istreambuf_iterator<char>());

//      //CommandLine
//      PARSE_PARAMETER(configuration_string, Types, minimum_track_length_for_landmark_creation)
//      PARSE_PARAMETER(configuration_string, Types, minimum_number_of_forced_updates)
//      PARSE_PARAMETER(configuration_string, Types, maximum_translation_error_to_depth_ratio)
//      PARSE_PARAMETER(configuration_string, Types, minimum_number_of_landmarks)
//      PARSE_PARAMETER(configuration_string, Types, minimum_distance_traveled_for_local_map)
//      PARSE_PARAMETER(configuration_string, Types, minimum_degrees_rotated_for_local_map)
//      PARSE_PARAMETER(configuration_string, Types, minimum_number_of_frames_for_local_map)

      //Types
      PARSE_PARAMETER(configuration_string, Types, minimum_track_length_for_landmark_creation)
      PARSE_PARAMETER(configuration_string, Types, minimum_number_of_forced_updates)
      PARSE_PARAMETER(configuration_string, Types, maximum_translation_error_to_depth_ratio)
      PARSE_PARAMETER(configuration_string, Types, minimum_number_of_landmarks)
      PARSE_PARAMETER(configuration_string, Types, minimum_distance_traveled_for_local_map)
      PARSE_PARAMETER(configuration_string, Types, minimum_degrees_rotated_for_local_map)
      PARSE_PARAMETER(configuration_string, Types, minimum_number_of_frames_for_local_map)

      //FramepointGeneration
      PARSE_PARAMETER(configuration_string, FramepointGeneration, target_number_of_keypoints_tolerance)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, detector_threshold)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, detector_threshold_minimum)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, detector_threshold_step_size)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, matching_distance_tracking_threshold)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, matching_distance_tracking_threshold_maximum)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, matching_distance_tracking_threshold_minimum)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, matching_distance_tracking_step_size)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, maximum_matching_distance_triangulation)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, baseline_factor)
      PARSE_PARAMETER(configuration_string, FramepointGeneration, minimum_disparity_pixels)

      //MotionEstimation
      PARSE_PARAMETER(configuration_string, MotionEstimation, minimum_number_of_landmarks_to_track)
      PARSE_PARAMETER(configuration_string, MotionEstimation, maximum_threshold_distance_tracking_pixels)
      PARSE_PARAMETER(configuration_string, MotionEstimation, minimum_threshold_distance_tracking_pixels)
      PARSE_PARAMETER(configuration_string, MotionEstimation, range_point_tracking)
      PARSE_PARAMETER(configuration_string, MotionEstimation, maximum_distance_tracking_pixels)
      PARSE_PARAMETER(configuration_string, MotionEstimation, maximum_number_of_landmark_recoveries)
      PARSE_PARAMETER(configuration_string, MotionEstimation, bin_size_pixels)
      PARSE_PARAMETER(configuration_string, MotionEstimation, ratio_keypoints_to_bins)

      //Relocalization
      PARSE_PARAMETER(configuration_string, Relocalization, preliminary_minimum_interspace_queries)
      PARSE_PARAMETER(configuration_string, Relocalization, preliminary_minimum_matching_ratio)
      PARSE_PARAMETER(configuration_string, Relocalization, minimum_number_of_matches_per_landmark)
      PARSE_PARAMETER(configuration_string, Relocalization, minimum_matches_per_correspondence)

      //ds done
      std::cerr << "Parameter::parseParametersFromFile|successfully loaded configuration from file: " << filename_ << std::endl;
    }

    static void parseParametersFromCommandLine(int32_t argc, char ** argv) {
      int32_t number_of_added_parameters = 1;
      while(number_of_added_parameters < argc){
        if (!std::strcmp(argv[number_of_added_parameters], "-topic-image-left") || !std::strcmp(argv[number_of_added_parameters], "-il")){
          number_of_added_parameters++;
          if (number_of_added_parameters == argc) {break;}
          topic_image_left = argv[number_of_added_parameters];
        } else if (!std::strcmp(argv[number_of_added_parameters], "-topic-image-right") || !std::strcmp(argv[number_of_added_parameters], "-ir")){
          number_of_added_parameters++;
          if (number_of_added_parameters == argc) {break;}
          topic_image_right = argv[number_of_added_parameters];
        } else if (!std::strcmp(argv[number_of_added_parameters], "-topic-camera-info-left") || !std::strcmp(argv[number_of_added_parameters], "-cl")){
          number_of_added_parameters++;
          if (number_of_added_parameters == argc) {break;}
          topic_camera_info_left = argv[number_of_added_parameters];
        } else if (!std::strcmp(argv[number_of_added_parameters], "-topic-camera-info-right") || !std::strcmp(argv[number_of_added_parameters], "-cr")){
          number_of_added_parameters++;
          if (number_of_added_parameters == argc) {break;}
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
        } else if (!std::strcmp(argv[number_of_added_parameters], "-configuration") || !std::strcmp(argv[number_of_added_parameters], "-c")){
          number_of_added_parameters++;
          if (number_of_added_parameters == argc) {break;}
          filename_configuration = argv[number_of_added_parameters];
        } else {
          filename_dataset = argv[number_of_added_parameters];
        }
        number_of_added_parameters++;
      }

      //ds if no configuration file was specified
      if (filename_configuration.length() == 0) {

        //ds check if a default configuration file is present
        if (srrg_core::isAccessible("configuration.yaml")) {
          filename_configuration = "configuration.yaml";
          std::cerr << "Parameter::parseParametersFromCommandLine|loading default configuration file: " << filename_configuration << std::endl;
        } else {
          std::cerr << "Parameter::parseParametersFromCommandLine|WARNING: no configuration file specified and no default configuration present (running with internal configuration)" << std::endl;
        }
      } else {

        //ds check if specified configuration file is not accessible
        if (!srrg_core::isAccessible(filename_configuration)) {
          std::cerr << "Parameter::parseParametersFromCommandLine|ERROR: specified configuration file is not accessible: " << filename_configuration << std::endl;
          exit(0);
        }
      }

      //ds if a valid configuration file is set (otherwise the field must be empty)
      if (filename_configuration.length()) {

        //ds parse parameters from configuration file
        parseParametersFromFile(filename_configuration);
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
      if (filename_configuration.length() > 0) { std::cerr << "-config                  " << filename_configuration << std::endl;}
      std::cerr << "-topic-image-left        " << topic_image_left << std::endl;
      std::cerr << "-topic-image-right       " << topic_image_right << std::endl;
      if (topic_camera_info_left.length() > 0) { std::cerr << "-topic-camera-left-info  " << topic_camera_info_left << std::endl;}
      if (topic_camera_info_right.length() > 0) { std::cerr << "-topic-camera-right-info " << topic_camera_info_right << std::endl;}
      std::cerr << "-use-gui                 " << option_use_gui << std::endl;
      std::cerr << "-open                    " << !option_use_relocalization << std::endl;
      std::cerr << "-show-top                " << option_show_top_viewer << std::endl;
      std::cerr << "-use-odometry            " << option_use_odometry << std::endl;
      std::cerr << "-depth-mode              " << (tracker_mode==TrackerMode::Depth) << std::endl;
      std::cerr << "-drop-framepoints        " << option_drop_framepoints << std::endl;
      std::cerr << "-equalize-histogram      " << option_equalize_histogram << std::endl;
      std::cerr << "-rectify-and-undistort   " << option_rectify_and_undistort << std::endl;
      if (filename_dataset.length() > 0) { std::cerr << "-dataset                 " << filename_dataset << std::endl;}
      std::cerr << "-------------------------------------------------------------------------" << std::endl;
    }

    static void printBanner() {std::cerr << banner << std::endl;}

    //! @brief converter function: string to regular integer
    static void setParameterValue(int32_t& parameter_, const std::string& parameter_value_) {parameter_ = std::stoi(parameter_value_);}

    //! @brief converter function: string to long integer
    static void setParameterValue(int64_t& parameter_, const std::string& parameter_value_) {parameter_ = std::stol(parameter_value_);}

    //! @brief converter function: string to unsigned integer
    static void setParameterValue(uint32_t& parameter_, const std::string& parameter_value_) {parameter_ = std::stoul(parameter_value_);}

    //! @brief converter function: string to large unsigned integer
    static void setParameterValue(uint64_t& parameter_, const std::string& parameter_value_) {parameter_ = std::stoull(parameter_value_);}

    //! @brief converter function: string to float
    static void setParameterValue(float& parameter_, const std::string& parameter_value_) {parameter_ = std::stof(parameter_value_);}

    //! @brief converter function: string to double
    static void setParameterValue(double& parameter_, const std::string& parameter_value_) {parameter_ = std::stod(parameter_value_);}

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
    static std::string filename_configuration;

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
      static int32_t minimum_threshold_distance_tracking_pixels;
      static int32_t maximum_threshold_distance_tracking_pixels;
      static int32_t range_point_tracking;             //ds pixel search range width for point vicinity tracking
      static int32_t maximum_distance_tracking_pixels; //ds maximum allowed pixel distance between image coordinates prediction and actual detection

      //ds framepoint track recovery
      static int32_t maximum_number_of_landmark_recoveries;

      //ds feature density regularization
      static Count bin_size_pixels;
      static real ratio_keypoints_to_bins;
    };

    struct Relocalization {

      //ds minimum query interspace
      static Count preliminary_minimum_interspace_queries;

      //ds minimum relative number of matches
      static real preliminary_minimum_matching_ratio;

      //ds minimum absolute number of matches
      static Count minimum_number_of_matches_per_landmark;

      //ds correspondence retrieval
      static Count minimum_matches_per_correspondence;
    };

    struct MapOptimization {

    };

    struct Visualization {

    };
  };
}
