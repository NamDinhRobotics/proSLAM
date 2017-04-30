#include "parameters.h"
#include "yaml-cpp/yaml.h"

namespace proslam {

  //ds statics
  Count Parameters::_number_of_instances = 0;
  std::string ParameterCollection::banner =
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

  //! @brief macro wrapping the YAML node parsing for a single parameter
  //! @param YAML_NODE the target root YAML node
  //! @param YAML_GROUP_NAME YAML parameter group name (e.g. command_line)
  //! @param STRUCT_NAME parameter Struct name (e.g. command_line_parameters)
  //! @param PARAMETER_NAME parameter name (e.g. topic_image_left)
  //! @param PARAMETER_TYPE parameter type (e.g. std::string)
  #define PARSE_PARAMETER(YAML_NODE, YAML_GROUP_NAME, STRUCT_NAME, PARAMETER_NAME, PARAMETER_TYPE) \
  try { \
    ++number_of_parameters_detected; \
    STRUCT_NAME->PARAMETER_NAME = YAML_NODE[#YAML_GROUP_NAME][#PARAMETER_NAME].as<PARAMETER_TYPE>(); \
    ++number_of_parameters_parsed; \
    /*std::cerr << "parameter name: '" << #STRUCT_NAME << ": " << #PARAMETER_NAME << "' value: '" << STRUCT_NAME->PARAMETER_NAME << "'" << std::endl;*/ \
  } catch (const YAML::TypedBadConversion<PARAMETER_TYPE>& exception_) { \
    std::cerr << "WARNING: unable to parse parameter: '" << #STRUCT_NAME << ": " << #PARAMETER_NAME << "' (exception: '" << exception_.what() << "')" << std::endl; \
  }

  //ds Command line
  void CommandLineParameters::print() const {
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
    std::cerr << "running with command line parameters:" << std::endl;
    if (filename_configuration.length() > 0) {std::cerr << "-configuration           " << filename_configuration << std::endl;}
    std::cerr << "-topic-image-left        " << topic_image_left << std::endl;
    std::cerr << "-topic-image-right       " << topic_image_right << std::endl;
    if (topic_camera_info_left.length() > 0) {std::cerr << "-topic-camera-left-info  " << topic_camera_info_left << std::endl;}
    if (topic_camera_info_right.length() > 0) {std::cerr << "-topic-camera-right-info " << topic_camera_info_right << std::endl;}
    std::cerr << "-use-gui                 " << option_use_gui << std::endl;
    std::cerr << "-open                    " << !option_use_relocalization << std::endl;
    std::cerr << "-show-top                " << option_show_top_viewer << std::endl;
    std::cerr << "-use-odometry            " << option_use_odometry << std::endl;
    std::cerr << "-depth-mode              " << (tracker_mode == TrackerMode::RGB_DEPTH) << std::endl;
    std::cerr << "-drop-framepoints        " << option_drop_framepoints << std::endl;
    std::cerr << "-equalize-histogram      " << option_equalize_histogram << std::endl;
    std::cerr << "-rectify-and-undistort   " << option_rectify_and_undistort << std::endl;
    if (filename_dataset.length() > 0) {std::cerr << "-dataset                 " << filename_dataset << std::endl;}
    std::cerr << "-------------------------------------------------------------------------" << std::endl;
  }



  //ds Types
  void FrameParameters::print() const {
    std::cerr << "FrameParameters::print|minimum_track_length_for_landmark_creation: " << minimum_track_length_for_landmark_creation << std::endl;
  }

  void LandmarkParameters::print() const {
    std::cerr << "LandmarkParameters::print|minimum_number_of_forced_updates: " << minimum_number_of_forced_updates << std::endl;
    std::cerr << "LandmarkParameters::print|maximum_translation_error_to_depth_ratio: " << maximum_translation_error_to_depth_ratio << std::endl;
  }

  void LocalMapParameters::print() const {
    std::cerr << "LocalMapParameters::print|minimum_number_of_landmarks: " << minimum_number_of_landmarks << std::endl;
  }

  void WorldMapParameters::print() const {
    std::cerr << "WorldMapParameters::print|minimum_distance_traveled_for_local_map: " << minimum_distance_traveled_for_local_map << std::endl;
    std::cerr << "WorldMapParameters::print|minimum_degrees_rotated_for_local_map: " << minimum_degrees_rotated_for_local_map << std::endl;
    std::cerr << "WorldMapParameters::print|minimum_number_of_frames_for_local_map: " << minimum_number_of_frames_for_local_map << std::endl;
  }



  //ds Framepoint estimation
  void BaseFramepointGeneratorParameters::print() const {
    std::cerr << "BaseFramepointGeneratorParameters::print|target_number_of_keypoints_tolerance: " << target_number_of_keypoints_tolerance << std::endl;
    std::cerr << "BaseFramepointGeneratorParameters::print|detector_threshold: " << detector_threshold << std::endl;
    std::cerr << "BaseFramepointGeneratorParameters::print|detector_threshold_minimum: " << detector_threshold_minimum << std::endl;
    std::cerr << "BaseFramepointGeneratorParameters::print|detector_threshold_step_size: " << detector_threshold_step_size << std::endl;
    std::cerr << "BaseFramepointGeneratorParameters::print|matching_distance_tracking_threshold: " << matching_distance_tracking_threshold << std::endl;
    std::cerr << "BaseFramepointGeneratorParameters::print|matching_distance_tracking_threshold_maximum: " << matching_distance_tracking_threshold_maximum << std::endl;
    std::cerr << "BaseFramepointGeneratorParameters::print|matching_distance_tracking_threshold_minimum: " << matching_distance_tracking_threshold_minimum << std::endl;
    std::cerr << "BaseFramepointGeneratorParameters::print|matching_distance_tracking_step_size: " << matching_distance_tracking_step_size << std::endl;
  }

  void StereoFramePointGeneratorParameters::print() const {
    std::cerr << "StereoFramepointGeneratorParameters::print|maximum_matching_distance_triangulation: " << maximum_matching_distance_triangulation << std::endl;
    std::cerr << "StereoFramepointGeneratorParameters::print|baseline_factor: " << baseline_factor << std::endl;
    std::cerr << "StereoFramepointGeneratorParameters::print|minimum_disparity_pixels: " << minimum_disparity_pixels << std::endl;
    BaseFramepointGeneratorParameters::print();
  }

  void DepthFramePointGeneratorParameters::print() const {
    BaseFramepointGeneratorParameters::print();
  }



  //ds Motion estimation
  void BaseTrackerParameters::print() const {
    std::cerr << "BaseTrackerParameters::print|minimum_number_of_landmarks_to_track: " << minimum_number_of_landmarks_to_track << std::endl;
    std::cerr << "BaseTrackerParameters::print|minimum_threshold_distance_tracking_pixels: " << minimum_threshold_distance_tracking_pixels << std::endl;
    std::cerr << "BaseTrackerParameters::print|maximum_threshold_distance_tracking_pixels: " << maximum_threshold_distance_tracking_pixels << std::endl;
    std::cerr << "BaseTrackerParameters::print|range_point_tracking: " << range_point_tracking << std::endl;
    std::cerr << "BaseTrackerParameters::print|maximum_distance_tracking_pixels: " << maximum_distance_tracking_pixels << std::endl;
    std::cerr << "BaseTrackerParameters::print|maximum_number_of_landmark_recoveries: " << maximum_number_of_landmark_recoveries << std::endl;
    std::cerr << "BaseTrackerParameters::print|bin_size_pixels: " << bin_size_pixels << std::endl;
    std::cerr << "BaseTrackerParameters::print|ratio_keypoints_to_bins: " << ratio_keypoints_to_bins << std::endl;
  }

  void StereoTrackerParameters::print() const {
    BaseTrackerParameters::print();
  }

  void DepthTrackerParameters::print() const {
    BaseTrackerParameters::print();
  }



  //ds Relocalization
  void RelocalizerParameters::print() const {
    std::cerr << "RelocalizerParameters::print|preliminary_minimum_interspace_queries: " << preliminary_minimum_interspace_queries << std::endl;
    std::cerr << "RelocalizerParameters::print|preliminary_minimum_matching_ratio: " << preliminary_minimum_matching_ratio << std::endl;
    std::cerr << "RelocalizerParameters::print|minimum_number_of_matches_per_landmark: " << minimum_number_of_matches_per_landmark << std::endl;
    std::cerr << "RelocalizerParameters::print|minimum_matches_per_correspondence: " << minimum_matches_per_correspondence << std::endl;
  }



  ParameterCollection::ParameterCollection(): number_of_parameters_detected(0), number_of_parameters_parsed(0) {
    std::cerr << "ParameterCollection::ParameterCollection|constructing" << std::endl;
    command_line_parameters = new CommandLineParameters();
    frame_parameters        = new FrameParameters();
    landmark_parameters     = new LandmarkParameters();
    local_map_parameters    = new LocalMapParameters();
    world_map_parameters    = new WorldMapParameters();
    relocalizer_parameters  = new RelocalizerParameters();
    std::cerr << "ParameterCollection::ParameterCollection|constructed" << std::endl;
  }

  void ParameterCollection::destroy() {
    std::cerr << "ParameterCollection::destroy|destroying" << std::endl;
    if (command_line_parameters) {delete command_line_parameters;}
    if (frame_parameters) {delete frame_parameters;}
    if (landmark_parameters) {delete landmark_parameters;}
    if (world_map_parameters) {delete local_map_parameters;}
    if (world_map_parameters) {delete world_map_parameters;}
    if (stereo_framepoint_generator_parameters) {delete stereo_framepoint_generator_parameters;}
    if (depth_framepoint_generator_parameters) {delete depth_framepoint_generator_parameters;}
    if (stereo_tracker_parameters) {delete stereo_tracker_parameters;}
    if (depth_tracker_parameters) {delete depth_tracker_parameters;}
    if (relocalizer_parameters) {delete relocalizer_parameters;}
    std::cerr << "ParameterCollection::destroy|destroyed" << std::endl;
  }

  ParameterCollection::~ParameterCollection() {
    std::cerr << "ParameterCollection::ParameterCollection|destroying" << std::endl;
    destroy();
    std::cerr << "ParameterCollection::ParameterCollection|destroyed" << std::endl;
  }

  void ParameterCollection::parseFromCommandLine(const int32_t& argc_, char ** argv_) {

    //ds skim the command line for configuration file input and tracker mode
    int32_t number_of_checked_parameters = 1;
    while(number_of_checked_parameters < argc_){
      if (!std::strcmp(argv_[number_of_checked_parameters], "-configuration") || !std::strcmp(argv_[number_of_checked_parameters], "-c")){
        number_of_checked_parameters++;
        if (number_of_checked_parameters == argc_) {break;}
        command_line_parameters->filename_configuration = argv_[number_of_checked_parameters];
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-depth-mode") || !std::strcmp(argv_[number_of_checked_parameters], "-dm")) {
        command_line_parameters->tracker_mode = CommandLineParameters::TrackerMode::RGB_DEPTH;
      }
      number_of_checked_parameters++;
    }

    //ds if no configuration file was specified
    if (command_line_parameters->filename_configuration.length() == 0) {

      //ds check if a default configuration file is present
      if (srrg_core::isAccessible("configuration.yaml")) {
        command_line_parameters->filename_configuration = "configuration.yaml";
        std::cerr << "ParameterCollection::parseParametersFromCommandLine|loading default configuration file: " << command_line_parameters->filename_configuration << std::endl;
      } else {
        std::cerr << "ParameterCollection::parseParametersFromCommandLine|WARNING: no configuration file found (running with internal settings)" << std::endl;
      }
    } else {

      //ds check if specified configuration file is not accessible
      if (!srrg_core::isAccessible(command_line_parameters->filename_configuration)) {
        std::cerr << "ParameterCollection::parseParametersFromCommandLine|ERROR: specified configuration file is not accessible: " << command_line_parameters->filename_configuration << std::endl;
        destroy();
        exit(0);
      }
    }

    //ds generate tracker mode specific parameters
    switch (command_line_parameters->tracker_mode) {
      case CommandLineParameters::TrackerMode::RGB_STEREO: {
        stereo_framepoint_generator_parameters = new StereoFramePointGeneratorParameters();
        stereo_tracker_parameters = new StereoTrackerParameters();
        break;
      }
      case CommandLineParameters::TrackerMode::RGB_DEPTH: {
        depth_framepoint_generator_parameters = new DepthFramePointGeneratorParameters();
        depth_tracker_parameters = new DepthTrackerParameters();
        break;
      }
      default: {
        std::cerr << "ParameterCollection::parseFromCommandLine|ERROR: invalid tracker mode" << std::endl;
        destroy();
        exit(0);
      }
    }

    //ds if a valid configuration file is set (otherwise the field must be empty)
    if (command_line_parameters->filename_configuration.length()) {

      //ds parse parameters from configuration file
      parseFromFile(command_line_parameters->filename_configuration);
    }

    //ds reset and check for other command line parameters, potentially overwritting the ones set in the configuration file
    number_of_checked_parameters = 1;
    while(number_of_checked_parameters < argc_){
      if (!std::strcmp(argv_[number_of_checked_parameters], "-topic-image-left") || !std::strcmp(argv_[number_of_checked_parameters], "-il")){
        number_of_checked_parameters++;
        if (number_of_checked_parameters == argc_) {break;}
        command_line_parameters->topic_image_left = argv_[number_of_checked_parameters];
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-topic-image-right") || !std::strcmp(argv_[number_of_checked_parameters], "-ir")){
        number_of_checked_parameters++;
        if (number_of_checked_parameters == argc_) {break;}
        command_line_parameters->topic_image_right = argv_[number_of_checked_parameters];
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-topic-camera-info-left") || !std::strcmp(argv_[number_of_checked_parameters], "-cl")){
        number_of_checked_parameters++;
        if (number_of_checked_parameters == argc_) {break;}
        command_line_parameters->topic_camera_info_left = argv_[number_of_checked_parameters];
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-topic-camera-info-right") || !std::strcmp(argv_[number_of_checked_parameters], "-cr")){
        number_of_checked_parameters++;
        if (number_of_checked_parameters == argc_) {break;}
        command_line_parameters->topic_camera_info_right = argv_[number_of_checked_parameters];
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-h") || !std::strcmp(argv_[number_of_checked_parameters], "--h")) {
        std::cerr << banner << std::endl;
        destroy();
        exit(0);
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-help") || !std::strcmp(argv_[number_of_checked_parameters], "--help")) {
        std::cerr << banner << std::endl;
        destroy();
        exit(0);
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-use-gui") || !std::strcmp(argv_[number_of_checked_parameters], "-ug")) {
        command_line_parameters->option_use_gui = true;
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-open")) {
        command_line_parameters->option_use_relocalization = false;
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-show-top") || !std::strcmp(argv_[number_of_checked_parameters], "-st")) {
        command_line_parameters->option_show_top_viewer = true;
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-drop-framepoints") || !std::strcmp(argv_[number_of_checked_parameters], "-df")) {
        command_line_parameters->option_drop_framepoints = true;
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-equalize-histogram") || !std::strcmp(argv_[number_of_checked_parameters], "-eh")) {
        command_line_parameters->option_equalize_histogram = true;
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-rectify-and-undistort") || !std::strcmp(argv_[number_of_checked_parameters], "-ru")) {
        command_line_parameters->option_rectify_and_undistort = true;
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-depth-mode") || !std::strcmp(argv_[number_of_checked_parameters], "-dm")) {
        command_line_parameters->tracker_mode = CommandLineParameters::TrackerMode::RGB_DEPTH;
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-use-odometry") || !std::strcmp(argv_[number_of_checked_parameters], "-uo")) {
        command_line_parameters->option_use_odometry = true;
      } else if (!std::strcmp(argv_[number_of_checked_parameters], "-configuration") || !std::strcmp(argv_[number_of_checked_parameters], "-c")){
        number_of_checked_parameters++;
      } else {
        if (command_line_parameters->filename_dataset.length() == 0) {command_line_parameters->filename_dataset = argv_[number_of_checked_parameters];}
      }
      number_of_checked_parameters++;
    }

    //ds validate input parameters and exit on failure
    validateParameters();
  }

  void ParameterCollection::parseFromFile(const std::string& filename_) {
    try {

      //ds attempt to open the configuration file and parse it into a YAML node
      YAML::Node configuration = YAML::LoadFile(filename_);

      //CommandLine
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, topic_image_left, std::string)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, topic_image_right, std::string)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, topic_camera_info_left, std::string)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, topic_camera_info_right, std::string)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, filename_dataset, std::string)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, option_use_gui, bool)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, option_use_odometry, bool)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, option_use_relocalization, bool)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, option_show_top_viewer, bool)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, option_drop_framepoints, bool)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, option_equalize_histogram, bool)
      PARSE_PARAMETER(configuration, command_line, command_line_parameters, option_rectify_and_undistort, bool)

      //Types
      PARSE_PARAMETER(configuration, frame, frame_parameters, minimum_track_length_for_landmark_creation, Count)
      PARSE_PARAMETER(configuration, landmark, landmark_parameters, minimum_number_of_forced_updates, Count)
      PARSE_PARAMETER(configuration, landmark, landmark_parameters, maximum_translation_error_to_depth_ratio, real)
      PARSE_PARAMETER(configuration, local_map, local_map_parameters, minimum_number_of_landmarks, Count)
      PARSE_PARAMETER(configuration, world_map, world_map_parameters, minimum_distance_traveled_for_local_map, real)
      PARSE_PARAMETER(configuration, world_map, world_map_parameters, minimum_degrees_rotated_for_local_map, real)
      PARSE_PARAMETER(configuration, world_map, world_map_parameters, minimum_number_of_frames_for_local_map, Count)

      //ds mode specific parameters
      switch (command_line_parameters->tracker_mode) {
        case CommandLineParameters::TrackerMode::RGB_STEREO: {

          //FramepointGeneration
          PARSE_PARAMETER(configuration, base_framepoint_generation, stereo_framepoint_generator_parameters, target_number_of_keypoints_tolerance, real)
          PARSE_PARAMETER(configuration, base_framepoint_generation, stereo_framepoint_generator_parameters, detector_threshold, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, stereo_framepoint_generator_parameters, detector_threshold_minimum, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, stereo_framepoint_generator_parameters, detector_threshold_step_size, real)
          PARSE_PARAMETER(configuration, base_framepoint_generation, stereo_framepoint_generator_parameters, matching_distance_tracking_threshold, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, stereo_framepoint_generator_parameters, matching_distance_tracking_threshold_maximum, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, stereo_framepoint_generator_parameters, matching_distance_tracking_threshold_minimum, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, stereo_framepoint_generator_parameters, matching_distance_tracking_step_size, int32_t)
          PARSE_PARAMETER(configuration, stereo_framepoint_generation, stereo_framepoint_generator_parameters, maximum_matching_distance_triangulation, int32_t)
          PARSE_PARAMETER(configuration, stereo_framepoint_generation, stereo_framepoint_generator_parameters, baseline_factor, real)
          PARSE_PARAMETER(configuration, stereo_framepoint_generation, stereo_framepoint_generator_parameters, minimum_disparity_pixels, real)

          //MotionEstimation
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, minimum_number_of_landmarks_to_track, Count)
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, maximum_threshold_distance_tracking_pixels, int32_t)
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, minimum_threshold_distance_tracking_pixels, int32_t)
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, range_point_tracking, int32_t)
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, maximum_distance_tracking_pixels, int32_t)
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, maximum_number_of_landmark_recoveries, Count)
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, bin_size_pixels, Count)
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, ratio_keypoints_to_bins, real)
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, minimum_delta_angular_for_movement, real)
          PARSE_PARAMETER(configuration, base_tracking, stereo_tracker_parameters, minimum_delta_translational_for_movement, real)
          break;
        }
        case CommandLineParameters::TrackerMode::RGB_DEPTH: {

          //FramepointGeneration
          PARSE_PARAMETER(configuration, base_framepoint_generation, depth_framepoint_generator_parameters, target_number_of_keypoints_tolerance, real)
          PARSE_PARAMETER(configuration, base_framepoint_generation, depth_framepoint_generator_parameters, detector_threshold, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, depth_framepoint_generator_parameters, detector_threshold_minimum, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, depth_framepoint_generator_parameters, detector_threshold_step_size, real)
          PARSE_PARAMETER(configuration, base_framepoint_generation, depth_framepoint_generator_parameters, matching_distance_tracking_threshold, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, depth_framepoint_generator_parameters, matching_distance_tracking_threshold_maximum, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, depth_framepoint_generator_parameters, matching_distance_tracking_threshold_minimum, int32_t)
          PARSE_PARAMETER(configuration, base_framepoint_generation, depth_framepoint_generator_parameters, matching_distance_tracking_step_size, int32_t)
          PARSE_PARAMETER(configuration, depth_framepoint_generation, depth_framepoint_generator_parameters, maximum_depth_near_meters, real)
          PARSE_PARAMETER(configuration, depth_framepoint_generation, depth_framepoint_generator_parameters, maximum_depth_far_meters, real)

          //MotionEstimation
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, minimum_number_of_landmarks_to_track, Count)
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, maximum_threshold_distance_tracking_pixels, int32_t)
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, minimum_threshold_distance_tracking_pixels, int32_t)
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, range_point_tracking, int32_t)
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, maximum_distance_tracking_pixels, int32_t)
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, maximum_number_of_landmark_recoveries, Count)
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, bin_size_pixels, Count)
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, ratio_keypoints_to_bins, real)
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, minimum_delta_angular_for_movement, real)
          PARSE_PARAMETER(configuration, base_tracking, depth_tracker_parameters, minimum_delta_translational_for_movement, real)
          break;
        }
        default: {
          std::cerr << "ParameterCollection::parseParametersFromFile|ERROR: invalid tracker mode" << std::endl;
          destroy();
          exit(0);
        }
      }

      //Relocalization
      PARSE_PARAMETER(configuration, relocalization, relocalizer_parameters, preliminary_minimum_interspace_queries, Count)
      PARSE_PARAMETER(configuration, relocalization, relocalizer_parameters, preliminary_minimum_matching_ratio, real)
      PARSE_PARAMETER(configuration, relocalization, relocalizer_parameters, minimum_number_of_matches_per_landmark, Count)
      PARSE_PARAMETER(configuration, relocalization, relocalizer_parameters, minimum_matches_per_correspondence, Count)

      //ds done
      std::cerr << "ParameterCollection::parseParametersFromFile|successfully loaded configuration from file: " << filename_ << std::endl;
      std::cerr << "ParameterCollection::parseParametersFromFile|number of imported parameters: " << number_of_parameters_parsed << "/" << number_of_parameters_detected << std::endl;
    } catch (const YAML::BadFile& exception_) {
      std::cerr << "ParameterCollection::parseParametersFromFile|ERROR: unable to parse configuration file: " << filename_ << " - exception: '" << exception_.what() << "'" << std::endl;
    }
  }

  void ParameterCollection::validateParameters() {

    //ds check camera topics
    if (command_line_parameters->topic_image_left.length() == 0) {
      std::cerr << "ParameterCollection::validateParameters|ERROR: empty value entered for parameter: -topic-image-left (-il) (enter -h for help)" << std::endl;
      destroy();
      exit(0);
    }
    if (command_line_parameters->topic_image_right.length() == 0) {
      std::cerr << "ParameterCollection::validateParameters|ERROR: empty value entered for parameter: -topic-image-right (-ir) (enter -h for help)" << std::endl;
      destroy();
      exit(0);
    }
  }
}
