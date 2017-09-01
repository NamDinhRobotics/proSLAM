#pragma once
#include "definitions.h"

namespace proslam {

  //! @class parameter base class
  class Parameters {
  public:

    //! @brief constructor
    Parameters(const LoggingLevel& logging_level_): logging_level(logging_level_) {}

    //! @brief disable default construction
    Parameters() = delete;

    //! @brief destructor
    virtual ~Parameters() {}

    //! @brief parameter printing function
    virtual void print() const = 0;

    //! @brief log level (for all components)
    LoggingLevel logging_level;
  };

  //! @class command line parameters
  class CommandLineParameters: public Parameters {

  //ds exported types
  public:

    //! @brief SLAM system tracker modes
    enum TrackerMode {RGB_STEREO, //ds stereo image processing
                      RGB_DEPTH}; //ds rgb + depth image processing

  public:

    //! @brief constructor
    CommandLineParameters(const LoggingLevel& logging_level_): Parameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief tracker mode
    TrackerMode tracker_mode = RGB_STEREO;

    //! @brief files/topics
    std::string topic_image_left        = "/camera_left/image_raw";
    std::string topic_image_right       = "/camera_right/image_raw";
    std::string topic_camera_info_left  = "/camera_left/camera_info";
    std::string topic_camera_info_right = "/camera_right/camera_info";
    std::string dataset_file_name       = "";
    std::string configuration_file_name = "";

    //! @brief options
    bool option_use_gui                   = false;
    bool option_disable_relocalization    = false;
    bool option_show_top_viewer           = false;
    bool option_drop_framepoints          = false;
    bool option_equalize_histogram        = false;
    bool option_undistort_and_rectify     = false;
    bool option_use_odometry              = false;
    bool option_recover_landmarks         = false;
    bool option_disable_bundle_adjustment = false;
  };

  //! @class generic aligner parameters, present in modules with aligner units
  class AlignerParameters: public Parameters {
  public:

    //! @brief constructor
    AlignerParameters(const LoggingLevel& logging_level_): Parameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief minimum error delta for convergence
    real error_delta_for_convergence   = 1e-5;

    //! @brief maximum allowed robust kernel error
    real maximum_error_kernel          = 0.5;

    //! @brief system damping factor
    real damping                       = 1;

    //! @brief alignment iteration cap
    Count maximum_number_of_iterations = 100;

    //! @brief the minimum number of inliers required for a valid alignment
    Count minimum_number_of_inliers    = 25;

    //! @brief the minimum ratio of inliers to outliers required for a valid alignment
    real minimum_inlier_ratio          = 0.25;
  };

  //! @class landmark parameters
  class LandmarkParameters: public Parameters {
  public:

    //! @brief constructor
    LandmarkParameters(const LoggingLevel& logging_level_): Parameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief minimum number of measurements before optimization is filtering
    Count minimum_number_of_forced_updates        = 2;

    //! @brief maximum allowed measurement divergence
    real maximum_translation_error_to_depth_ratio = 1;

    //! @brief disable relocalization (prevents descriptor accumulation in landmark)
    bool option_disable_relocalization = false;
  };

  //! @class local map parameters
  class LocalMapParameters: public Parameters {
  public:

    //! @brief constructor
    LocalMapParameters(const LoggingLevel& logging_level_): Parameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief target minimum number of landmarks for local map creation
    Count minimum_number_of_landmarks = 50;
  };

  //! @class world map parameters
  class WorldMapParameters: public Parameters {
  public:

    //! @brief default constructor
    WorldMapParameters(const LoggingLevel& logging_level_): Parameters(logging_level_),
                                                            landmark(new LandmarkParameters(logging_level_)),
                                                            local_map(new LocalMapParameters(logging_level_)) {};

    //! @brief destructor: clean inner parameters
    ~WorldMapParameters() {delete landmark; delete local_map;}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief key frame generation properties
    real minimum_distance_traveled_for_local_map = 0.5;
    real minimum_degrees_rotated_for_local_map   = 0.5;
    Count minimum_number_of_frames_for_local_map = 4;

    //! @brief landmark generation parameters
    LandmarkParameters* landmark;

    //! @brief local map generation parameters
    LocalMapParameters* local_map;
  };

  //! @class framepoint generation parameters
  class BaseFramepointGeneratorParameters: public Parameters {
  public:

    //! @brief constructor
    BaseFramepointGeneratorParameters(const LoggingLevel& logging_level_): Parameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief dynamic thresholds for feature detection
    real target_number_of_keypoints_tolerance = 0.1;
    int32_t detector_threshold                = 15;
    int32_t detector_threshold_minimum        = 5;
    real detector_threshold_step_size         = 5;

    //! @brief dynamic thresholds for descriptor matching
    int32_t matching_distance_tracking_threshold         = 50;
    int32_t matching_distance_tracking_threshold_maximum = 50;
    int32_t matching_distance_tracking_threshold_minimum = 15;
    int32_t matching_distance_tracking_step_size         = 1;
  };

  //! @class framepoint generation parameters for a stereo camera setup
  class StereoFramePointGeneratorParameters: public BaseFramepointGeneratorParameters {
  public:

    //! @brief constructor
    StereoFramePointGeneratorParameters(const LoggingLevel& logging_level_): BaseFramepointGeneratorParameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief stereo: triangulation
    int32_t maximum_matching_distance_triangulation = 50;
    real baseline_factor                            = 50;
    real minimum_disparity_pixels                   = 1;
    uint32_t epipolar_line_thickness_pixels         = 0;
  };

  //! @class framepoint generation parameters for a rgbd camera setup
  class DepthFramePointGeneratorParameters: public BaseFramepointGeneratorParameters {
  public:

    //! @brief constructor
    DepthFramePointGeneratorParameters(const LoggingLevel& logging_level_): BaseFramepointGeneratorParameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief depth sensor configuration
    real maximum_depth_near_meters = 5;
    real maximum_depth_far_meters  = 20;
  };

  //! @class base tracker parameters
  class BaseTrackerParameters: public Parameters {
  protected:

    //! @brief default construction (only by subclasses)
    BaseTrackerParameters(const LoggingLevel& logging_level_);

    //! @brief destructor: clean inner parameters
    ~BaseTrackerParameters() {delete aligner;}

  public:

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief this criteria is used for the decision of whether creating a landmark or not from a track of framepoints
    Count minimum_track_length_for_landmark_creation = 3;

    //! @brief track lost criteria
    Count minimum_number_of_landmarks_to_track = 5;

    //! @brief point tracking thresholds
    int32_t minimum_threshold_distance_tracking_pixels = 4*4;
    int32_t maximum_threshold_distance_tracking_pixels = 7*7;

    //! @brief pixel search range width for point vicinity tracking
    int32_t range_point_tracking = 2;

    //! @brief maximum allowed pixel distance between image coordinates prediction and actual detection
    int32_t maximum_distance_tracking_pixels = 150*150;

    //! @brief framepoint track recovery
    bool enable_landmark_recovery               = false;
    Count maximum_number_of_landmark_recoveries = 3;

    //! @brief feature density regularization
    Count bin_size_pixels        = 16;
    real ratio_keypoints_to_bins = 1;

    //! @brief pose optimization
    real minimum_delta_angular_for_movement       = 0.001;
    real minimum_delta_translational_for_movement = 0.01;

    //! @brief parameters of aligner unit
    AlignerParameters* aligner;
  };

  //! @class stereo tracker parameters
  class StereoTrackerParameters: public BaseTrackerParameters {
  public:

    //! @brief constructor
    StereoTrackerParameters(const LoggingLevel& logging_level_): BaseTrackerParameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;
  };

  //! @class depth tracker parameters
  class DepthTrackerParameters: public BaseTrackerParameters {
  public:

    //! @brief constructor
    DepthTrackerParameters(const LoggingLevel& logging_level_): BaseTrackerParameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;
  };

  //! @class relocalization parameters
  class RelocalizerParameters: public Parameters {
  public:

    //! @brief default constructor
    RelocalizerParameters(const LoggingLevel& logging_level_): Parameters(logging_level_),
                                                               aligner(new AlignerParameters(logging_level_)) {}

    //! @brief destructor: clean inner parameters
    ~RelocalizerParameters() {delete aligner;}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief minimum query interspace
    Count preliminary_minimum_interspace_queries = 5;

    //! @brief minimum relative number of matches
    real preliminary_minimum_matching_ratio      = 0.1;

    //! @brief minimum absolute number of matches
    Count minimum_number_of_matches_per_landmark = 20;

    //! @brief correspondence retrieval
    Count minimum_matches_per_correspondence     = 0;

    //! @brief parameters of aligner unit
    AlignerParameters* aligner;
  };

  //! @class pose graph optimizer parameters
  class GraphOptimizerParameters: public Parameters {
  public:

    //! @brief default constructor
    GraphOptimizerParameters(const LoggingLevel& logging_level_): Parameters(logging_level_) {}

    //! @brief destructor: clean inner parameters
    ~GraphOptimizerParameters() {}

    //! @brief parameter printing function
    virtual void print() const;

    //! @brief g2o identifier space between frames and landmark vertices
    Count identifier_space = 1000000;

    //! @brief determines window size for bundle adjustment
    Count number_of_frames_per_bundle_adjustment = 100;

    //! @brief base frame weight in pose graph (assuming 1 for landmarks)
    real base_information_frame = 1e5;

    //! @brief enable robust kernel for landmark measurements
    bool enable_robust_kernel_for_landmark_measurements = false;
  };

  //! @class image viewer parameters
  class ImageViewerParameters: public Parameters {
  public:

    //! @brief default constructor
    ImageViewerParameters(const LoggingLevel& logging_level_): Parameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;
  };

  //! @class map viewer parameters
  class MapViewerParameters: public Parameters {
  public:

    //! @brief default constructor
    MapViewerParameters(const LoggingLevel& logging_level_): Parameters(logging_level_) {}

    //! @brief parameter printing function
    virtual void print() const;
  };

  //! @class object holding all parameters
  class ParameterCollection: public Parameters {

  //ds object management
  public:

    //! @brief default constructor
    //! allocates the minimal set of parameters
    //! specific parameter sets are allocated automatically after parsing the command line
    //! @param[in] logging_level_ desired logging level for contained parameters
    ParameterCollection(const LoggingLevel& logging_level_);

    //! @brief default destructor
    ~ParameterCollection();

  //ds functionality
  public:

    //! @brief utility parsing command line parameters - overwriting the configuration specified by file
    //! @param[in] argc_ main argument count
    //! @param[in] argv_ main argument values
    void parseFromCommandLine(const int32_t& argc_, char** argv_);

    //! @brief utility parsing parameters from a file (YAML)
    //! @param[in] filename_ target parameter YAML file
    void parseFromFile(const std::string& filename_);

    //! @brief validates certain parameters
    void validateParameters();

    //! @brief sets tracking mode related objects
    //! @param[in] mode_ desired tracking mode
    void setMode(const CommandLineParameters::TrackerMode& mode_);

    //! @brief triggers all inner print methods of set parameters
    virtual void print() const;

  //ds parameter bundles
  public:

    //! @brief program banner
    static std::string banner;

    //! @brief inner parameters (required for logging inside parameter collection)
    Parameters* _parameters = 0;

    CommandLineParameters* command_line_parameters                              = 0;
    WorldMapParameters* world_map_parameters                                    = 0;
    StereoFramePointGeneratorParameters* stereo_framepoint_generator_parameters = 0;
    DepthFramePointGeneratorParameters* depth_framepoint_generator_parameters   = 0;
    StereoTrackerParameters* stereo_tracker_parameters                          = 0;
    DepthTrackerParameters* depth_tracker_parameters                            = 0;
    RelocalizerParameters* relocalizer_parameters                               = 0;
    GraphOptimizerParameters* graph_optimizer_parameters                        = 0;

    ImageViewerParameters* image_viewer_parameters = 0;
    MapViewerParameters* map_viewer_parameters     = 0;

  //ds inner attributes
  protected:

    //! @informative, scanned parameter count in the file - unparsed
    Count number_of_parameters_detected;

    //! @informative, parsed and imported parameter count
    Count number_of_parameters_parsed;
  };
}
