#include "qapplication.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"

#include "map_optimization/graph_optimizer.h"
#include "relocalization/relocalizer.h"
#include "motion_estimation/base_tracker.h"
#include "visualization/viewer_input_images.h"
#include "visualization/viewer_output_map.h"

namespace proslam {

  //ds simple assembly of the different SLAM modules provided by ProSLAM
  class SLAMAssembly {

  //ds object management
  public:

    //ds default constructor - already allocating required default objects
    SLAMAssembly();

    //ds default destructor
    ~SLAMAssembly();

  //ds functionality
  public:

    //ds initializes txt_io playback modules
    void initializeMessageFile();

    //ds attempts to load the camera configuration based on the current input setting
    void loadCameras();

    //ds attempts to load the camera configuration based on the current input setting
    void loadCameras(const Camera* camera_left_, const Camera* camera_right_);

    //ds initializes gui components
    void initializeGUI(QApplication* ui_server_);

    //ds updated gui components
    void updateGUI();

    //ds clean gui components
    int32_t closeGUI(const bool& let_user_close_ = true);

    //ds playback txt_io message file
    void playbackMessageFile();

    //ds sets ground truth to current frame
    void addGroundTruthMeasurement(const TransformMatrix3D& robot_to_world_ground_truth_);

    //ds process a pair of rectified and undistorted stereo images
    void process(const cv::Mat& intensity_image_left_, const cv::Mat& intensity_image_right_);

    //ds prints extensive run summary
    void printReport();

  //ds getters/setters
  public:

    WorldMap* worldMap() {return _world_map;}
    GraphOptimizer* optimizer() {return _optimizer;}
    Relocalizer* relocalizer() {return _relocalizer;}
    BaseTracker* tracker() {return _tracker;}
    ViewerInputImages* viewerInputImages() {return _viewer_input_images;}
    const bool& isGUIRunning() const {return _is_gui_running;}

  //ds SLAM modules
  protected:

    BaseTracker* _makeStereoTracker(const Camera* camera_left,
				      const Camera* camera_right);

    BaseTracker* _makeDepthTracker(const Camera* camera_left,
				   const Camera* camera_right);

    //ds the SLAM map, containing landmarks and trajectory
    WorldMap* _world_map;

    //ds pose graph optimization handler
    GraphOptimizer* _optimizer;

    //ds relocalization module
    Relocalizer* _relocalizer;

    //ds tracking component, deriving the robots odometry
    BaseTracker* _tracker;

  //ds visualization only
  protected:

    //ds Qt UI server
    QApplication* _ui_server = 0;

    //ds raw image input processing viewer
    ViewerInputImages* _viewer_input_images = 0;

    //ds 3D output viewers
    ViewerOutputMap* _context_viewer_bird = 0;
    ViewerOutputMap* _context_viewer_top  = 0;

  //ds txt_io playback components
  protected:

    //ds txt_io message parser
    srrg_core::MessageReader _sensor_message_reader;

    //ds txt_io message synchronizer
    srrg_core::MessageTimestampSynchronizer _synchronizer;

    //ds loaded cameras
    CameraMap _cameras_by_topic;

    //ds false if termination was requested
    bool _is_gui_running = true;

  //ds informative only
  protected:

    //ds recorded ground truth input
    std::vector<TransformMatrix3D> _robot_to_world_ground_truth_poses;

    //ds total system runtime
    double _duration_total_seconds = 0;
  };
}
