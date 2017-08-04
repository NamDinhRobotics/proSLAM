#include "qapplication.h"
#include <thread>
#include <atomic>

#include "srrg_messages/message_reader.h"
#include "srrg_messages/message_timestamp_synchronizer.h"

#include "../position_tracking/base_tracker.h"
#include "map_optimization/graph_optimizer.h"
#include "relocalization/relocalizer.h"
#include "visualization/image_viewer.h"
#include "visualization/map_viewer.h"

namespace proslam {

//ds simple assembly of the different SLAM modules provided by ProSLAM
class SLAMAssembly {

//ds object management
public:

  //ds default constructor - already allocating required default objects
  SLAMAssembly(ParameterCollection* parameters_);

  //ds default destructor
  ~SLAMAssembly();

//ds functionality
public:

  //ds attempts to load the camera configuration based on the current input setting
  void loadCamerasFromMessageFile();

  //ds attempts to load the camera configuration based on the current input setting
  void loadCameras(Camera* camera_left_, Camera* camera_right_);

  //ds initializes gui components
  void initializeGUI(QApplication* ui_server_);

  //ds updates GUI components with current state
  void updateGUI();

  //! @brief triggers draw functions of all connected viewers
  void draw();

  //! @brief playback txt_io message file
  void playbackMessageFile();

  //! @brief thread wrapping
  std::thread* playbackMessageFileInThread() {
    return new std::thread([=] {playbackMessageFile();});
  }

  //ds sets ground truth to current frame
  void addGroundTruthMeasurement(const TransformMatrix3D& robot_to_world_ground_truth_);

  //ds process a pair of rectified and undistorted stereo images
  void process(const cv::Mat& intensity_image_left_,
               const cv::Mat& intensity_image_right_,
               const bool& use_odometry_ = false,
               const TransformMatrix3D& odometry_ = TransformMatrix3D::Identity());


  //ds prints extensive run summary
  void printReport() const;

  //! @brief dump trajectory to file (in KITTI benchmark format: 4x4 isometries per line)
  //! @param[in] file_name_ text file path in which the poses are saved to
  void writeTrajectory(const std::string& file_name_ = "") const {if (_world_map) {_world_map->writeTrajectory(file_name_);}}

  //! @brief save trajectory to a vector (in KITTI benchmark format: 4x4 isometries per line)
  //! @param[in,out] poses_ vector with poses, set in the function
  template<typename RealType>
  void writeTrajectory(std::vector<Eigen::Matrix<RealType, 4, 4>>& poses_) const {if (_world_map) {_world_map->writeTrajectory<RealType>(poses_);}}

  //! @brief resets the complete pipeline, releasing memory
  void reset();

//ds getters/setters
public:

  void requestTermination() {_is_termination_requested = true;}
  const bool isViewerOpen() const {return _is_viewer_open;}

//ds helpers:
protected:

  void _createStereoTracker(Camera* camera_left_, Camera* camera_right_);

  void _createDepthTracker(const Camera* camera_left_, const Camera* camera_right_);

//ds SLAM modules
protected:

  //! @brief all configurable system parameters
  ParameterCollection* _parameters;

  //ds the SLAM map, containing landmarks and trajectory
  WorldMap* _world_map;

  //ds pose graph optimization handler
  GraphOptimizer* _optimizer;

  //ds relocalization module
  Relocalizer* _relocalizer;

  //ds tracking component, deriving the robots odometry
  BaseTracker* _tracker;

  //ds loaded sensors
  Camera* _camera_left;
  Camera* _camera_right;

//ds visualization only
protected:

  //ds Qt UI server
  QApplication* _ui_server;

  //ds raw image input processing viewer
  ImageViewer* _image_viewer;

  //ds 3D output viewers
  MapViewer* _map_viewer;
  MapViewer* _minimap_viewer;

//ds txt_io playback components
protected:

  //ds txt_io message parser
  srrg_core::MessageReader _message_reader;

  //ds txt_io message synchronizer
  srrg_core::MessageTimestampSynchronizer _synchronizer;

  //! @brief termination check - terminates processing loop
  std::atomic<bool> _is_termination_requested;

  //! @brief viewer status
  bool _is_viewer_open;

//ds informative only
protected:

  //ds recorded ground truth input
  std::vector<TransformMatrix3D> _robot_to_world_ground_truth_poses;

  //ds total system runtime
  double _duration_total_seconds = 0;
};
}
