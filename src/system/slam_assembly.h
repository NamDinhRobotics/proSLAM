#include "qapplication.h"
#include <thread>
#include <atomic>

#include "srrg_messages/message_reader.h"
#include "srrg_messages/message_timestamp_synchronizer.h"

#include "position_tracking/base_tracker.h"
#include "map_optimization/graph_optimizer.h"
#include "relocalization/relocalizer.h"
#include "visualization/image_viewer.h"
#include "visualization/map_viewer.h"
#include "framepoint_generation/stereo_framepoint_generator.h"

namespace proslam {

//ds simple assembly of the different SLAM modules provided by ProSLAM
class SLAMAssembly {

//ds object management
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  void initializeGUI(std::shared_ptr<QApplication> ui_server_);

  //ds updates GUI components with current state
  void updateGUI();

  //! @brief triggers draw functions of all connected viewers
  void draw();

  //! @brief saves current pose graph to disk
  //! @param[in] file_name_ desired file name for the g2o outfile
  void writePoseGraphToFile(const std::string& file_name_ = "pose_graph.g2o") const;

  //! @brief playback txt_io message file
  void playbackMessageFile();

  //! @brief thread wrapping
  std::shared_ptr<std::thread> playbackMessageFileInThread() {
    return std::make_shared<std::thread>([=] {playbackMessageFile();});
  }

  //! @brief process a pair of rectified and undistorted stereo images
  void process(const cv::Mat& intensity_image_left_,
               const cv::Mat& intensity_image_right_,
               const double& timestamp_image_left_seconds_ = 0,
               const bool& use_odometry_ = false,
               const TransformMatrix3D& odometry_ = TransformMatrix3D::Identity());

  //ds prints extensive run summary
  void printReport() const;

  //! @brief dump trajectory to file (in KITTI benchmark format: 4x4 isometries per line)
  //! @param[in] file_name_ text file path in which the poses are saved to
  void writeTrajectoryKITTI(const std::string& file_name_ = "") const {if (_world_map) {_world_map->writeTrajectoryKITTI(file_name_);}}

  //! @brief dump trajectory to file (in TUM benchmark format: timestamp x z y and qx qy qz qw per line)
  //! @param[in] filename_ text file in which the poses are saved to
  void writeTrajectoryTUM(const std::string& file_name_ = "") const {if (_world_map) {_world_map->writeTrajectoryTUM(file_name_);}}

  //! @brief save trajectory to a vector
  //! @param[in,out] poses_ vector with poses, set in the function
  template<typename RealType>
  void writeTrajectory(std::vector<Eigen::Matrix<RealType, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<RealType, 4, 4>>>& poses_) const {if (_world_map) {_world_map->writeTrajectory<RealType>(poses_);}}

  //! @brief save trajectory to a vector with timestamps
  //! @param[in,out] poses_ vector with timestamps and poses, set in the function
  template<typename RealType>
  void writeTrajectoryWithTimestamps(std::vector<std::pair<RealType, Eigen::Transform<RealType, 3, Eigen::Isometry>>>& poses_) const {if (_world_map) {_world_map->writeTrajectoryWithTimestamps<RealType>(poses_);}}

  //! @brief resets the complete pipeline, releasing memory
  void reset();

//ds getters/setters
public:

  void requestTermination() {_is_termination_requested = true;}
  const bool isViewerOpen() const {return _is_viewer_open;}
  const double currentFPS() const {return _current_fps;}
  const double averageNumberOfLandmarksPerFrame() const {return _tracker->totalNumberOfLandmarks()/_number_of_processed_frames;}
  const double averageNumberOfTracksPerFrame() const {return _tracker->totalNumberOfTrackedPoints()/_number_of_processed_frames;}
  const Count numberOfRecursiveRegistrations() const {return _tracker->numberOfRecursiveRegistrations();}
  const real meanTrackingRatio() const {return _tracker->meanTrackingRatio();}
  const real meanTriangulationRatio() const {return dynamic_cast<StereoFramePointGenerator*>(_tracker->framepointGenerator())->meanTriangulationSuccessRatio();}

//ds helpers:
protected:

  void _createStereoTracker(Camera* camera_left_, Camera* camera_right_);

  void _createDepthTracker(Camera* camera_left_, Camera* camera_right_);

//ds SLAM modules
protected:

  //! @brief all configurable system parameters
  ParameterCollection* _parameters;

  //ds the SLAM map, containing landmarks and trajectory
  WorldMap* _world_map;

  //ds pose graph optimization handler
  GraphOptimizer* _graph_optimizer;

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
  std::shared_ptr<QApplication> _ui_server;

  //ds raw image input processing viewer
  std::shared_ptr<ImageViewer> _image_viewer;

  //ds 3D output viewers
  std::shared_ptr<MapViewer> _map_viewer;
  std::shared_ptr<MapViewer> _minimap_viewer;

//  std::atomic<bool> _new_image_available;

//ds playback components
protected:

  //! @brief srrg message parser
  srrg_core::MessageReader _message_reader;

  //ds txt_io message synchronizer
  srrg_core::MessageTimestampSynchronizer _synchronizer;

  //! @brief termination check - terminates processing loop
  std::atomic<bool> _is_termination_requested;

  //! @brief flag that is checked if an OpenGL or OpenCV window is currently active
  std::atomic<bool> _is_viewer_open;

//ds informative only
protected:

  //! @brief total system processing runtime
  double _processing_time_total_seconds = 0;

  //! @brief frame-wise processing times
  std::vector<double> _processing_times_seconds;

  //! @brief total number of processed frames
  Count _number_of_processed_frames = 0;

  //! @brief current average fps
  double _current_fps = 0;
};
}
