#pragma once
#include <mutex>
#include <atomic>
#include <QtGlobal>
#include "srrg_core_viewers/simple_viewer.h"
#include "types/world_map.h"

namespace proslam {

class MapViewer: public srrg_core_viewers::SimpleViewer {

//ds object life
public:

  //! @brief default constructor
  //! @brief param[in] object_scale_ global display scale of objects in the map
  //! @brief param[in] window_title_ the GL window title
  MapViewer(const real& object_scale_ = 0.25,
            const std::string& window_title_ = "output: map [OpenGL]");

  //! @brief default destructor
  ~MapViewer();

//ds access
public:

  //! @brief GUI update function, copies framepoints from provided frame into thread-safe, drawable containers  - LOCKING
  //! @param[in] frame_ the frame to display
  void update(const Frame* frame_);

  //! @brief manual data transfer locking (generally used to block the GUI from drawing during critical operations e.g. global map updates)
  void lock() {_mutex_data_exchange.lock();}

  //! @brief manual data transfer unlocking (enables the GUI to freely draw again)
  void unlock() {_mutex_data_exchange.unlock();}

//ds setters/getters
public:

  void setWorldMap(const WorldMap* world_map_) {_world_map = world_map_;}

  inline bool landmarksDrawn() const {return _landmarks_drawn;}
  inline void setLandmarksDrawn(const bool& landmarks_drawn_) {_landmarks_drawn = landmarks_drawn_;}

  inline bool framesDrawn() const {return _frames_drawn;}
  inline void setFramesDrawn(const bool& frames_drawn_) {_frames_drawn = frames_drawn_;}

  inline bool followRobot() const {return _follow_robot;}
  inline void setFollowRobot(const bool& follow_robot_) {_follow_robot = follow_robot_;}

  void setWorldToRobotOrigin(const TransformMatrix3D& world_to_robot_origin_) {_world_to_robot_origin = world_to_robot_origin_;}
  void setRotationRobotView(const TransformMatrix3D& rotation_robot_view_) {_robot_viewpoint = rotation_robot_view_;}
  void setCameraLeftToRobot(const TransformMatrix3D& camera_left_to_robot_) {_camera_left_to_robot = camera_left_to_robot_;}

  inline const bool optionStepwisePlayback() const {return _option_stepwise_playback;}
  inline const Count requestedPlaybackSteps() const {return _requested_playback_steps;}
  inline void decrementPlaybackSteps() {assert(_requested_playback_steps > 0); --_requested_playback_steps;}

//ds helpers
protected:

  //! @brief Qt standard draw function - LOCKING
  virtual void draw();

  //! @brief Qt key event handling
  virtual void keyPressEvent(QKeyEvent* event_);

  //! @brief Qt help string
  virtual QString helpString() const;

  //! @brief draws frame in the map, differentiating between local map anchors and regular frames
  //! @param[in] frame_
  //! @param[in] color_rgb_
  void _drawFrame(const Frame* frame_, const Vector3& color_rgb_) const;

  //! @brief draws landmark in different states in the map
  void _drawLandmarks() const;

  //! @brief draws framepoints for the currently active frame
  void _drawFramepoints() const;

//ds attributes
protected:

  //! @brief mutex for data exchange, owned by the viewer
  std::mutex _mutex_data_exchange;

  //! @brief map context (drawn with draw and updated with update method)
  const WorldMap* _world_map;

  //! @brief current frame handle (drawn in draw function)
  const Frame* _current_frame;

  //! @brief display options
  bool _frames_drawn;
  bool _landmarks_drawn;
  bool _follow_robot;
  bool _ground_truth_drawn;

  //! @brief enable stepwise playback
  std::atomic<bool> _option_stepwise_playback;

  //! @brief default sizes
  real _object_scale;
  real _point_size;

  //! @brief stepping buffer for stepwise playback
  std::atomic<Count> _requested_playback_steps;

  //! @brief viewer window title
  const std::string _window_title;

  //! @brief configuration: camera left to robot relation
  TransformMatrix3D _camera_left_to_robot;

  //! @brief display transformation: origin viewpoint
  TransformMatrix3D _world_to_robot_origin;

  //! @brief display transformation: viewpoint adjustment for ego perspective
  TransformMatrix3D _robot_viewpoint;

  //! @brief current robot position
  TransformMatrix3D _world_to_robot;
};
}
