#pragma once
#include <mutex>
#include <atomic>
#include <QtGlobal>
#include "srrg_core_viewers/simple_viewer.h"
#include "types/frame.h"
#include "types/landmark.h"

namespace proslam {

class MapViewer: public srrg_core_viewers::SimpleViewer {

//ds exports
public:

  //! @struct thread-safe, drawable container for a Landmark object
  struct DrawableLandmark {

    //! @brief default constructor
    DrawableLandmark(): world_coordinates(PointCoordinates::Zero()),
                        is_near(false),
                        is_in_loop_closure_query(false),
                        is_in_loop_closure_reference(false) {}

    //! @brief map insert constructor
    DrawableLandmark(const PointCoordinates& world_coordinates_,
                     const bool& is_near_,
                     const bool& is_in_loop_closure_query_,
                     const bool& is_in_loop_closure_reference_): world_coordinates(world_coordinates_),
                                                                 is_near(is_near_),
                                                                 is_in_loop_closure_query(is_in_loop_closure_query_),
                                                                 is_in_loop_closure_reference(is_in_loop_closure_reference_) {}

    PointCoordinates world_coordinates;
    bool is_near;
    bool is_in_loop_closure_query;
    bool is_in_loop_closure_reference;
  };

  //! @struct thread-safe, drawable container for a Frame object
  struct DrawableFrame {

    //! @brief default constructor
    DrawableFrame(): robot_to_world(TransformMatrix3D::Identity()),
                     robot_to_world_ground_truth(TransformMatrix3D::Identity()),
                     is_keyframe(false),
                     is_closed(false) {}

    //! @brief map insert constructor
    DrawableFrame(const TransformMatrix3D& robot_to_world_,
                  const TransformMatrix3D& robot_to_world_ground_truth_,
                  const bool& is_keyframe_,
                  const bool& is_closed_): robot_to_world(robot_to_world_),
                                           robot_to_world_ground_truth(robot_to_world_ground_truth_),
                                           is_keyframe(is_keyframe_),
                                           is_closed(is_closed_) {}

    TransformMatrix3D robot_to_world;
    TransformMatrix3D robot_to_world_ground_truth;
    bool is_keyframe;
    bool is_closed;
  };

//ds object life
public:

  //! @brief default constructor
  //! @brief param[in] object_scale_ global display scale of objects in the map
  //! @brief param[in] window_title_ the GL window title
  MapViewer(const real& object_scale_ = 0.25,
            const std::string& window_title_ = "output: map");

  //! @brief default destructor
  ~MapViewer();

//ds access
public:

  //! @brief GUI update function, copies inner objects from provided frame into thread-safe, drawable containers - LOCKING
  //! @param[in] frame_ the frame to display
  //! @param[in] frame_queue_for_local_map_
  //! @param[in] permanent_frames_
  //! @param[in] tracked_landmarks_ vector of currently tracked landmarks
  //! @param[in] temporary_landmarks_
  //! @param[in] permanent_landmarks_
  //! @param[in] graph_optimized_
  void update(const Frame* frame_,
              const FramePointerVector& frame_queue_for_local_map_,
              const FramePointerMap& permanent_frames_,
              const std::vector<Landmark*>& tracked_landmarks_,
              const LandmarkPointerMap& temporary_landmarks_,
              const LandmarkPointerMap& permanent_landmarks_,
              const bool& graph_optimized_ = false);

//ds setters/getters
public:

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
  void _drawFrame(const DrawableFrame& frame_, const Vector3& color_rgb_);

  //! @brief draws landmark in different states in the map
  void _drawLandmarks();

  //! @brief draws currently generated framepoints (set with update)
  void _drawFramepoints();

//ds attributes
protected:

  //! @brief mutex for data exchange, owned by the viewer
  std::mutex _mutex_data_exchange;

  //! @brief frames in current frame queue for local map generation (head)
  std::vector<DrawableFrame> _frame_queue_for_local_map;

  //! @brief permanent frames in map
  std::map<Identifier, DrawableFrame> _permanent_frames;

  //! @brief frame iterator at last addition (update call) - assuming that the permanent frame map is consistently growing
  FramePointerMap::const_iterator _iterator_permanent_frame_update;

  //! @brief active framepoints to be drawn
  std::vector<PointCoordinates> _active_framepoints;

  //! @brief currently tracked landmarks to be drawn
  std::vector<DrawableLandmark> _tracked_landmarks;

  //! @brief landmarks in current window for local map creation (not permanent yet)
  std::vector<DrawableLandmark> _temporary_landmarks;

  //! @brief landmarks which are to remain in the map
  std::map<Identifier, DrawableLandmark> _permanent_landmarks;

  //! @brief landmark iterator at last addition (update call) - assuming that the permanent landmark map is consistently growing
  LandmarkPointerMap::const_iterator _iterator_permanent_landmark_update;

  //! @brief display options
  bool _frames_drawn             = false;
  bool _local_maps_drawn         = true;
  bool _landmarks_drawn          = true;
  bool _follow_robot             = true;
  bool _ground_truth_drawn       = false;

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
