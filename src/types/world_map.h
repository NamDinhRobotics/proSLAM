#pragma once
#include "local_map.h"

namespace proslam {

  //ds the world map is an overarching map entity, generating and owning all landmarks, frames and local map objects
  class WorldMap {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds object handling
  public:

    WorldMap();
    virtual void configure(WorldMapParameters* parameters_);
    virtual ~WorldMap();

  //ds functionality
  public:

    //ds clears all internal structures (prepares a fresh world map)
    void clear();

    //ds creates a new frame living in this instance at the provided pose
    Frame* createFrame(const TransformMatrix3D& robot_to_world_, const real& maximum_depth_near_);

    //ds creates a new landmark living in this instance, using the provided framepoint as origin
    Landmark* createLandmark(const FramePoint* origin_);

    //ds attempts to create a new local map if the generation criteria are met (returns true if a local map was generated)
    const bool createLocalMap();

    //ds resets the window for the local map generation
    void resetWindowForLocalMapCreation();

    //ds adds a loop closure constraint between 2 local maps
    void addCorrespondence(LocalMap* query_, const LocalMap* reference_, const TransformMatrix3D& transform_query_to_reference_, const real& omega_ = 1);

    //ds dump trajectory to file (in KITTI benchmark format: 4x4 isometries per line)
    void writeTrajectory(const std::string& filename_ = "") const;

    //! @brief this function does what you think it does
    //! @param[in] frame_ frame at which the track was broken
    void breakTrack(const Frame* frame_);

    //! @brief this function does what you think it does
    //! @param[in] frame_ frame at which the track was found again
    void setTrack(Frame* frame_);

  //ds getters/setters
  public:

    const Frame* rootFrame() const {return _root_frame;}
    Frame* currentFrame() const {return _current_frame;}
    void setCurrentFrame(Frame* current_frame_) {_current_frame = current_frame_;}
    const Frame* previousFrame() const {return _previous_frame;}
    void setPreviousFrame(Frame* previous_frame_) {_previous_frame = previous_frame_;}

    const LandmarkPointerMap& landmarksInWindowForLocalMap() const {return _landmarks_in_window_for_local_map;}
    LandmarkPointerMap& landmarks() {return _landmarks;}
    std::vector<Landmark*>& currentlyTrackedLandmarks() {return _currently_tracked_landmarks;}

    LocalMap* currentLocalMap() {return _current_local_map;}
    const LocalMapPointerVector& localMaps() const {return _local_maps;}

    void setRobotToWorld(const TransformMatrix3D& robot_pose_) {robot_to_world = robot_pose_;}
    const TransformMatrix3D robotToWorld() const {return robot_to_world;}

    const bool relocalized() const {return _relocalized;}
    const Count numberOfClosures() const {return _number_of_closures;}

    //ds memory saving option
    void setDropFramepoints(const bool& drop_framepoints_) {_drop_framepoints = drop_framepoints_;}

    //ds visualization only
    const FramePointerMap& frames() const {return _frames;}
    const FramePointerVector& frameQueueForLocalMap() const {return _frame_queue_for_local_map;}
    void setRobotToWorldGroundTruth(const TransformMatrix3D& robot_to_world_ground_truth_) {if (_current_frame) {_current_frame->setRobotToWorldGroundTruth(robot_to_world_ground_truth_);}}

  //ds helpers
  public:

    //ds obtain angular values from rotation matrix - used for the local map generation criteria in rotation
    static const Vector3 toOrientationRodrigues(const Matrix3& rotation_matrix_) {
      cv::Vec<real, 3> rotation_angles;
      cv::Rodrigues(srrg_core::toCv(rotation_matrix_), rotation_angles);
      return srrg_core::fromCv(rotation_angles);
    }

  protected:

    //ds robot path information
    const Frame* _root_frame = 0;
    Frame* _current_frame    = 0;
    Frame* _previous_frame   = 0;

    //ds potential landmarks, to be moved into the permanent holder once validated
    LandmarkPointerMap _landmarks_in_window_for_local_map;

    //ds all permanent landmarks in the map
    LandmarkPointerMap _landmarks;

    //ds currently tracked landmarks (=visible in the current image)
    std::vector<Landmark*> _currently_tracked_landmarks;

    //ds active frames in the map
    FramePointerMap _frames;

    //ds localization
    TransformMatrix3D robot_to_world = TransformMatrix3D::Identity();
    bool _relocalized = false;

    //ds current frame window buffer for local map generation
    real _distance_traveled_window = 0;
    real _degrees_rotated_window   = 0;

    //ds local map control structures
    FramePointerVector _frame_queue_for_local_map;
    LocalMap* _current_local_map  = 0;
    LocalMapPointerVector _local_maps;

    //ds track recovery
    Frame* _last_frame_before_track_break = 0;
    LocalMap* _last_local_map_before_track_break = 0;
    LocalMap* _root_local_map = 0;

    //ds memory saving options (slightly less processing speed)
    bool _drop_framepoints = false;

  private:

    //! @brief configurable parameters
    WorldMapParameters* _parameters;

    //ds informative/visualization only
    Count _number_of_closures = 0;
  };
}
