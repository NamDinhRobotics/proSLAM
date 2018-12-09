#pragma once
#include "landmark.h"
#include "relocalization/closure.h"

namespace proslam {

//! @class this class condenses a group of Frame objects into a single Local Map object, which used for relocalization and pose optimization
class LocalMap {
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//ds object handling
protected:

  //! @brief constructs a local map that lives in the reference frame of the consumed frames
  //! @param[in] frames_ the collection of frames to be contained in the local map (same track)
  //! @param[in] local_map_root_ the first local map in the same track
  //! @param[in] local_map_previous_ the preceding local map in the same track
  //! @param[in] minimum_number_of_landmarks_ target minimum number of landmarks to contain in local map
  LocalMap(FramePointerVector& frames_,
           const LocalMapParameters* parameters_,
           LocalMap* local_map_root_ = nullptr,
           LocalMap* local_map_previous_ = nullptr);

  //ds cleanup of dynamic structures
  ~LocalMap();

  //ds prohibit default construction
  LocalMap() = delete;

//ds functionality:
public:

  //! @brief clears all internal structures (prepares a fresh world map)
  void clear();

  //! @brief adds a loop closure constraint between this local map and a reference map
  //! @param[in] local_map_reference_ the corresponding reference local map
  //! @param[in] transform_query_to_reference_ the spatial relation between query and reference (from query to reference)
  //! @param[in] landmark_correspondences_ underlying landmark correspondences
  //! @param[in] omega_ 1D information value of the correspondence
  void addCorrespondence(LocalMap* local_map_reference_,
                         const TransformMatrix3D& query_to_reference_,
                         const Closure::CorrespondencePointerVector& landmark_correspondences_,
                         const real& omega_ = 1) {_closures.push_back(Closure::ClosureConstraint(local_map_reference_, query_to_reference_, landmark_correspondences_, omega_));}

  //! @brief replaces a landmark with another (e.g. merged)
  //! @param[in] landmark_old_ landmark currently in this local map
  //! @param[in] landmark_new_ landmark to replace the currently present landmark_old_ in this local map
  void replace(Landmark* landmark_old_, Landmark* landmark_new_);

//ds getters/setters
public:

  inline const Identifier& identifier() const {return _identifier;}

  inline const TransformMatrix3D& robotToWorld() const {return _keyframe->robotToWorld();}
  inline const TransformMatrix3D& worldToRobot() const {return _keyframe->worldToRobot();}
  void setRobotToWorld(const TransformMatrix3D& robot_to_world_, const bool update_landmark_world_coordinates_ = false);

  inline LocalMap* root() {return _root;}
  void setRoot(LocalMap* root_) {_root = root_;}
  inline LocalMap* previous() {return _previous;}
  void setPrevious(LocalMap* local_map_) {_previous = local_map_;}
  inline LocalMap* next() {return _next;}
  void setNext(LocalMap* local_map_) {_next = local_map_;}
  inline Frame* keyframe() const {return _keyframe;}
  inline const FramePointerVector& frames() const {return _frames;}
  inline Closure::LandmarkStateMap& landmarks() {return _landmarks;}
  inline AppearanceVector& appearances() {return _appearances;}
  inline const AppearanceVector& appearances() const {return _appearances;}

  //ds TODO purge this
  inline const Closure::ClosureConstraintVector& closures() const {return _closures;}

  //ds reset allocated object counter
  static void reset() {_instances = 0;}

//ds attributes
protected:

  //ds unique identifier for a local map (exists once in memory)
  const Identifier _identifier;

  //ds links to preceding and subsequent instances
  LocalMap* _root;
  LocalMap* _previous;
  LocalMap* _next = nullptr;

  //! @brief the keyframe of the local map
  Frame* _keyframe;

  //ds the contained Frames
  FramePointerVector _frames;

  //ds the contained landmarks with coordinates in the local map frame (i.e. w.r.t. key frame)
  //ds these estimates are currently frozen after local map creation TODO update it after optimization
  Closure::LandmarkStateMap _landmarks;

  //ds appearance vector, corresponding to the union of all appearances stored in _landmarks
  //ds this vector is emptied after a local map gets consumed by HBST for place recognition
  AppearanceVector _appearances;

  //ds loop closures for the local map
  Closure::ClosureConstraintVector _closures;

  //ds grant access to local map producer
  friend WorldMap;

//ds class specific
private:

  //! @brief configurable parameters
  const LocalMapParameters* _parameters;

  //! @brief inner instance count - incremented upon constructor call (also unsuccessful calls)
  static Count _instances;

};

typedef std::vector<LocalMap*> LocalMapPointerVector;
typedef std::set<LocalMap*> LocalMapPointerSet;
typedef std::vector<const LocalMap*> ConstLocalMapPointerVector;

}
