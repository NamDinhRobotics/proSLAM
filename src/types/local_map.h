#pragma once
#include "landmark.h"
#include "relocalization/closure.h"

namespace proslam {

//! @class this class condenses a group of Frame objects into a single Local Map object, which used for relocalization and pose optimization
class LocalMap {

//ds exported types
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds loop closure constraint element between 2 local maps TODO move to relocalizer
  struct ClosureConstraint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ClosureConstraint(const LocalMap* local_map_,
                      const TransformMatrix3D& relation_,
                      const Closure::CorrespondencePointerVector& landmark_correspondences_,
                      const real& omega_ = 1): local_map(local_map_),
                                               relation(relation_),
                                               landmark_correspondences(landmark_correspondences_),
                                               omega(omega_){}

    const LocalMap* local_map;
    const TransformMatrix3D relation;
    const Closure::CorrespondencePointerVector landmark_correspondences;
    const real omega;
  };

  typedef std::vector<ClosureConstraint, Eigen::aligned_allocator<ClosureConstraint>> ClosureConstraintVector;

  //ds landmark snapshot at creation of local map
  struct LandmarkState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LandmarkState(Landmark* landmark_,
                  PointCoordinates coordinates_in_local_map_): landmark(landmark_),
                                                               coordinates_in_local_map(coordinates_in_local_map_) {}

    void updateCoordinatesInWorld(const TransformMatrix3D& local_map_to_world_) {
      landmark->setCoordinates(local_map_to_world_*coordinates_in_local_map);
    }

    Landmark* landmark;
    PointCoordinates coordinates_in_local_map;
  };

  typedef std::pair<const Identifier, LandmarkState> LandmarkStateMapElement;
  typedef std::map<const Identifier, LandmarkState, std::less<Identifier>, Eigen::aligned_allocator<LandmarkStateMapElement> > LandmarkStateMap;

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

  //! @brief updates local map pose, automatically updating contained Frame poses (pyramid)
  //! @param[in] local_map_to_world_ the local map pose with respect to the world map coordinate frame
  void update(const TransformMatrix3D& local_map_to_world_);

  //! @brief adds a loop closure constraint between this local map and a reference map
  //! @param[in] local_map_reference_ the corresponding reference local map
  //! @param[in] transform_query_to_reference_ the spatial relation between query and reference (from query to reference)
  //! @param[in] landmark_correspondences_ underlying landmark correspondences
  //! @param[in] omega_ 1D information value of the correspondence
  void addCorrespondence(const LocalMap* local_map_reference_,
                         const TransformMatrix3D& query_to_reference_,
                         const Closure::CorrespondencePointerVector& landmark_correspondences_,
                         const real& omega_ = 1) {_closures.push_back(ClosureConstraint(local_map_reference_, query_to_reference_, landmark_correspondences_, omega_));}

  //! @brief replaces a landmark with another (e.g. merged)
  //! @param[in] landmark_old_ landmark currently in this local map
  //! @param[in] landmark_new_ landmark to replace the currently present landmark_old_ in this local map
  void replace(Landmark* landmark_old_, Landmark* landmark_new_);

//ds getters/setters
public:

  inline const Identifier& identifier() const {return _identifier;}

  inline const TransformMatrix3D& localMapToWorld() const {return _local_map_to_world;}
  inline const TransformMatrix3D& worldToLocalMap() const {return _world_to_local_map;}
  void setLocalMapToWorld(const TransformMatrix3D& local_map_to_world_, const bool update_landmark_world_coordinates_ = false);

  inline LocalMap* root() {return _root;}
  void setRoot(LocalMap* root_) {_root = root_;}
  inline LocalMap* previous() {return _previous;}
  void setPrevious(LocalMap* local_map_) {_previous = local_map_;}
  inline LocalMap* next() {return _next;}
  void setNext(LocalMap* local_map_) {_next = local_map_;}
  inline Frame* keyframe() const {return _keyframe;}
  inline const FramePointerVector& frames() const {return _frames;}
  inline LandmarkStateMap& landmarks() {return _landmarks;}
  inline AppearanceVector& appearances() {return _appearances;}
  inline const AppearanceVector& appearances() const {return _appearances;}

  //ds TODO purge this
  inline const ClosureConstraintVector& closures() const {return _closures;}

  //ds reset allocated object counter
  static void reset() {_instances = 0;}

//ds attributes
protected:

  //ds unique identifier for a local map (exists once in memory)
  const Identifier _identifier;

  //! @brief pose of the local map with respect to the world map coordinate frame
  TransformMatrix3D _local_map_to_world = TransformMatrix3D::Identity();

  //! @brief transform to map world geometries into the local map coordinate frame
  TransformMatrix3D _world_to_local_map = TransformMatrix3D::Identity();

  //ds links to preceding and subsequent instances
  LocalMap* _root;
  LocalMap* _previous;
  LocalMap* _next;

  //! @brief the keyframe of the local map
  Frame* _keyframe;

  //ds the contained Frames
  FramePointerVector _frames;

  //ds the contained landmarks with coordinates in the local map frame (i.e. w.r.t. key frame)
  //ds these estimates are currently frozen after local map creation TODO update it after optimization
  LandmarkStateMap _landmarks;

  //ds appearance vector, corresponding to the union of all appearances stored in _landmarks
  //ds this vector is emptied after a local map gets consumed by HBST for place recognition
  AppearanceVector _appearances;

  //ds loop closures for the local map
  ClosureConstraintVector _closures;

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
