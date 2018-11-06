#pragma once
#include "frame.h"

namespace proslam {

//ds this class represents a salient 3D point in the world, perceived in a sequence of images
class Landmark {
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//ds exported types
public:

  //ds snapshot of the landmark in an arbitrary context (e.g. LocalMap)
  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State(Landmark* landmark_,
          const AppearanceVector& appearances_,
          const PointCoordinates& coordinates_): landmark(landmark_),
                                                 appearances(appearances_),
                                                 coordinates(coordinates_) {}

    Landmark* landmark;
    AppearanceVector appearances;
    PointCoordinates coordinates;
  };
  typedef std::vector<State*> StatePointerVector;

  //ds a landmark measurement (used for position optimization)
  struct Measurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Measurement(const FramePoint* framepoint_): world_to_camera(framepoint_->frame()->worldToCameraLeft()),
                                                camera_coordinates(framepoint_->cameraCoordinatesLeft()),
                                                world_coordinates(framepoint_->worldCoordinates()),
                                                inverse_depth_meters(1/framepoint_->cameraCoordinatesLeft().z()) {}

    Measurement(): world_to_camera(TransformMatrix3D::Identity()),
                   camera_coordinates(PointCoordinates::Zero()),
                   world_coordinates(PointCoordinates::Zero()),
                   inverse_depth_meters(0) {}

    TransformMatrix3D world_to_camera;
    PointCoordinates camera_coordinates;
    PointCoordinates world_coordinates;
    real inverse_depth_meters;
  };
  typedef std::vector<Measurement, Eigen::aligned_allocator<Measurement>> MeasurementVector;

//ds object handling: specific instantiation controlled by WorldMap class (factory)
protected:

  //ds initial landmark coordinates must be provided
  Landmark(FramePoint* origin_, const LandmarkParameters* parameters_);

  //ds cleanup of dynamic structures
  ~Landmark();

  //ds prohibit default construction
  Landmark() = delete;

//ds getters/setters
public:

  //ds unique identifier for a landmark (exists once in memory)
  inline const Index identifier() const {return _identifier;}

  //ds framepoint in an image at the time when the landmark was created
  inline FramePoint* origin() const {return _origin;}

  inline const PointCoordinates& coordinates() const {return _world_coordinates;}
  void setCoordinates(const PointCoordinates& coordinates_) {_world_coordinates = coordinates_;}

  const AppearanceVector& appearances() const {return _appearances;}
  void addState(State* landmark_state_);

  //ds position related
  const Count numberOfUpdates() const {return _number_of_updates;}

  //ds information about whether the landmark is visible in the current image
  inline const bool isCurrentlyTracked() const {return _is_currently_tracked;}
  inline void setIsCurrentlyTracked(const bool& is_currently_tracked_) {_is_currently_tracked = is_currently_tracked_;}

  //ds landmark coordinates update with visual information (tracking)
  void update(const FramePoint* point_);

  const Count& numberOfRecoveries() const {return _number_of_recoveries;}
  void incrementNumberOfRecoveries() {++_number_of_recoveries;}
  void setNumberOfRecoveries(const Count& number_of_recoveries_) {_number_of_recoveries = number_of_recoveries_;}

  //! @brief incorporates another landmark into this (e.g. used when relocalizing)
  //! @param[in] landmark_ the landmark to absorbed, landmark_ will be freed and its memory location will point to this
  void merge(Landmark* landmark_);

  //ds reset allocated object counter
  static void reset() {_instances = 0;}

  //ds visualization only
  inline const bool isInLoopClosureQuery() const {return _is_in_loop_closure_query;}
  inline const bool isInLoopClosureReference() const {return _is_in_loop_closure_reference;}
  inline void setIsInLoopClosureQuery(const bool& is_in_loop_closure_query_) {_is_in_loop_closure_query = is_in_loop_closure_query_;}
  inline void setIsInLoopClosureReference(const bool& is_in_loop_closure_reference_) {_is_in_loop_closure_reference = is_in_loop_closure_reference_;}

//ds attributes
protected:

  //ds unique identifier for a landmark (exists once in memory)
  const Identifier _identifier;

  //ds linked FramePoint in an image at the time of creation of this instance
  FramePoint* _origin;

  //ds world coordinates of the landmark
  PointCoordinates _world_coordinates;

  //ds currently active appearances of this landmark (not yet in local map)
  AppearanceVector _appearances;

  //ds landmark states captured in local maps
  StatePointerVector _states_in_local_maps;

  //ds flags
  bool _is_currently_tracked = false; //ds set if the landmark is visible (=tracked) in the current image

  //ds landmark coordinates optimization
  MeasurementVector _measurements;
  Count _number_of_updates    = 0;
  Count _number_of_recoveries = 0;

  //ds grant access to landmark factory
  friend WorldMap;

  //ds visualization only
  bool _is_in_loop_closure_query     = false;
  bool _is_in_loop_closure_reference = false;

//ds class specific
private:

  //! @brief configurable parameters
  const LandmarkParameters* _parameters;

  //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
  static Count _instances;
};

typedef std::vector<Landmark*> LandmarkPointerVector;
typedef std::pair<const Identifier, Landmark*> LandmarkPointerMapElement;
typedef std::map<const Identifier, Landmark*> LandmarkPointerMap;
typedef std::set<const Landmark*> LandmarkPointerSet;
}
