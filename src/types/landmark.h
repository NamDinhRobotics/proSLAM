#pragma once
#include "frame.h"

namespace proslam {

  //ds this class represents a salient 3D point in the world, perceived in a sequence of images
  class Landmark {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

    //ds container describing the landmark at the time of local map construction
    struct State {
      State(Landmark* landmark_): landmark(landmark_), coordinates_in_local_map(Vector3::Zero()), world_coordinates(Vector3::Zero()), local_map(0) {
        appearances.clear();
      }
      ~State() {
        for (const HBSTNode::Matchable* matchable: appearances) {
          delete matchable;
        }
        appearances.clear();
      }

      Landmark* landmark;
      HBSTNode::MatchableVector appearances;
      PointCoordinates coordinates_in_local_map;
      PointCoordinates world_coordinates;
      const LocalMap* local_map;
    };
    typedef std::vector<State*> StatePointerVector;

  //ds object handling: specific instantiation controlled by WorldMap class (factory)
  protected:

    //ds initial landmark coordinates must be provided
    Landmark(const FramePoint* origin_, const LandmarkParameters* parameters_);

    //ds cleanup of dynamic structures
    ~Landmark();

    //ds prohibit default construction
    Landmark() = delete;

  //ds getters/setters
  public:

    //ds unique identifier for a landmark (exists once in memory)
    inline const Index identifier() const {return _identifier;}

    //ds framepoint in an image at the time when the landmark was created
    inline const FramePoint* origin() const {return _origin;}

    inline const PointCoordinates& coordinates() const { return _coordinates; }
    inline void setCoordinates(const PointCoordinates& coordinates_) {_coordinates = coordinates_;}

    //ds reset landmark coordinates to a certain position (loss of past measurements!)
    void resetCoordinates(const PointCoordinates& coordinates_);

    //ds landmark state - locked inside a local map and refreshed afterwards
    inline State* state() {return _state;}
    inline void refreshState() {_state = new State(this);}
    inline const LocalMap* localMap() const {return _local_map;}
    inline void setLocalMap(const LocalMap* local_map_) {_local_map = local_map_;}

    //ds position related
    inline const bool areCoordinatesValidated() const {return _are_coordinates_validated;}
    const Count numberOfUpdates() const {return _number_of_updates;}

    //ds information about whether the landmark is visible in the current image
    inline const bool isCurrentlyTracked() const {return _is_currently_tracked;}
    inline void setIsCurrentlyTracked(const bool& is_currently_tracked_) {_is_currently_tracked = is_currently_tracked_;}

    //ds landmark coordinates update - without visual information (e.g. map optimization)
    void update(const PointCoordinates& coordinates_in_world_, const real& depth_meters_ = 1);

    //ds landmark coordinates update with visual information (tracking)
    void update(const FramePoint* point_);

    const Count& numberOfRecoveries() const {return _number_of_recoveries;}
    void incrementNumberOfRecoveries() {++_number_of_recoveries;}
    void setNumberOfRecoveries(const Count& number_of_recoveries_) {_number_of_recoveries = number_of_recoveries_;}

    //ds visualization only
    inline const bool isNear() const {return _is_near;}
    inline void setIsNear(const bool& is_near_) {_is_near = is_near_;}
    inline const bool isInLoopClosureQuery() const {return _is_in_loop_closure_query;}
    inline const bool isInLoopClosureReference() const {return _is_in_loop_closure_reference;}
    inline void setIsInLoopClosureQuery(const bool& is_in_loop_closure_query_) {_is_in_loop_closure_query = is_in_loop_closure_query_;}
    inline void setIsInLoopClosureReference(const bool& is_in_loop_closure_reference_) {_is_in_loop_closure_reference = is_in_loop_closure_reference_;}

  //ds attributes
  protected:

    //ds unique identifier for a landmark (exists once in memory)
    const Identifier _identifier;

    //ds linked FramePoint in an image at the time of creation of this instance
    const FramePoint* _origin;

    //ds the 3D point coordinates of the landmark expressed in the WorldMap coordinate frame
    PointCoordinates _coordinates;

    //ds the current connected state handle (links the landmark to the local map)
    State* _state;
    const LocalMap* _local_map = 0;

    //ds flags
    bool _are_coordinates_validated = false; //ds 3D coordinates have been updated successfully with new measurements (updates)
    bool _is_currently_tracked      = false; //ds set if the landmark is visible (=tracked) in the current image
    bool _is_near                   = false; //ds set if the landmark coordinates are within a certain threshold (close to the camera)

    //ds landmark coordinates optimization
    real _total_weight = 0;
    Count _number_of_updates = 0;
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
  typedef std::pair<Identifier, Landmark*> LandmarkPointerMapElement;

  class LandmarkPointerMap: public std::map<Identifier, Landmark*> {
  public:
    Landmark* get(const Identifier& identifier_);
    void put(Landmark* landmark);
  };
}
