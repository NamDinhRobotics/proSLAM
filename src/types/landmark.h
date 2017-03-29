#pragma once
#include "frame.h"

namespace proslam {

  //ds this class represents a salient 3D point in the world, perceived in a sequence of images
  class Landmark {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

    //ds inner forward declarations
    struct State;

    //ds container encapsulating the visual information of the appearance of a landmark in an image
    struct Appearance {
      Appearance(const State* landmark_state_,
                 const cv::Mat& descriptor_cv_): landmark_state(landmark_state_),
                                                 descriptor(getDescriptor(descriptor_cv_)) {}

      const State* landmark_state;
      const HBSTMatchable::BinaryDescriptor descriptor;
    };
    typedef std::vector<const Appearance*> AppearancePtrVector;

    //ds container describing the landmark at the time of local map construction
    struct State {
      State(Landmark* landmark_): landmark(landmark_) {
        appearances.clear();
      }
      ~State() {
        for (const Appearance* appearance: appearances) {
          delete appearance;
        }
        appearances.clear();
      }

      Landmark* landmark;
      AppearancePtrVector appearances;
      PointCoordinates robot_coordinates;
      const LocalMap* local_map = 0;
    };
    typedef std::vector<State*> StatePointerVector;

  //ds object handling: specific instantiation controlled by WorldMap class (factory)
  protected:

    //ds initial landmark coordinates must be provided
    Landmark(const PointCoordinates& point_coordinates_);

    //ds cleanup of dynamic structures
    ~Landmark();

    //ds prohibit default construction
    Landmark() = delete;

  //ds getters/setters
  public:

    //ds unique identifier for a landmark (exists once in memory)
    inline const Index identifier() const {return _identifier;}

    inline const PointCoordinates& coordinates() const { return _coordinates; }
    inline void setCoordinates(const PointCoordinates& coordinates_) {_coordinates = coordinates_;}

    //ds reset landmark coordinates to a certain position (loss of past measurements!)
    void resetCoordinates(const PointCoordinates& coordinates_);

    //ds landmark state - locked inside a local map and refreshed afterwards
    inline State* state() {return _state;}
    inline void refreshState() {_state = new State(this);}

    inline const bool areCoordinatesValidated() const {return _are_coordinates_validated;}
    inline const bool isInPoseGraph() const {return _is_in_pose_graph; }
    inline void setIsInPoseGraph(const bool& is_in_pose_graph_) {_is_in_pose_graph = is_in_pose_graph_;}

    //ds landmark coordinates update - no visual information (e.g. map optimization)
    void update(const PointCoordinates& coordinates_in_world_,
                const real& depth_meters_ = 1);

    //ds landmark coordinates update with visual information (tracking)
    void update(const PointCoordinates& coordinates_in_world_,
                const cv::Mat& descriptor_left_,
                const cv::Mat& descriptor_right_,
                const real& depth_meters_);

    //ds visualization only
    inline const bool isActive() const {return _is_active; }
    inline void setIsActive(const bool& is_active) {_is_active = is_active;}
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

    //ds the 3D point coordinates of the landmark expressed in the WorldMap coordinate frame
    PointCoordinates _coordinates;
    PointCoordinates _coordinates_alt = PointCoordinates::Zero();

    //ds the current connected state handle (links the landmark to the local map)
    State* _state;

    //ds flags
    bool _are_coordinates_validated = false;
    bool _is_in_pose_graph          = false;

    //ds landmark coordinates optimization
    real _total_weight = 0;
    const real _maximum_acceptable_relative_displacement = 0.75;
    Count _number_of_updates = 0;

    //ds grant access to landmark factory
    friend WorldMap;

    //ds visualization only
    bool _is_active                    = false;
    bool _is_near                      = false;
    bool _is_in_loop_closure_query     = false;
    bool _is_in_loop_closure_reference = false;

  //ds class specific
  private:

    //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
    static Count _instances;
  };
  
  typedef std::vector<Landmark*> LandmarkPointerVector;
  typedef std::pair<Identifier, Landmark*> LandmarkPointerMapElement;

  class LandmarkPointerMap: public std::map<Identifier, Landmark*> {
  public:
    Landmark* get(const Identifier& identifier_);
    void put(Landmark* landmark);

    //ds visualization only
    void clearActive(); 
  };
}
