#pragma once
#include "landmark.h"

namespace proslam {

  //ds this class condenses a group of Frame objects into a single Local Map object, which used for relocalization and pose optimization
  class LocalMap: public Frame {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

      //ds loop closure constraint element between 2 local maps
      struct Closure {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Closure(const LocalMap* local_map_,
                const TransformMatrix3D& relation_): local_map(local_map_),
                                                     relation(relation_) {}

        const LocalMap* local_map;
        const TransformMatrix3D relation;
      };
      typedef std::vector<Closure> ClosureVector;

  //ds object handling
  protected:

    //ds the local map is by the WorldMap and is created from a batch of Frames
    LocalMap(FramePointerVector& frames_);

    //ds cleanup of dynamic structures
    virtual ~LocalMap();

    //ds prohibit default construction
    LocalMap() = delete;

  //ds getters/setters
  public:

    //ds overridden pose method - automatically updating contained Frame poses
    virtual void setRobotToWorld(const TransformMatrix3D& robot_to_world_);

    //ds inner data
    const Landmark::AppearancePointerVector& appearances() const {return _appearances;}
    const Landmark::StatePointerVector& landmarks() const {return _landmarks;}

    //ds adds a loop closure constraint between this local map and a reference map
    void add(const LocalMap* local_map_reference_, const TransformMatrix3D& transform_query_to_reference_) {_closures.push_back(Closure(local_map_reference_, transform_query_to_reference_));}

    //ds returns all active loop closures for this local map
    const ClosureVector& closures() const {return _closures;}

  //ds attributes
  protected:

    //ds the contained Frames
    FramePointerVector _frames;

    //ds landmarks in the configuration at the time of the creation of the local map
    Landmark::StatePointerVector _landmarks;

    //ds one merged pool of all corresponding landmark appearances
    Landmark::AppearancePointerVector _appearances;

    //ds loop closures for the local map
    ClosureVector _closures;

    //ds grant access to local map producer
    friend WorldMap;

    //ds informative only
    const Count _minimum_number_of_landmarks = 50;
  };

  typedef std::vector<LocalMap*> LocalMapPointerVector;
}
