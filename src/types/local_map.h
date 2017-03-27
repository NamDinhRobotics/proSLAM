#pragma once
#include "landmark.h"

namespace proslam {

  //ds this class
  class LocalMap: public Frame {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

      //ds loop closure constraint element
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

    //ds owned by world map
    LocalMap(Frame* frame_for_context_, FramePointerVector& frames_);

    //ds cleanup of dynamic structures
    virtual ~LocalMap();

    //ds prohibit default construction
    LocalMap() = delete;

  //ds getters/setters
  public:

    virtual void setRobotToWorld(const TransformMatrix3D& robot_to_world_);
    const Landmark::AppearancePtrVector& appearances() const {return _appearances;}
    const Landmark::StatePointerVector& landmarks() const {return _landmarks;}
    void add(const LocalMap* local_map_reference_, const TransformMatrix3D& transform_query_to_reference_) {_closures.push_back(Closure(local_map_reference_, transform_query_to_reference_));}
    const ClosureVector& closures() const {return _closures;}

  //ds attributes
  protected:

    Landmark::AppearancePtrVector _appearances;
    ClosureVector _closures;
    Landmark::StatePointerVector _landmarks;
    FramePointerVector _frames;

    //ds grant access to local map producer
    friend WorldMap;

    //ds informative only
    const Count _minimum_number_of_landmarks = 50;

  };

  typedef std::vector<LocalMap*> LocalMapPointerVector;
}
