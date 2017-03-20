#pragma once
#include "landmark.h"

namespace proslam {

  class LocalMap;
  typedef std::vector<LocalMap*> LocalMapPointerVector;

  class LocalMap: public Frame {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds exported types
  public:

      struct TransformMatrix3DWithInformation {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TransformMatrix3DWithInformation(const TransformMatrix3D& transform_,
                                         const Matrix6& information_): transform(transform_),
                                                                       information(information_) {}

        const TransformMatrix3D transform;
        const Matrix6 information;
      };
      struct LocalMapCorrespondence {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LocalMapCorrespondence(const LocalMap* keyframe_,
                               const TransformMatrix3DWithInformation& relation_): keyframe(keyframe_),
                                                                                   relation(relation_) {}

        const LocalMap* keyframe = 0;
        const TransformMatrix3DWithInformation relation;
      };
      typedef std::vector<LocalMapCorrespondence> KeyFrameCorrespondenceVector;

  //ds object handling
  protected:

    //ds owned by world map
    LocalMap(Frame* frame_for_context_, FramePtrVector& frames_);
    virtual ~LocalMap();
    LocalMap() = delete;

  public:

    //ds implement subcontext update
    virtual void updateSubContext();

  public:

    //ds descriptor tracks for all landmarks contained in the given frame
    const TransformMatrix3D& spatials() const {return robotToWorld();}
    const Landmark::AppearancePtrVector& appearances() const {return _appearances;}
    const Landmark::ItemPointerVector& items() const {return _items;}

    void add(const LocalMap* keyframe_reference_,
             const TransformMatrix3D transform_query_to_reference_,
             const Matrix6& information_ = Matrix6::Identity()) {_matches.push_back(LocalMapCorrespondence(keyframe_reference_, TransformMatrix3DWithInformation(transform_query_to_reference_, information_)));}
    const KeyFrameCorrespondenceVector& closures() const {return _matches;}

  protected:

    Landmark::AppearancePtrVector _appearances;
    KeyFrameCorrespondenceVector _matches;
    Landmark::ItemPointerVector _items;
    FramePtrVector _subcontext;
    static constexpr Count _minimum_number_of_items = 100;

  //ds grant access to local map producer
  friend WorldMap;

  };
}
