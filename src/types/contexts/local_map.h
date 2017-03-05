#pragma once
#include "../items/landmark.h"
#include "../items/landmark_item.h"

namespace proslam {

  class KeyFrame;
  typedef std::vector<KeyFrame*> KeyFramePtrVector;

  class KeyFrame: public Frame {

  //ds exported objects
  public:

      struct TransformMatrix3DWithInformation {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        TransformMatrix3DWithInformation(const TransformMatrix3D& transform_,
                                         const Matrix6& information_): transform(transform_),
                                                                       information(information_) {}

        const TransformMatrix3D transform;
        const Matrix6 information;
      };
      struct KeyFrameCorrespondence {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        KeyFrameCorrespondence(const KeyFrame* keyframe_,
                               const TransformMatrix3DWithInformation& relation_): keyframe(keyframe_),
                                                                                   relation(relation_) {}

        const KeyFrame* keyframe = 0;
        const TransformMatrix3DWithInformation relation;
      };
      typedef std::vector<KeyFrameCorrespondence> KeyFrameCorrespondenceVector;

  //ds object handling
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    KeyFrame(Frame* frame_for_context_, FramePtrVector& frames_);
    virtual ~KeyFrame();
    KeyFrame() = delete;

  public:

    //ds implement subcontext update
    virtual void updateSubContext();

  public:

    //ds descriptor tracks for all landmarks contained in the given frame
    const TransformMatrix3D& spatials() const {return robotToWorld();}
    const AppearancePtrVector& appearances() const {return _appearances;}
    const LandmarkItemPointerVector& items() const {return _items;}

    void add(const KeyFrame* keyframe_reference_,
             const TransformMatrix3D transform_query_to_reference_,
             const Matrix6& information_ = Matrix6::Identity()) {_matches.push_back(KeyFrameCorrespondence(keyframe_reference_, TransformMatrix3DWithInformation(transform_query_to_reference_, information_)));}
    const KeyFrameCorrespondenceVector& closures() const {return _matches;}

  protected:

    AppearancePtrVector _appearances;
    KeyFrameCorrespondenceVector _matches;
    LandmarkItemPointerVector _items;
    FramePtrVector _subcontext;
    const Count _minimum_number_of_items = 100;

  };
}
