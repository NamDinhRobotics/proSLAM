#pragma once
#include "correspondence_collection.h"
#include "base_aligner.h"

namespace proslam {

  class XYZAligner: public BaseAligner6_3 {
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds object handling
    protected:

      //ds instantiation controlled by aligner factory
      XYZAligner(): BaseAligner6_3(1e-5, 0.05) {}
      ~XYZAligner() {}

    //ds required interface
    public:

      //ds initialize aligner with minimal entity TODO purify this
      void init(BaseContext* context_, const TransformMatrix3D& current_to_reference_ = TransformMatrix3D::Identity());

      //ds linearize the system: to be called inside oneRound
      void linearize(const bool& ignore_outliers_);

      //ds solve alignment problem for one round: to be called inside converge
      void oneRound(const bool& ignore_outliers_);

      //ds solve alignment problem until convergence is reached
      void converge();

    //ds additional accessors
    public:

      //ds getters/setters
      const TransformMatrix3D currentToReference() const {return _current_to_reference;}
      void setMinimumNumberOfInliers(const Count& minimum_number_of_inliers_) {_minimum_number_of_inliers = minimum_number_of_inliers_;}
      void setMinimumInlierRatio(const real& minimum_inlier_ratio_) {_minimum_inlier_ratio = minimum_inlier_ratio_;}
      void setContextDepth(const Count& context_depth_) {_context_depth = context_depth_;}
      const Count& contextDepth() const {return _context_depth;}

    //ds aligner specific
    protected:

      //ds context
      CorrespondenceCollection* _context = 0;

      //ds objective
      TransformMatrix3D _current_to_reference = TransformMatrix3D::Identity();

      //ds validation criteria
      Count _minimum_number_of_inliers = 100;
      real _minimum_inlier_ratio    = 0.75;

      //ds enable deep correspondences -> landmarks, landmark views, etc..
      Count _context_depth = 0;

    //ds grant access to factory: ctor/dtor
    friend AlignerFactory;

  };
}
