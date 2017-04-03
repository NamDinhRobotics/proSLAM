#pragma once
#include "relocalization/correspondence_collection.h"
#include "base_aligner.h"

namespace proslam {

  //ds this class specifies an aligner for camera centric point clouds (used to compute the spatial relation between local maps for a loop closure)
  class XYZAligner: public BaseAligner6_3 {

    //ds object handling
    public:

      XYZAligner(): BaseAligner6_3(1e-5, 0.05) {}
      ~XYZAligner() {}

    //ds required interface
    public:

      //ds initialize aligner with minimal entity
      void init(CorrespondenceCollection* context_, const TransformMatrix3D& current_to_reference_ = TransformMatrix3D::Identity());

      //ds linearize the system: to be called inside oneRound
      void linearize(const bool& ignore_outliers_);

      //ds solve alignment problem for one round: to be called inside converge
      void oneRound(const bool& ignore_outliers_);

      //ds solve alignment problem until convergence is reached
      void converge();

    //ds getters/setters
    public:

      const TransformMatrix3D& currentToReference() const {return _current_to_reference;}
      void setMinimumNumberOfInliers(const Count& minimum_number_of_inliers_) {_minimum_number_of_inliers = minimum_number_of_inliers_;}
      void setMinimumInlierRatio(const real& minimum_inlier_ratio_) {_minimum_inlier_ratio = minimum_inlier_ratio_;}

    //ds aligner specific
    protected:

      //ds context
      CorrespondenceCollection* _context = 0;

      //ds objective
      TransformMatrix3D _current_to_reference = TransformMatrix3D::Identity();

      //ds validation criteria
      Count _minimum_number_of_inliers = 100;
      real _minimum_inlier_ratio       = 0.75;
  };
}
